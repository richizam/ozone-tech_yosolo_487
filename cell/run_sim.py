# -*- coding: utf-8 -*-
"""Cell simulation entrypoint — the closed loop, end to end (MuJoCo twin of
isaac/run_isaac.py on the tilt-tray sorter executive).

    python -m cell.run_sim --scenario scenarios/base.yaml --seed 42
    python -m cell.run_sim --viewer          (watch it live)

Flow (freight motion = powered-surface drive + tray contact + gravity):
  * belt A (1 m/s, FIXED) carries items through the vision station; the
    pre-gate hold keeps a single item in the measurement window; multi-read
    fusion + guard-banded official rule order produce the B/C/D verdict
    BEFORE the item reaches the escapement;
  * the normally-closed escapement releases the item synchronized to an
    inbound EMPTY tray; the item rides off the belt-A knife nose and lands
    on the moving tray (landing offset measured per item);
  * the carrier train (kinematic chain joints; hinge-actuated tilt trays)
    carries each item to its route's discharge station: C/D tilt south onto
    32-deg brake chutes into the roll-cages, B tilts north onto a powered
    incline connector feeding the FIXED belt B, REVIEW tilts north into the
    manual-review pen (double occupancy / discharge-miss fallback);
  * jam watchdog: no displacement over the timeout window -> exception arm
    recovers C/D chute snags route-correct into their own cage; anything it
    cannot reach safely is an operator call-out. A stuck tray needs no arm:
    its freight rides to the REVIEW station / end-line call-out by design.

Writes runs/<stamp>/{events.csv, items.csv, summary.json, actuator_log.csv}
and exits non-zero if any item was misrouted, dropped or escaped.
"""
import argparse
import datetime
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import yaml  # noqa: E402

from cell import params as P  # noqa: E402
from cell.belt import Belts  # noqa: E402
from cell.bus import Bus  # noqa: E402
from cell.controller import Controller  # noqa: E402
from cell.metrics import Metrics  # noqa: E402
from cell.scene import make_model  # noqa: E402
from cell.sorter import SorterControl  # noqa: E402
from cell.visuals import Visuals  # noqa: E402


class Recorder:
    """Offscreen MP4 capture of a named scene camera (--record)."""

    def __init__(self, model, path, camera="overview", fps=30, size=(1280, 720)):
        import cv2
        import mujoco
        self.cv2 = cv2
        self.renderer = mujoco.Renderer(model, height=size[1], width=size[0])
        self.camera = camera
        self.dt_frame = 1.0 / fps
        self.next_t = 0.0
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                                      fps, size)
        self.path = path

    def maybe_capture(self, data, t):
        if t < self.next_t:
            return
        self.next_t = t + self.dt_frame
        self.renderer.update_scene(data, camera=self.camera)
        frame = self.renderer.render()
        self.writer.write(self.cv2.cvtColor(frame, self.cv2.COLOR_RGB2BGR))

    def close(self):
        self.writer.release()
        self.renderer.close()


class ItemManager:
    """Spawning, classification (oracle or camera), induction hand-off to the
    sorter, delivery adjudication and post-delivery containment watch."""

    CAM_WINDOW = P.VIRTUAL_SENSOR["window_x"]    # classify while the item center
                                                 # crosses this (ends before the
                                                 # escapement gate)

    def __init__(self, model, data, manifest, order, gaps, bus, ev,
                 perception_latency, perceiver=None, spawn_rng=None,
                 belts=None, cls_cfg=None):
        self.m, self.d, self.bus, self.ev = model, data, bus, ev
        self.entries = {e["slug"]: e for e in manifest}
        self.queue = list(order)                   # slugs to spawn
        self.gaps = list(gaps)
        self.next_spawn_t = 1.0
        self.latency = perception_latency
        self.perceiver = perceiver                 # None -> oracle mode
        self.spawn_rng = spawn_rng
        self.belts = belts                         # for gate-hold accounting
        self.sorter = None                         # bound after construction
        self.cls_cfg = dict(P.CLASSIFICATION, **(cls_cfg or {}))
        self.proc_latency = P.VIRTUAL_SENSOR["processing_latency_s"]
        self.watch_zones = dict(P.cages_for())
        self.watch_zones["REVIEW"] = P.REVIEW_PEN
        self.active = {}                           # slug -> state dict
        self.done = {}                             # slug -> zone_actual
        self.routes = {}                           # slug -> carrier route
        self.frozen = set()                        # injected snags (fault drill)
        self.frozen_pose = {}
        self.vwrites = {"nominal_set_v": 0, "fault_injection": 0}
        self.cage_watch = {}                       # slug -> containment state
        self.window_conflicts = 0                  # events: >=2 items co-occupied
        self._window_multi = False                 # the measurement window

    def _adr(self, slug):
        jid = self.m.joint(f"fj_{slug}").id
        return self.m.jnt_qposadr[jid], self.m.jnt_dofadr[jid]

    def pose(self, slug):
        qadr, _ = self._adr(slug)
        return self.d.qpos[qadr:qadr + 3]

    def vel(self, slug):
        _, dadr = self._adr(slug)
        return self.d.qvel[dadr:dadr + 3]

    def item_pos_of(self, slug):
        """Position hook for the sorter's discharge confirmation."""
        if slug in self.active:
            return np.array(self.pose(slug))
        return None

    def spawn_zone_clear(self):
        for slug in self.active:
            x, y, _ = self.pose(slug)
            if x < 1.2 and abs(y - P.BELT_A["y"]) < 0.4:
                return False
        return True

    def step(self, t, dt=0.0):
        a = P.BELT_A
        # spawn
        if self.queue and t >= self.next_spawn_t and self.spawn_zone_clear():
            slug = self.queue.pop(0)
            e = self.entries[slug]
            qadr, dadr = self._adr(slug)
            # random yaw where the geometry allows it (big items arrive
            # pre-aligned by the upstream infeed, as in the CAD layout note)
            yaw = 0.0
            if self.spawn_rng is not None:
                diag = float(np.hypot(e["dims_m"][0], e["dims_m"][1]))
                yaw = (float(self.spawn_rng.uniform(-0.09, 0.09)) if diag > 0.48
                       else float(self.spawn_rng.uniform(0, 2 * np.pi)))
            self.d.qpos[qadr:qadr + 3] = [0.4, a["y"], a["top"] + e["dims_m"][2] / 2 + 0.003]
            self.d.qpos[qadr + 3:qadr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
            self.d.qvel[dadr:dadr + 6] = 0
            self.active[slug] = {"classified": False,
                                 "classify_t": t + self.latency}
            self.bus.publish("item_spawned", t=t, slug=slug,
                             asset=e.get("source", e.get("file", slug)),
                             zone_true=e["zone"])
            self.next_spawn_t = t + (self.gaps.pop(0) if self.gaps else 6.0)

        # single-item discipline check: how many items are INSIDE the window?
        # (the pre-gate hold exists to keep this at <=1; violations are counted)
        in_window = sum(1 for s in self.active
                        if self.CAM_WINDOW[0] <= float(self.pose(s)[0]) <= self.CAM_WINDOW[1])
        if in_window >= 2 and not self._window_multi:
            self.window_conflicts += 1
        self._window_multi = in_window >= 2

        for slug, st in list(self.active.items()):
            if st.get("recovering"):            # carried by the arm
                continue
            e = self.entries[slug]
            pos = self.pose(slug)
            vel = self.vel(slug)
            # detection-zone entry: cycle_start for BOTH perception modes
            if "t_detected" not in st and pos[0] >= self.CAM_WINDOW[0]:
                st["t_detected"] = t
                self.bus.publish("item_detected", t=t, slug=slug)
            # escapement/pre-gate holds: accumulated per item (spacing control;
            # the BELT never stops — see conveyor_a_stop_count)
            if self.belts is not None and dt > 0.0 and slug not in self.frozen:
                front = pos[0] + e["dims_m"][0] / 2
                if (not self.belts.gate_open
                        and a["gate_x"] - 0.05 <= front < a["gate_x"] + 0.05):
                    st["gate_hold_s"] = st.get("gate_hold_s", 0.0) + dt
                if (not self.belts.hold2_open
                        and a["hold2_x"] - 0.05 <= front < a["hold2_x"] + 0.05):
                    st["hold2_hold_s"] = st.get("hold2_hold_s", 0.0) + dt
            # ---- classification (existing twin sensor path, window per params)
            self._step_classification(slug, st, e, pos, t)
            # ---- induction: offer the head item to the sorter
            if (self.sorter is not None and st.get("classified")
                    and "t_released" not in st and slug not in self.frozen
                    and self.CAM_WINDOW[1] - 0.05 < pos[0] < a["nose_x"]
                    and abs(pos[1] - a["y"]) < 0.35):
                dims_meas = (st.get("cls") or {}).get("dims_mm")
                L_meas = (dims_meas[0] / 1000.0) if dims_meas else                     self.entries[slug]["dims_m"][0]
                self.sorter.offer(t, slug, st["zone"], float(pos[0]),
                                  vx=float(vel[0]), ready=True,
                                  length_m=L_meas)
            # ---- landing confirmation (tag the carrier)
            if ("t_released" in st and "t_inducted" not in st
                    and pos[0] > a["nose_x"] - 0.10):
                if os.environ.get("TWIN_TRACE") and int(t * 5) != int(
                        (t - 0.02) * 5):
                    cars_s = " ".join(
                        f"c{c['i']}:{c['x']:.2f}{'*' if c['slug'] else ''}"
                        for c in self.sorter.cars if c["leg"] == "top")
                    print(f"[twin] {slug} t={t:7.2f} pos=({pos[0]:.3f},"
                          f"{pos[1]:.3f},{pos[2]:.3f}) {cars_s}", flush=True)
                idx = self.sorter.confirm_landing(t, slug, pos)
                if idx is not None:
                    self.routes[slug] = self.sorter.cars[idx]["route"]
                    st["routed_t"] = t
                    st["watch_xy"] = (float(pos[0]), float(pos[1]))
                    st["watch_t"] = t
                    self.bus.publish("routing_cmd", t=t, slug=slug,
                                     zone=self.routes[slug], carrier=idx)
            # end-of-line operator call-out (dead tilt / unresolvable freight)
            if st.get("end_callout") and not st.get("end_handled"):
                st["end_handled"] = True
                self._deliver(slug, "MANUAL", t)
                continue
            # ---- deliveries
            if pos[1] > P.BELT_B["y_delivered"] and abs(pos[0] - P.BELT_B["cx"]) < 0.3:
                self._deliver(slug, "B", t)
                continue
            dest = self._which_dest(pos)
            if dest is not None and pos[2] < (0.42 if dest == "REVIEW" else 0.56):
                self._deliver(slug, dest, t)
                continue
            if (pos[2] < 0.22 and dest is None and pos[1] > 0
                    and not (abs(pos[0] - 0.4) < 0.3 and pos[1] < 0)):
                self._deliver(slug, "FLOOR", t)
                continue
            # ---- jam watchdog (operator call-out / arm recovery, via run_sim)
            if "routed_t" in st and not st.get("jam_reported"):
                if t - st["watch_t"] >= P.JAM_TIMEOUT_S:
                    moved = float(np.hypot(pos[0] - st["watch_xy"][0],
                                           pos[1] - st["watch_xy"][1]))
                    st["watch_xy"] = (float(pos[0]), float(pos[1]))
                    st["watch_t"] = t
                    if moved < P.JAM_MIN_PROGRESS_M:
                        st["jam_reported"] = True
                        st["jam_pending"] = True
                        self.ev(t, "jam_detected", slug, zone=st.get("zone", ""),
                                pos=[round(float(v), 3) for v in pos])
            elif "routed_t" not in st:
                # watchdog progress reset while commanded-held at a blade
                st["watch_xy"] = (float(pos[0]), float(pos[1]))
                st["watch_t"] = t
        self._watch_containment(t)

    # ---------------------------------------------------------- classification
    def _step_classification(self, slug, st, e, pos, t):
        cfg = self.cls_cfg
        if not st["classified"] and "pending_cls" in st:
            # pending verdict: published after the modeled processing latency
            if t >= st["pending_cls"]["t_ready"]:
                self._publish_classification(slug, t)
        elif not st["classified"]:
            if self.perceiver is None:
                # oracle mode: ground truth stamped after a model latency
                if "t_detected" in st and t >= st["classify_t"]:
                    st["classified"] = True
                    st["zone"] = e["zone"]
                    st["t_route_cmd"] = t
                    dims_mm = [round(d * 1000, 1) for d in e["dims_m"]]
                    over = bool(np.any(np.sort(dims_mm)[::-1] > np.array(P.LIMIT_MAX_MM)))
                    under = bool(np.any(np.array(dims_mm) < P.LIMIT_MIN_MM))
                    rule_dim = "undersize" if under else ("oversize" if over else "pass")
                    self.bus.publish("item_classified", t=t, slug=slug,
                                     zone=e["zone"], zone_true=e["zone"],
                                     zone_raw=e["zone"], confidence=1.0, flags="",
                                     dims_mm=dims_mm, n_reads=0,
                                     latency_ms=round(self.latency * 1000, 1),
                                     rule_dim=rule_dim,
                                     rule_circ="ground_truth")
            elif "t_detected" in st and pos[0] > self.CAM_WINDOW[1]:
                # escaped the window between captures: commit what we have
                # (no reads at all -> sensor_miss -> manual stream)
                self._commit_classification(slug, e, st.get("cls_reads") or [], t)
            elif (self.CAM_WINDOW[0] <= pos[0] <= self.CAM_WINDOW[1]
                  and self._front_unclassified() == slug):
                # multi-capture evidence for the FRONT-most unclassified
                # item only (the pre-gate hold keeps followers upstream);
                # verdicts fuse per the official decision order and shape
                # policies fire only on persistent evidence
                if t >= st.get("next_capture_t", 0.0):
                    st["next_capture_t"] = t + P.VIRTUAL_SENSOR["capture_period_s"]
                    st.setdefault("t_capture_start", t)
                    st["t_capture_end"] = t
                    # tailgater proximity: a follower within 8 cm of this
                    # item's rear can merge into its cloud — remember it
                    # (the fused verdict then may not commit to B)
                    rear = pos[0] - e["dims_m"][0] / 2
                    for other in self.active:
                        if other == slug:
                            continue
                        op = self.pose(other)
                        if abs(op[1] - P.BELT_A["y"]) > 0.4:
                            continue
                        ofront = op[0] + self.entries[other]["dims_m"][0] / 2
                        if ofront <= pos[0] + 0.01 and rear - ofront < 0.08:
                            st["window_conflict"] = True
                    res = self.perceiver.classify(self.d, x_hint=float(pos[0]))
                    reads = st.setdefault("cls_reads", [])
                    if res is not None:
                        reads.append(res)
                    leaving = pos[0] > self.CAM_WINDOW[1] - 0.06
                    # commit once the pose has stabilised (two consecutive
                    # agreeing reads after >=N) — rocking items keep being
                    # read while moving, up to a hard cap
                    stable = False
                    if len(reads) >= 2:
                        r1, r2 = reads[-2], reads[-1]
                        stable = (r1["zone"] == r2["zone"]
                                  and r1.get("dims_mm") and r2.get("dims_mm")
                                  and max(abs(a_ - b_) for a_, b_ in
                                          zip(r1["dims_mm"], r2["dims_mm"]))
                                  < cfg["stable_dims_tol_mm"])
                    if ((len(reads) >= cfg["stable_reads_required"] and stable)
                            or len(reads) >= cfg["read_cap"]
                            or (leaving and reads)):
                        self._commit_classification(slug, e, reads, t)

    def _commit_classification(self, slug, e, reads, t):
        """Fuse the window's reads following the OFFICIAL decision order:
        dimensions first (median over reads — robust to transient tilt), then
        circle-in-section, where a corroborated detection in ANY read counts
        (the rule is existential: 'circle in any section') and the weak
        isolated-circle policy fires only when persistent across reads.

        The verdict is stamped `t_ready = t + processing_latency_s` and
        published then — the route command is generated after the modeled
        fusion/inference time, while the item keeps moving."""
        st = self.active[slug]
        cfg = self.cls_cfg
        flags = []
        if reads:
            # stable-tail rejection (standard DWS practice): keep the longest
            # run of mutually consistent trailing reads — transient frames
            # (e.g. a second item clipping the window during a queue release)
            # disagree by centimetres and are discarded
            tail = [reads[-1]]
            for r in reversed(reads[:-1]):
                if (r.get("dims_mm") and tail[-1].get("dims_mm")
                        and max(abs(a - b) for a, b in
                                zip(r["dims_mm"], tail[-1]["dims_mm"])) < 25.0):
                    tail.append(r)
                else:
                    break
            if 2 <= len(tail) < len(reads):
                flags.append("unstable_reads_rejected")
                reads = list(reversed(tail))
        low_conf, fb_reason = False, None
        if not reads:
            zone_raw = zone = cfg["low_confidence_route"]   # sensor miss: manual stream
            rep = {"confidence": 0.0, "max_ratio": None, "dims_mm": None}
            conf, minor = 0.0, None
            rule_dim, rule_circ = "no_measurement", "no_measurement"
            low_conf, fb_reason = True, "sensor_miss"
            flags.append("sensor_miss")
        else:
            rep = max(reads, key=lambda r: (r.get("max_ratio") is not None,
                                            r.get("max_ratio") or 0))
            dims = np.median(np.array([r["dims_mm"] for r in reads if r.get("dims_mm")]),
                             axis=0)
            minors = [r["minor_mm"] for r in reads if r.get("minor_mm") is not None]
            minor = float(np.median(minors)) if minors else None
            conf = float(np.median([r.get("confidence", 0.0) for r in reads]))
            undersize = bool(np.any(dims < P.LIMIT_MIN_MM)
                             or (minor is not None and minor < P.LIMIT_MIN_MM))
            oversize = bool(np.any(np.sort(dims)[::-1] > np.array(P.LIMIT_MAX_MM)))
            circ_strong = any(r.get("circular") for r in reads)
            n_relaxed = sum(1 for r in reads if r.get("circular_relaxed"))
            n_isolated = sum(1 for r in reads if "isolated_end_circle" in r.get("flags", []))
            weak_persistent = (n_isolated * 2 >= len(reads)) or (n_relaxed >= 2)
            # guard bands (legal-metrology practice): a measurement within the
            # sensor's uncertainty of a decision threshold cannot support the
            # PERMISSIVE outcome — divert to the safe side. Bands scale with
            # the configured sensor noise and vanish at the ideal baseline.
            noise_mm = (self.perceiver.noise_m * 1000.0
                        if self.perceiver is not None else 0.0)
            # certification floor (sampling physics, present at zero noise): a
            # dimension the sensor cannot resolve better than ~2x its ground
            # sampling cannot certify "> 10 mm" — divert to C, per the rules'
            # priority (a maybe-undersize item never feeds the sorter)
            g_dim = 2.0 * noise_mm + 2.0 * P.VIRTUAL_SENSOR["ground_res_mm"]
            dim_suspect = False
            if not (undersize or oversize):
                mins = np.concatenate([dims, [] if minor is None else [minor]])
                near_under = bool(np.any(mins < P.LIMIT_MIN_MM + g_dim))
                near_over = bool(np.any(
                    np.sort(dims)[::-1] > np.array(P.LIMIT_MAX_MM)
                    - max(2.0 * noise_mm, 1e-9)))
                dim_suspect = near_under or near_over
            ratio_suspect = False
            max_ratio = rep.get("max_ratio")
            if not circ_strong and max_ratio is not None:
                # ratio uncertainty = (noise + sampling resolution) over the
                # cross-section circumradius (the two smaller extents span the
                # section). The resolution term never vanishes: at 3 mm ground
                # sampling a 73 mm-radius pentagon (true cos36 = 0.809)
                # measured 0.799 — one part in a thousand below the threshold
                # cannot certify "not circular" (found by stress seed 3)
                srt = np.sort(dims)[::-1]
                r_est = float(np.hypot(srt[1], srt[2])) / 2.0
                g_ratio = min(0.25, (2.0 * noise_mm
                                     + P.VIRTUAL_SENSOR["ground_res_mm"])
                              / max(r_est, 5.0))
                ratio_suspect = P.CIRCLE_RATIO - g_ratio <= max_ratio < P.CIRCLE_RATIO
            rule_dim = "undersize" if undersize else ("oversize" if oversize else "pass")
            rule_circ = ("strong" if circ_strong
                         else "weak_persistent" if weak_persistent else "none")
            if undersize or oversize:
                zone_raw = zone = "C"
            elif dim_suspect:
                zone_raw = zone = "C"          # guard band: may violate dims
                low_conf, fb_reason = True, "dims_within_noise_of_limit"
                flags.append("guard_band_dims")
            elif circ_strong:
                zone_raw = zone = "D"
            elif ratio_suspect:
                zone_raw, zone = "B", "D"      # guard band: may be circular
                low_conf, fb_reason = True, "ratio_within_noise_of_threshold"
                flags.append("guard_band_ratio")
            elif weak_persistent and cfg["weak_evidence_reroute"]:
                zone_raw, zone = "B", "D"      # policy: uncertain shape -> repack
                low_conf, fb_reason = True, "weak_circle_evidence"
                flags.append("policy_reroute_D")
            else:
                zone_raw = zone = "B"
            # low-confidence safe fallback: an uncertain item NEVER goes to B
            if zone == "B" and conf < cfg["min_confidence_for_B"]:
                zone = cfg["low_confidence_route"]
                low_conf, fb_reason = True, "confidence_below_threshold"
                flags.append("low_confidence_fallback")
            # single-item discipline defense-in-depth: if a tailgater came
            # within cloud-merge range during measurement, this verdict may be
            # a merged-object artifact — it must not feed the sorter
            if zone == "B" and st.get("window_conflict"):
                zone = cfg["low_confidence_route"]
                low_conf, fb_reason = True, "tailgater_during_measurement"
                flags.append("window_conflict")
            rep = dict(rep, dims_mm=[round(float(d), 1) for d in dims])
            flags = list(rep.get("flags", [])) + flags
            if os.environ.get("SIM_DEBUG_CLS") and zone != e["zone"]:
                print(f"[cls] {slug} true={e['zone']} got={zone} dims={rep['dims_mm']} "
                      f"minor={minor} n={len(reads)} "
                      f"all_dims={[r.get('dims_mm') for r in reads]}")
        st["pending_cls"] = {
            "t_ready": t + self.proc_latency,
            "payload": dict(
                zone=zone, zone_true=e["zone"], zone_raw=zone_raw,
                confidence=round(conf, 3), ratio=rep.get("max_ratio"),
                dims_mm=rep.get("dims_mm"), n_reads=len(reads),
                rule_dim=rule_dim, rule_circ=rule_circ,
                low_confidence=low_conf, fallback_reason=fb_reason,
                flags=";".join(flags)),
        }

    def _publish_classification(self, slug, t):
        """Route command generation: the fused verdict becomes the command."""
        st = self.active[slug]
        payload = st.pop("pending_cls")["payload"]
        st["classified"] = True
        st["zone"] = payload["zone"]
        st["t_route_cmd"] = t
        latency_ms = None
        if "t_capture_start" in st:
            latency_ms = round((t - st["t_capture_start"]) * 1000, 1)
        self.bus.publish("item_classified", t=t, slug=slug,
                         t_capture_start=st.get("t_capture_start"),
                         t_capture_end=st.get("t_capture_end"),
                         latency_ms=latency_ms, **payload)

    # ------------------------------------------------------------- deliveries
    def _which_dest(self, pos):
        """Destination whose inner footprint contains pos (isaac which_dest)."""
        for zone, cage in self.watch_zones.items():
            cx, cy = cage["center"]
            ix, iy = cage["inner"]
            if abs(pos[0] - cx) < ix / 2 + 0.03 and abs(pos[1] - cy) < iy / 2 + 0.03:
                return zone
        return None

    def _deliver(self, slug, zone_actual, t):
        st = self.active.pop(slug)
        e = self.entries[slug]
        qadr, dadr = self._adr(slug)
        v_entry = float(np.linalg.norm(self.d.qvel[dadr:dadr + 3]))
        if self.sorter is not None:
            self.sorter.clear_item(slug)
        self.routes.pop(slug, None)
        ok = zone_actual == e["zone"]
        self.done[slug] = zone_actual
        if zone_actual == "MANUAL":
            # operator removal to the manual-handling station (machine-part
            # action, mirrors isaac deliver(): not a nominal freight write)
            self.frozen.discard(slug)
            self.frozen_pose.pop(slug, None)
            n_manual = sum(1 for z in self.done.values() if z == "MANUAL")
            mx, my = P.MANUAL_STATION
            self.d.qpos[qadr:qadr + 3] = [mx, my + 0.45 * (n_manual - 1), 0.15]
            self.d.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
            self.d.qvel[dadr:dadr + 6] = 0
            self.ev(t, "operator_removed", slug, station=[mx, my])
        # delivery into a walled destination is not the end of the story: the
        # item is tracked until the run ends — it must SETTLE and STAY inside
        if zone_actual in self.watch_zones:
            self.cage_watch[slug] = {"zone": zone_actual, "t_enter": t,
                                     "v_entry": v_entry, "violated": False,
                                     "t_settle": None, "max_z": 0.0}
        # park delivered B-items back off-cell so contacts stay cheap
        if zone_actual == "B":
            idx = list(self.entries).index(slug)
            self.d.qpos[qadr:qadr + 3] = [0.6 + idx * 0.85, -2.5, e["dims_m"][2] / 2 + 0.001]
            self.d.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
            self.d.qvel[dadr:dadr + 6] = 0
        self.bus.publish("item_delivered", t=t, slug=slug, zone=zone_actual, ok=ok,
                         zone_true=e["zone"], v_entry=round(v_entry, 3),
                         route_command=st.get("zone"),
                         gate_hold_s=round(st.get("gate_hold_s", 0.0), 2),
                         hold2_hold_s=round(st.get("hold2_hold_s", 0.0), 2))

    # ------------------------------------------------- containment validation
    def _watch_containment(self, t):
        """Routed != done: a delivered C/D/REVIEW item is watched until the
        run ends. Leaving the destination footprint (or flying above it) is a
        violation (same envelope math as isaac/run_isaac.py)."""
        for slug, w in self.cage_watch.items():
            cage = self.watch_zones[w["zone"]]
            qadr, dadr = self._adr(slug)
            pos = self.d.qpos[qadr:qadr + 3]
            speed = float(np.linalg.norm(self.d.qvel[dadr:dadr + 3]))
            cx, cy = cage["center"]
            hx = cage["inner"][0] / 2 + cage["wall_t"] + P.CONTAIN["margin"]
            hy = cage["inner"][1] / 2 + cage["wall_t"] + P.CONTAIN["margin"]
            w["max_z"] = max(w["max_z"], float(pos[2]))
            outside = (abs(pos[0] - cx) > hx or abs(pos[1] - cy) > hy
                       or pos[2] > P.CONTAIN["z_fly"])
            if outside and not w["violated"]:
                w["violated"] = True
                self.ev(t, "containment_violation", slug, zone=w["zone"],
                        pos=[round(float(v), 3) for v in pos])
            if w["t_settle"] is None:
                if speed < P.CONTAIN["settle_speed"]:
                    w.setdefault("low_since", t)
                    if t - w["low_since"] >= P.CONTAIN["settle_time"]:
                        w["t_settle"] = t
                else:
                    w.pop("low_since", None)

    def finalize_containment(self, t):
        """End of run: emit the per-item containment verdicts."""
        for slug, w in self.cage_watch.items():
            settle_s = (round(w["t_settle"] - w["t_enter"], 2)
                        if w["t_settle"] is not None else None)
            self.bus.publish("containment_final", t=t, slug=slug, zone=w["zone"],
                             contained=not w["violated"], v_entry=w["v_entry"],
                             settle_s=settle_s, t_settle=w["t_settle"],
                             max_z=round(w["max_z"], 3))

    def _front_unclassified(self):
        """Slug of the most-downstream unclassified item inside the window."""
        best, best_x = None, -1e9
        for slug, st in self.active.items():
            if st["classified"]:
                continue
            x = float(self.pose(slug)[0])
            if self.CAM_WINDOW[0] <= x <= self.CAM_WINDOW[1] and x > best_x:
                best, best_x = slug, x
        return best

    def pin_frozen(self, carried=None):
        """Fault drill: hold the injected snag's POSE until the arm grasps it
        (velocity writes counted as fault_injection, mirror isaac)."""
        for slug in list(self.frozen):
            if slug not in self.active or slug == carried:
                continue
            if slug in self.frozen_pose:
                qadr, dadr = self._adr(slug)
                self.d.qpos[qadr:qadr + 7] = self.frozen_pose[slug]
                self.d.qvel[dadr:dadr + 6] = 0
                self.vwrites["fault_injection"] += 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", default=str(ROOT / "scenarios" / "base.yaml"))
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--viewer", action="store_true", help="open the MuJoCo viewer")
    ap.add_argument("--out", default=None)
    ap.add_argument("--perception", choices=["oracle", "camera"], default=None,
                    help="override the scenario's perception mode")
    ap.add_argument("--executive", default=None,
                    help="legacy flag: EXEC modes collapsed to 'sorter'")
    ap.add_argument("--record", nargs="?", const="demo.mp4", default=None,
                    help="capture an MP4 of the run (default file demo.mp4 in the run dir)")
    ap.add_argument("--camera", default="overview",
                    help="scene camera for --record (overview|top_view|routing|lookahead)")
    ap.add_argument("--fps", type=int, default=30, help="--record frame rate")
    args = ap.parse_args(argv)

    sc = yaml.safe_load(Path(args.scenario).read_text(encoding="utf-8"))
    seed = args.seed if args.seed is not None else sc.get("seed", 42)
    rng = np.random.default_rng(seed)

    mode = args.perception or sc.get("perception", "oracle")
    executive = "sorter"                     # single executive (tilt-tray)

    model, manifest, _ = make_model(mode=executive)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    slugs = []
    for name, count in sc["items"].items():
        if int(count) != 1:
            raise SystemExit(
                f"scenario item '{name}': count={count} unsupported — each item "
                f"is one physical body; volume comes from running multiple "
                f"seeds (see cell/validate.py and scenarios/stress_mix.yaml)")
        slugs += [name] * int(count)
    order = list(rng.permutation(slugs))
    gap_lo, gap_hi = sc.get("spawn_gap_s", [6.0, 8.0])
    gaps = list(rng.uniform(gap_lo, gap_hi, size=len(order)))

    sensor_cfg = dict(P.VIRTUAL_SENSOR, **(sc.get("sensor") or {}))
    perceiver = None
    sensor_model = "oracle_ground_truth"
    if mode == "camera":
        from perception.pipeline import LookaheadPerception
        perceiver = LookaheadPerception(
            model, noise_mm=sensor_cfg["depth_noise_mm"],
            noise_rng=np.random.default_rng(seed + 2000))
        sensor_model = sensor_cfg["model"]

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out) if args.out else ROOT / "runs" / f"{stamp}_seed{seed}_{mode}_{executive}"
    run_id = out_dir.name
    bus = Bus()
    metrics = Metrics(bus, out_dir)
    belts = Belts(model, manifest)

    def ev(t, event, slug="", **kv):
        """Isaac-style event hook: one log stream + per-item state stamps."""
        bus.publish("cell_event", t=t, event=event, slug=slug, **kv)
        st = items.active.get(slug)
        if st is not None:
            if event == "escapement_release":
                st["t_released"] = t
            elif event == "induction_landed":
                st["t_inducted"] = t
                st["carrier"] = kv.get("carrier")
            elif event == "end_line_callout":
                st["end_callout"] = True
            elif event == "double_occupancy":
                st["review_reason"] = "double_occupancy"
            elif event == "discharge_missed":
                st["review_reason"] = "discharge_miss"

    items = ItemManager(model, data, manifest, order, gaps, bus, ev,
                        perception_latency=sc.get("perception_latency_s", 0.15),
                        perceiver=perceiver,
                        spawn_rng=np.random.default_rng(seed + 1000),
                        belts=belts, cls_cfg=sc.get("classification"))

    # ---- the sorter executive
    sorter = SorterControl(model, data, ev, seed=seed,
                           dead_route=sc.get("inject_tray_fault"))
    items.sorter = sorter
    ctrl = Controller(model, data, bus, mode=executive)

    a = P.BELT_A
    ea_id = model.actuator("egate_a").id
    ha_id = model.actuator("hold2_a").id
    blade_state = {"hold2": False}

    def set_egate(up):
        data.ctrl[ea_id] = P.BELT_A["blade_up"] if up else 0.0

    def blade_clear_egate():
        for s2 in items.active:
            if s2 in belts.skip:
                continue
            p2 = items.pose(s2)
            if abs(p2[1] - a["y"]) > 0.45:
                continue
            hm2 = max(items.entries[s2]["dims_m"][0],
                      items.entries[s2]["dims_m"][1]) / 2
            if abs(p2[0] - a["gate_x"]) < hm2 + 0.05:
                return False
        return True

    def blade_clear_hold2():
        for s2 in items.active:
            if s2 in belts.skip:
                continue
            p2 = items.pose(s2)
            if abs(p2[1] - a["y"]) > 0.45:
                continue
            hm2 = max(items.entries[s2]["dims_m"][0],
                      items.entries[s2]["dims_m"][1]) / 2
            if abs(p2[0] - a["hold2_x"]) < hm2 + 0.05:
                return False
        return True

    def set_hold2(want_closed):
        if want_closed and not blade_state["hold2"] and not blade_clear_hold2():
            want_closed = False
        blade_state["hold2"] = want_closed
        data.ctrl[ha_id] = P.BELT_A["blade_up"] if want_closed else 0.0
        belts.hold2_open = not want_closed

    sorter.bind_escapement(set_egate, blade_clear_egate)   # normally closed
    belts.gate_open = not sorter.egate_up

    inject = sc.get("inject_jam")      # e.g. {slug: helmet, at_y: 2.6}
    # single-item-discipline + belt-state counters (реакция на нештатные потоки)
    flow_stats = {"spacing_gate_activations": 0,
                  "multi_object_window_events": 0,
                  "conveyor_a_stop_count": 0}

    # presentation layer: category tint, station lamps, beacons, andon tower
    vis = Visuals(model)
    bus.subscribe("item_classified",
                  lambda **m: vis.set_item_zone(m["slug"], m["zone"]))

    recorder = None
    if args.record:
        rec_path = Path(args.record)
        if not rec_path.is_absolute():
            out_dir.mkdir(parents=True, exist_ok=True)
            rec_path = out_dir / rec_path
        try:
            recorder = Recorder(model, rec_path, camera=args.camera, fps=args.fps)
        except Exception as exc:       # no GL context (bare CI box): run on
            print(f"WARNING: --record disabled ({exc})")

    recovery = {"active": None}

    def on_attached(**m):
        if m.get("event") == "attached":
            belts.skip.add(m["slug"])
            items.frozen.discard(m["slug"])
            items.frozen_pose.pop(m["slug"], None)
    bus.subscribe("cell_event", on_attached)

    def on_released(**m):
        if m.get("event") == "released":
            belts.skip.discard(m["slug"])
    bus.subscribe("cell_event", on_released)

    def on_job_end(**m):
        if m.get("event") in ("job_done", "job_abort") and m["slug"] == recovery["active"]:
            recovery["active"] = None
            sorter.station_hold.clear()
            st = items.active.get(m["slug"])
            if st is None:
                return
            st["recovering"] = False
            if m.get("event") == "job_done":
                # re-arm the watchdog: if the item STILL fails to arrive,
                # escalate again — at most twice, then flag for manual
                st["recoveries"] = st.get("recoveries", 0) + 1
                st["jam_reported"] = False
                pr = items.pose(m["slug"])
                st["watch_xy"] = (float(pr[0]), float(pr[1]))
                st["watch_t"] = m["t"]
                ev(m["t"], "recovery_done", m["slug"], ok=True,
                   attempts=st["recoveries"])
    bus.subscribe("cell_event", on_job_end)

    def on_abort(**m):
        if m.get("event") != "job_abort":
            return
        slug = m["slug"]
        belts.skip.discard(slug)
        st = items.active.get(slug)
        if st is None:
            return
        st["recovering"] = False
        if m.get("carrying"):
            # item dropped mid-carry: let the delivery detector judge the landing
            return
        st["attempts"] = st.get("attempts", 0) + 1
        if st["attempts"] >= 2:
            items._deliver(slug, "MANUAL", m["t"])   # give up: operator call-out
        else:
            st["jam_reported"] = False               # let the watchdog re-escalate
    bus.subscribe("cell_event", on_abort)

    viewer_ctx = None
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer_ctx = mj_viewer.launch_passive(model, data)

    n_total = len(order)
    max_t = sc.get("max_sim_s", 60.0 * n_total)
    decim = P.SIM["control_decimation"]
    dt_phys = model.opt.timestep
    dt_ctrl = dt_phys * decim
    step = 0
    print(f"scenario={Path(args.scenario).name} seed={seed} perception={mode} "
          f"executive={executive} items={n_total} -> {out_dir}")
    print(f"[sorter] {P.SORTER['n_carriers']} carriers, pitch "
          f"{P.SORTER['pitch']} m, v {P.SORTER['v_mps']} m/s, tilt "
          f"{P.SORTER['tilt_deg']} deg, latency "
          f"{P.SORTER['latency_s'] * 1000:.0f} ms")

    def window_clear():
        """Pre-gate hold condition: nobody OVERLAPS the hold->gate corridor —
        the vision window measures one item at a time. Long items count until
        their REAR clears the gate commit line (front-only checks release the
        next item while a 435 mm cylinder's tail is still being scanned)."""
        for slug in items.active:
            if slug in belts.skip:
                continue
            x, y = items.pose(slug)[:2]
            half = items.entries[slug]["dims_m"][0] / 2
            front, rear = x + half, x - half
            if (front > a["hold2_x"] + 0.05
                    and rear <= a["gate_x"] + 0.05
                    and abs(y - a["y"]) < 0.4):
                return False
        return True

    while len(items.done) < n_total and data.time < max_t:
        if step % decim == 0:
            t = data.time
            items.step(t, dt_ctrl)
            # ---- jam dispatch: C/D chute snags are arm jobs; anything else
            # (train top, B connector, induction) is an operator call-out
            for slug in [s for s, st_ in items.active.items()
                         if st_.get("jam_pending")]:
                st_j = items.active[slug]
                st_j["jam_pending"] = False
                p = items.pose(slug)
                zone_j = st_j.get("zone", "")
                arm_ok = (zone_j in ("C", "D")
                          and p[1] < P.ARM_GRASP_Y_MAX
                          and p[2] < 0.60
                          and abs(p[0] - P.STATIONS[zone_j]["x"]) < 0.5)
                if (arm_ok and not ctrl.busy and recovery["active"] is None
                        and st_j.get("recoveries", 0) < 2):
                    st_j["recovering"] = True
                    recovery["active"] = slug
                    # station out of service while the arm works its chute
                    sorter.station_hold.add(zone_j)
                    ctrl.start_job(slug, zone_j, items.entries[slug], t)
                else:
                    items._deliver(slug, "MANUAL", t)
            # ---- flow discipline (pre-gate hold blade)
            was_hold2 = belts.hold2_open
            set_hold2(not window_clear())
            if was_hold2 and not belts.hold2_open:
                flow_stats["spacing_gate_activations"] += 1
            flow_stats["multi_object_window_events"] = items.window_conflicts
            # ---- fault injection: a snag on a discharge chute (fault drills)
            if inject and inject["slug"] in items.active \
                    and inject["slug"] not in items.frozen:
                st_i = items.active[inject["slug"]]
                pp = items.pose(inject["slug"])
                on_chute = (pp[2] < 0.50 and st_i.get("zone") in ("C", "D")
                            and pp[1] < inject.get("at_y", 2.6))
                if on_chute and not st_i.get("recovering"):
                    items.frozen.add(inject["slug"])
                    items.frozen_pose[inject["slug"]] = \
                        items.d.qpos[items._adr(inject["slug"])[0]:
                                     items._adr(inject["slug"])[0] + 7].copy()
                    ev(t, "snag_injected", inject["slug"],
                       at_y=round(float(pp[1]), 3))
            # ---- sorter executive (tilt triggering, ramps, escapement blade)
            sorter.step(t, dt_ctrl, item_pos_of=items.item_pos_of)
            if os.environ.get("TWIN_TRACE"):
                for c_ in sorter.cars:
                    if abs(c_["goal"]) > 1.0 and c_["slug"]                             and int(t * 10) != int((t - dt_ctrl) * 10):
                        ip_ = items.item_pos_of(c_["slug"])
                        if ip_ is not None:
                            gid_ = model.geom(f"g_{c_['slug']}").id
                            parts = []
                            for ci_ in range(data.ncon):
                                cc_ = data.contact[ci_]
                                other = None
                                if cc_.geom1 == gid_:
                                    other = cc_.geom2
                                elif cc_.geom2 == gid_:
                                    other = cc_.geom1
                                if other is not None:
                                    parts.append(mujoco.mj_id2name(
                                        model, mujoco.mjtObj.mjOBJ_GEOM,
                                        other) or str(other))
                            print(f"[disch] {c_['slug']} t={t:7.2f} "
                                  f"pos=({ip_[0]:.3f},{ip_[1]:.3f},{ip_[2]:.3f}) "
                                  f"car_x={c_['x']:.2f} roll="
                                  f"{sorter.tray_roll(c_):+.1f} "
                                  f"goal={c_['goal']:+.1f} "
                                  f"touch={sorted(set(parts))}", flush=True)
            belts.gate_open = not sorter.egate_up
            # snag pin: hold the injected jam's POSE until the arm grasps it
            carried = (ctrl.job["slug"] if (ctrl.job and ctrl.job.get("weld_on"))
                       else None)
            items.pin_frozen(carried=carried)
            ctrl.step(dt_ctrl, t)
            # presentation state: lane lights, beacons, tower
            active_zones = {c["route"] for c in sorter.cars if c["route"]}
            jam = bool(recovery["active"]
                       or any(st_.get("jam_pending") for st_ in items.active.values()))
            vis.update(t, active_zones, jam)
        belts.step(data, dt_phys)
        sorter.physics_tick(dt_phys)
        t_before = data.time
        mujoco.mj_step(model, data)
        if data.time < t_before:
            # MuJoCo auto-reset after a solver divergence (mjWARN_BADQACC):
            # the world state is no longer meaningful — fail loudly instead of
            # silently continuing on a reset scene
            bus.publish("cell_event", t=t_before, event="physics_divergence")
            print(f"FATAL: physics divergence at t={t_before:.2f} "
                  f"(solver reset detected) — aborting run")
            break
        step += 1
        if recorder is not None:
            recorder.maybe_capture(data, data.time)
        if viewer_ctx is not None and step % 8 == 0:
            viewer_ctx.sync()
            if not viewer_ctx.is_running():
                break

    items.finalize_containment(data.time)
    if recorder is not None:
        recorder.close()
        print(f"video: {recorder.path}")
    out_dir.mkdir(parents=True, exist_ok=True)
    sorter.write_log(out_dir / "actuator_log.csv")
    unfinished = n_total - len(items.done)
    summary = metrics.finalize(data.time, extra={
        "engine": f"MuJoCo {mujoco.__version__}",
        "seed": seed, "perception": mode,
        "executive": "tilt_tray_sorter",
        "executive_mode": executive,
        "scenario": Path(args.scenario).name,
        "perception_mode": mode,
        "sensor_model": sensor_model,
        "oracle_used_for_classification": perceiver is None,
        "depth_noise_mm": sensor_cfg["depth_noise_mm"] if perceiver is not None else None,
        "run_id": run_id,
        "nominal_motion_model": "conveyor_surface_drive+tray_contact_gravity",
        "direct_velocity_writes_nominal": items.vwrites["nominal_set_v"],
        "direct_velocity_writes_fault_injection": items.vwrites["fault_injection"],
        "conveyor_drive_writes": belts.drive_writes,
        "sorter": sorter.summary(),
        "deadlocks": unfinished,
        **flow_stats,
    })
    print(json.dumps(summary, indent=2))
    # outcome classes (unified recovery policy): REVIEW / MANUAL are SAFE
    # designed diversions (they already cost routing_accuracy, never the run);
    # FLOOR is always a failure.
    review = [s for s, z in items.done.items() if z in ("REVIEW", "MANUAL")]
    floor = [s for s, z in items.done.items() if z == "FLOOR"]
    misrouted = [s for s, z in items.done.items()
                 if z != items.entries[s]["zone"]
                 and z not in ("REVIEW", "MANUAL", "FLOOR")]
    escaped = [s for s, w in items.cage_watch.items() if w["violated"]]
    # UNSAFE outcomes (a C/D item delivered to the B sorter) always fail; a
    # fault scenario may accept CONSERVATIVE wrong-cage outcomes
    unsafe = [s for s in misrouted if items.done[s] == "B"]
    if sc.get("allow_conservative"):
        misrouted = unsafe
    if review:
        print(f"SAFE REVIEW/MANUAL DIVERSIONS: {review}")
    if misrouted:
        print(f"MISROUTED: {misrouted}")
    if floor:
        print(f"FLOOR DROPS: {floor}")
    if unfinished:
        print(f"UNFINISHED: {unfinished} items (timeout at t={data.time:.0f}s)")
    if escaped:
        print(f"CONTAINMENT VIOLATED: {escaped}")
    return 1 if (misrouted or unfinished or escaped or floor) else 0


if __name__ == "__main__":
    raise SystemExit(main())
