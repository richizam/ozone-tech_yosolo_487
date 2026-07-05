# -*- coding: utf-8 -*-
"""Cell simulation entrypoint — the closed loop, end to end.

    python -m cell.run_sim --scenario scenarios/base.yaml --seed 42
    python -m cell.run_sim --viewer          (watch it live)

Spawns the official items onto conveyor A, classifies each (v0: oracle =
ground truth, stamped with a configurable perception latency — the look-ahead
station), lets the belt carry it to the accumulator, and the arm routes it to
B / C / D. Writes runs/<stamp>/events.csv + summary.json and exits non-zero
if any item was misrouted."""
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
from cell.table import Table  # noqa: E402
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
    """Spawning, classification (oracle or camera), routing state, delivery."""

    CAM_WINDOW = P.VIRTUAL_SENSOR["window_x"]    # classify while the item center
                                                 # crosses this (ends before the
                                                 # escapement gate)

    def __init__(self, model, data, manifest, order, gaps, bus, perception_latency,
                 perceiver=None, spawn_rng=None, executive="arm", table=None,
                 belts=None, cls_cfg=None):
        self.m, self.d, self.bus = model, data, bus
        self.entries = {e["slug"]: e for e in manifest}
        self.queue = list(order)                   # slugs to spawn
        self.gaps = list(gaps)
        self.next_spawn_t = 1.0
        self.latency = perception_latency
        self.perceiver = perceiver                 # None -> oracle mode
        self.spawn_rng = spawn_rng
        self.executive = executive
        self.table = table                         # Table instance in table mode
        self.belts = belts                         # for gate-hold accounting
        self.cls_cfg = dict(P.CLASSIFICATION, **(cls_cfg or {}))
        self.proc_latency = P.VIRTUAL_SENSOR["processing_latency_s"]
        self.cages = P.cages_for(executive)
        self.active = {}                           # slug -> state dict
        self.done = {}                             # slug -> zone_actual
        self.cage_watch = {}                       # slug -> post-delivery containment state
        self.window_conflicts = 0                  # events: >=2 items co-occupied
        self._window_multi = False                 # the measurement window

    def _adr(self, slug):
        jid = self.m.joint(f"fj_{slug}").id
        return self.m.jnt_qposadr[jid], self.m.jnt_dofadr[jid]

    def pose(self, slug):
        qadr, _ = self._adr(slug)
        return self.d.qpos[qadr:qadr + 3]

    def spawn_zone_clear(self):
        for slug in self.active:
            x, y, _ = self.pose(slug)
            if x < 1.2 and abs(y - P.BELT_A["y"]) < 0.4:
                return False
        return True

    def step(self, t, dt=0.0):
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
            self.d.qpos[qadr:qadr + 3] = [0.4, P.BELT_A["y"], P.BELT_A["top"] + e["dims_m"][2] / 2 + 0.003]
            self.d.qpos[qadr + 3:qadr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
            self.d.qvel[dadr:dadr + 6] = 0
            self.active[slug] = {"classified": False, "settled": False, "picked": False,
                                 "released_t": None, "classify_t": t + self.latency,
                                 "attempts": 0}
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

        cfg = self.cls_cfg
        for slug, st in list(self.active.items()):
            e = self.entries[slug]
            qadr, dadr = self._adr(slug)
            pos = self.d.qpos[qadr:qadr + 3]
            vel = self.d.qvel[dadr:dadr + 3]
            # detection-zone entry: cycle_start for BOTH perception modes
            if "t_detected" not in st and pos[0] >= self.CAM_WINDOW[0]:
                st["t_detected"] = t
                self.bus.publish("item_detected", t=t, slug=slug)
            # escapement/pre-gate holds: accumulated per item (spacing control;
            # the BELT never stops — see conveyor_a_stop_count)
            if self.belts is not None and dt > 0.0 and not st["picked"]:
                front = pos[0] + e["dims_m"][0] / 2
                a = P.BELT_A
                if (not self.belts.gate_open
                        and a["gate_x"] - 0.004 <= front < a["gate_x"] + 0.05):
                    st["gate_hold_s"] = st.get("gate_hold_s", 0.0) + dt
                if (not self.belts.hold2_open
                        and a["hold2_x"] - 0.004 <= front < a["hold2_x"] + 0.05):
                    st["hold2_hold_s"] = st.get("hold2_hold_s", 0.0) + dt
            # pending classification verdict: published after the modeled
            # processing latency (route command generation time)
            if not st["classified"] and "pending_cls" in st:
                if t >= st["pending_cls"]["t_ready"]:
                    self._publish_classification(slug, t)
            # look-ahead classification at the vision station
            elif not st["classified"]:
                if self.perceiver is None:
                    # oracle mode: ground truth stamped after a model latency
                    if t >= st["classify_t"]:
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
                elif pos[0] > self.CAM_WINDOW[1]:
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
                        # read through the gate dwell, up to a hard cap
                        stable = False
                        if len(reads) >= 2:
                            r1, r2 = reads[-2], reads[-1]
                            stable = (r1["zone"] == r2["zone"]
                                      and r1.get("dims_mm") and r2.get("dims_mm")
                                      and max(abs(a - b) for a, b in
                                              zip(r1["dims_mm"], r2["dims_mm"]))
                                      < cfg["stable_dims_tol_mm"])
                        if ((len(reads) >= cfg["stable_reads_required"] and stable)
                                or len(reads) >= cfg["read_cap"]
                                or (leaving and reads)):
                            self._commit_classification(slug, e, reads, t)
            # fell to the floor while nobody was carrying it? adjudicate as a
            # logged fault instead of stranding the run (cage interiors are
            # legitimate low-z places — excluded via the cage check)
            if (not st["picked"] and pos[2] < 0.25 and pos[1] > 0
                    and self._which_cage(pos) is None
                    and not (abs(pos[0] - 0.4) < 0.3 and pos[1] < 0)):
                self._deliver(slug, "FLOOR", t)
                continue
            # delivered to B (belt B carried it past the exit line)?
            if pos[1] > P.BELT_B["y_delivered"] and abs(pos[0] - P.BELT_B["cx"]) < 0.3:
                self._deliver(slug, "B", t)
                continue
            if self.executive == "table":
                self._step_table_item(slug, st, pos, t)
            else:
                self._step_arm_item(slug, st, e, pos, vel, t)
        self._watch_containment(t)

    # ----------------------------------------------------------- table mode flow
    def _step_table_item(self, slug, st, pos, t):
        # assign the route as soon as the classified item commits to the table
        if st["classified"] and "routed_t" not in st and pos[0] > P.TABLE["x0"] - 0.25:
            self.table.routes[slug] = st["zone"]
            st["routed_t"] = t
            st["watch_xy"] = (float(pos[0]), float(pos[1]))
            st["watch_t"] = t
            self.bus.publish("routing_cmd", t=t, slug=slug, zone=st["zone"])
            self.bus.publish("table_state", t=t, slug=slug,
                             state=f"TABLE_ROUTE_{st['zone']}")
        # physical entry onto the table: command_margin_s reference point
        if "t_table_entry" not in st and pos[0] > P.TABLE["x0"] and pos[2] > 0.5:
            st["t_table_entry"] = t
            self.bus.publish("table_entry", t=t, slug=slug)
        # delivered into a cage? (rode the chute through the aperture — items
        # queued on the in-cage slope section count: they are inside the
        # cage volume and the containment watch takes over from here)
        zone = self._which_cage(pos)
        if zone is not None and pos[2] < 0.56:
            self._deliver(slug, zone, t)
            return
        # routing watchdog: a jam is NO DISPLACEMENT over the timeout window
        # (an item creeping through a queue is flow, not a fault)
        if "routed_t" in st and not st.get("jam_reported") and not st["picked"]:
            if "watch_xy" not in st:
                st["watch_xy"] = (float(pos[0]), float(pos[1]))
                st["watch_t"] = t
            elif t - st["watch_t"] >= P.JAM_TIMEOUT_S:
                moved = float(np.hypot(pos[0] - st["watch_xy"][0],
                                       pos[1] - st["watch_xy"][1]))
                st["watch_xy"] = (float(pos[0]), float(pos[1]))
                st["watch_t"] = t
                if moved < P.JAM_MIN_PROGRESS_M:
                    if st.get("recoveries", 0) >= 2:
                        self._deliver(slug, "MANUAL", t)   # give up: operator call-out
                        return
                    st["jam_reported"] = True
                    self.bus.publish("cell_event", t=t, event="jam_detected",
                                     slug=slug, zone=st["zone"],
                                     pos=[round(float(v), 3) for v in pos])

    # ----------------------------------------------------------- arm mode flow
    def _step_arm_item(self, slug, st, e, pos, vel, t):
        # settled at accumulator?
        if st["classified"] and not st["settled"]:
            at_stop = pos[0] + e["dims_m"][0] / 2 > P.BELT_A["x_stop"] - 0.05
            if at_stop and float(np.linalg.norm(vel)) < P.SIM["settle_speed"]:
                st.setdefault("low_v_since", t)
                if t - st["low_v_since"] >= P.SIM["settle_time"]:
                    st["settled"] = True
                    self.bus.publish("item_settled", t=t, slug=slug)
            else:
                st.pop("low_v_since", None)
        if st["released_t"] is not None:
            since_release = t - st["released_t"]
            on_belt_a = (pos[0] < P.BELT_A["x_stop"] + 0.1
                         and abs(pos[1] - P.BELT_A["y"]) < 0.3
                         and P.BELT_A["top"] - 0.02 < pos[2] < P.BELT_A["top"] + 0.5)
            if on_belt_a and since_release > 1.5 and st["attempts"] < 3:
                # dropped back onto the feed belt: run it through again
                st["attempts"] += 1
                st["picked"] = False
                st["settled"] = False
                st["released_t"] = None
                st.pop("low_v_since", None)
            elif st["zone"] in ("C", "D") and since_release > 1.0:
                # dropped into a cage: settled inside within a second?
                zone = self._which_cage(pos)
                self._deliver(slug, zone if zone else "FLOOR", t)
            elif st["zone"] == "B" and since_release > 6.0:
                # fault fallback: item released to B never crossed the exit line
                self._deliver(slug, self._which_cage(pos) or "FLOOR", t)

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
            if not circ_strong and max_ratio is not None and noise_mm > 0.0:
                # ratio uncertainty scales with noise over the cross-section
                # circumradius (the two smaller extents span the section)
                srt = np.sort(dims)[::-1]
                r_est = float(np.hypot(srt[1], srt[2])) / 2.0
                g_ratio = min(0.25, 2.0 * noise_mm / max(r_est, 5.0))
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

    @staticmethod
    def _in_cage_region(pos, cage, closed_m, open_m):
        """Inside the cage envelope: `closed_m` beyond the inner walls on the
        closed sides, `open_m` along the aperture side — the hooded chute
        mouth is part of the cage's containment envelope (items piling there
        rest on the guided path, bounded by hood, rails and skirt)."""
        cx, cy = cage["center"]
        hx, hy = cage["inner"][0] / 2, cage["inner"][1] / 2
        m = {"-x": closed_m, "+x": closed_m, "-y": closed_m, "+y": closed_m}
        side = cage.get("open_side")
        if side in m:
            m[side] = open_m
        return (cx - hx - m["-x"] <= pos[0] <= cx + hx + m["+x"]
                and cy - hy - m["-y"] <= pos[1] <= cy + hy + m["+y"])

    def _which_cage(self, pos):
        for zone, cage in self.cages.items():
            if self._in_cage_region(pos, cage, 0.05, 0.30) and pos[2] < 0.9:
                return zone
        return None

    def _deliver(self, slug, zone_actual, t):
        st = self.active.pop(slug)
        e = self.entries[slug]
        qadr, dadr = self._adr(slug)
        v_entry = float(np.linalg.norm(self.d.qvel[dadr:dadr + 3]))
        if self.table is not None:
            self.table.routes.pop(slug, None)
        ok = zone_actual == e["zone"]
        self.done[slug] = zone_actual
        # delivery into a cage is not the end of the story: the item is
        # tracked until the run ends — it must SETTLE and STAY inside
        if zone_actual in ("C", "D"):
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
                         v_entry=round(v_entry, 3),
                         route_command=st.get("zone"),
                         gate_hold_s=round(st.get("gate_hold_s", 0.0), 2),
                         hold2_hold_s=round(st.get("hold2_hold_s", 0.0), 2))

    # ------------------------------------------------- containment validation
    def _watch_containment(self, t):
        """Routed != done: a delivered C/D item is watched until the run ends.
        Leaving the cage footprint (or flying above it) is a violation."""
        for slug, w in self.cage_watch.items():
            cage = self.cages[w["zone"]]
            qadr, dadr = self._adr(slug)
            pos = self.d.qpos[qadr:qadr + 3]
            speed = float(np.linalg.norm(self.d.qvel[dadr:dadr + 3]))
            w["max_z"] = max(w["max_z"], float(pos[2]))
            outside = (not self._in_cage_region(
                pos, cage, cage["wall_t"] + P.CONTAIN["margin"], 0.32)
                or pos[2] > P.CONTAIN["z_fly"])
            if outside and not w["violated"]:
                w["violated"] = True
                self.bus.publish("cell_event", t=t, event="containment_violation",
                                 slug=slug, zone=w["zone"],
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

    def ready_for_pick(self):
        for slug, st in self.active.items():
            if st["settled"] and not st["picked"]:
                return slug, st
        return None, None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", default=str(ROOT / "scenarios" / "base.yaml"))
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--viewer", action="store_true", help="open the MuJoCo viewer")
    ap.add_argument("--out", default=None)
    ap.add_argument("--perception", choices=["oracle", "camera"], default=None,
                    help="override the scenario's perception mode")
    ap.add_argument("--executive", choices=["table", "arm"], default=None,
                    help="override the scenario's executive architecture")
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
    executive = args.executive or sc.get("executive", P.EXEC_DEFAULT)

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
    table = Table(model, manifest) if executive == "table" else None
    belts = Belts(model, manifest, mode=executive)
    items = ItemManager(model, data, manifest, order, gaps, bus,
                        perception_latency=sc.get("perception_latency_s", 0.15),
                        perceiver=perceiver,
                        spawn_rng=np.random.default_rng(seed + 1000),
                        executive=executive, table=table, belts=belts,
                        cls_cfg=sc.get("classification"))
    ctrl = Controller(model, data, bus, mode=executive)
    inject = sc.get("inject_jam")      # e.g. {slug: helmet, at_x: 8.0}
    # single-item-discipline + belt-state counters (реакция на нештатные потоки)
    flow_stats = {"spacing_gate_activations": 0, "escapement_gate_activations": 0,
                  "multi_object_window_events": 0, "conveyor_a_stop_count": 0}

    # presentation layer: category tint, lane lights, beacons, andon tower
    vis = Visuals(model)
    ea_id = model.actuator("ea").id    # escapement-gate flag actuator
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

    recovery = {"pending": [], "active": None}

    def on_attached(**m):
        if m.get("event") == "attached":
            belts.skip.add(m["slug"])
            if table is not None:
                table.skip.add(m["slug"])
                table.unfreeze(m["slug"])
    bus.subscribe("cell_event", on_attached)

    def on_released(**m):
        if m.get("event") == "released":
            belts.skip.discard(m["slug"])
            if table is not None:
                table.skip.discard(m["slug"])
            if m["slug"] in items.active:
                items.active[m["slug"]]["released_t"] = m["t"]

    def on_job_end(**m):
        if m.get("event") in ("job_done", "job_abort") and m["slug"] == recovery["active"]:
            recovery["active"] = None
            if table is not None:
                table.paused = False
                table.hold_open.clear()
                ok = m.get("event") == "job_done"
                bus.publish("cell_event", t=m["t"], event="recovery_done",
                            slug=m["slug"], ok=ok)
                bus.publish("table_state", t=m["t"], slug=m["slug"],
                            state="TABLE_IDLE")
                st = items.active.get(m["slug"])
                if st is not None:
                    # re-arm the watchdog: if the item STILL fails to arrive,
                    # escalate again — at most twice, then flag for manual
                    st["recoveries"] = st.get("recoveries", 0) + 1
                    st["routed_t"] = m["t"]
                    st["jam_reported"] = False
                    st["picked"] = False       # watchdog must be able to re-arm
                    st.pop("watch_xy", None)   # fresh displacement window
    bus.subscribe("cell_event", on_released)
    bus.subscribe("cell_event", on_job_end)

    def on_jam(**m):
        if m.get("event") == "jam_detected":
            recovery["pending"].append(m["slug"])
    bus.subscribe("cell_event", on_jam)

    def on_abort(**m):
        if m.get("event") != "job_abort":
            return
        slug = m["slug"]
        belts.skip.discard(slug)
        if table is not None:
            table.skip.discard(slug)
        st = items.active.get(slug)
        if st is None:
            return
        if m.get("carrying"):
            # item dropped mid-carry: let the delivery detector judge the landing
            st["released_t"] = m["t"]
            return
        st["attempts"] += 1
        if st["attempts"] >= 2:
            items._deliver(slug, "GRASP_FAIL", m["t"])   # give up: flagged for manual handling
        elif executive == "table":
            st["picked"] = False
            st["jam_reported"] = False                    # let the watchdog re-escalate
        else:
            st["picked"] = False                          # retry once with a fresh pose
            st["settled"] = False
            st.pop("low_v_since", None)
    bus.subscribe("cell_event", on_abort)

    viewer_ctx = None
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer_ctx = mj_viewer.launch_passive(model, data)

    n_total = len(order)
    max_t = sc.get("max_sim_s", 60.0 * n_total)
    decim = P.SIM["control_decimation"]
    dt_ctrl = model.opt.timestep * decim
    step = 0
    print(f"scenario={Path(args.scenario).name} seed={seed} perception={mode} "
          f"executive={executive} items={n_total} -> {out_dir}")

    def downstream_clear():
        """Escapement-gate condition: nobody (not held by the arm) committed
        past the gate line and still being processed downstream."""
        for slug in items.active:
            if slug in belts.skip:
                continue
            x, y = items.pose(slug)[:2]
            front = x + items.entries[slug]["dims_m"][0] / 2
            if front > P.BELT_A["gate_x"] + 0.05 and abs(y - P.BELT_A["y"]) < 0.65:
                return False
        return True

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
            if (front > P.BELT_A["hold2_x"] + 0.05
                    and rear <= P.BELT_A["gate_x"] + 0.05
                    and abs(y - P.BELT_A["y"]) < 0.4):
                return False
        return True

    while len(items.done) < n_total and data.time < max_t:
        if step % decim == 0:
            t = data.time
            items.step(t, dt_ctrl)
            was_gate, was_hold2 = belts.gate_open, belts.hold2_open
            belts.gate_open = downstream_clear()
            belts.hold2_open = window_clear()
            # flow-discipline telemetry: each open->closed transition is one
            # activation; a hold2 activation means a SECOND item approached the
            # measurement corridor while it was busy (single-item discipline)
            if was_gate and not belts.gate_open:
                flow_stats["escapement_gate_activations"] += 1
            if was_hold2 and not belts.hold2_open:
                flow_stats["spacing_gate_activations"] += 1
            flow_stats["multi_object_window_events"] = items.window_conflicts
            # fault injection: a snag on the table (fault scenarios)
            if inject and inject["slug"] in items.active and table is not None \
                    and not items.active[inject["slug"]].get("snagged"):
                if items.pose(inject["slug"])[0] > inject.get("at_x", 8.0):
                    items.active[inject["slug"]]["snagged"] = True
                    table.freeze(inject["slug"])
                    bus.publish("cell_event", t=t, event="snag_injected", slug=inject["slug"])
            if executive == "table":
                # dispatch the arm to jammed items (one recovery at a time)
                if recovery["pending"] and recovery["active"] is None and not ctrl.busy:
                    slug = recovery["pending"].pop(0)
                    if slug in items.active:
                        recovery["active"] = slug
                        table.paused = True
                        st_r = items.active[slug]
                        st_r["picked"] = True
                        # the route STAYS assigned: after the arm places the
                        # item back on its lane the table drive re-delivers it
                        # a confidently dimension-gated item still belongs in C;
                        # everything else uncertain goes to D / manual review
                        rzone = "C" if st_r.get("zone") == "C" else "D"
                        table.routes[slug] = rzone
                        table.hold_open.add(rzone)     # recovery path crosses this gate
                        bus.publish("cell_event", t=t, event="recovery_start",
                                    slug=slug, target=rzone)
                        bus.publish("table_state", t=t, slug=slug,
                                    state="TABLE_RECOVERY_LOCKOUT")
                        ctrl.start_job(slug, rzone, items.entries[slug], t)
            else:
                if not ctrl.busy:
                    slug, st = items.ready_for_pick()
                    if slug is not None:
                        st["picked"] = True
                        ctrl.start_job(slug, st["zone"], items.entries[slug], t)
            ctrl.step(dt_ctrl, t)
            # presentation state: escapement-gate flag, lane lights, tower
            data.ctrl[ea_id] = 0.5 if belts.gate_open else 0.0
            active_zones = set()
            if table is not None:
                active_zones = {r for s, r in table.routes.items() if s not in table.skip}
            else:
                active_zones = {st["zone"] for st in items.active.values()
                                if st.get("picked") and st.get("zone")}
            jam = bool(recovery["pending"] or recovery["active"])
            vis.update(t, active_zones, jam,
                       table.gate_open_frac(data) if table is not None else None)
            if os.environ.get("SIM_DEBUG_STALL") and step % (decim * 50) == 0:
                for slug in items.active:
                    st_d = items.active[slug]
                    if "routed_t" not in st_d or st_d.get("picked"):
                        continue
                    qadr_d, dadr_d = items._adr(slug)
                    pos_d = data.qpos[qadr_d:qadr_d + 3]
                    spd = float(np.linalg.norm(data.qvel[dadr_d:dadr_d + 3]))
                    if spd < 0.05 and t - st_d["routed_t"] > 3.0:
                        gid_d = model.geom(f"g_{slug}").id
                        parts = []
                        for ci in range(data.ncon):
                            c = data.contact[ci]
                            other = c.geom2 if c.geom1 == gid_d else (
                                c.geom1 if c.geom2 == gid_d else None)
                            if other is not None:
                                parts.append(mujoco.mj_id2name(
                                    model, mujoco.mjtObj.mjOBJ_GEOM, other))
                        print(f"[stall] t={t:.1f} {slug} zone={st_d.get('zone')} "
                              f"pos=({pos_d[0]:.3f},{pos_d[1]:.3f},{pos_d[2]:.3f}) "
                              f"v={spd:.3f} contacts={sorted(set(parts))}")
        belts.step(data)
        if table is not None:
            table.step(data)
        mujoco.mj_step(model, data)
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
    unfinished = n_total - len(items.done)
    summary = metrics.finalize(data.time, extra={
        "seed": seed, "perception": mode, "executive": executive,
        "scenario": Path(args.scenario).name,
        "perception_mode": mode, "executive_mode": executive,
        "sensor_model": sensor_model,
        "oracle_used_for_classification": perceiver is None,
        "depth_noise_mm": sensor_cfg["depth_noise_mm"] if perceiver is not None else None,
        "run_id": run_id,
        "deadlocks": unfinished,
        **flow_stats,
    })
    print(json.dumps(summary, indent=2))
    misrouted = [s for s, z in items.done.items() if z != items.entries[s]["zone"]]
    escaped = [s for s, w in items.cage_watch.items() if w["violated"]]
    # fault scenarios may accept CONSERVATIVE outcomes (a sortable item diverted
    # to inspection during recovery); UNSAFE outcomes (bad item delivered to the
    # sorter) always fail the run
    unsafe = [s for s in misrouted if items.done[s] == "B"]
    if sc.get("allow_conservative"):
        misrouted = unsafe
    if misrouted:
        print(f"MISROUTED: {misrouted}")
    if unfinished:
        print(f"UNFINISHED: {unfinished} items (timeout at t={data.time:.0f}s)")
    if escaped:
        print(f"CONTAINMENT VIOLATED: {escaped}")
    return 1 if (misrouted or unfinished or escaped) else 0


if __name__ == "__main__":
    raise SystemExit(main())
