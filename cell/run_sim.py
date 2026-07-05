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


class ItemManager:
    """Spawning, classification (oracle or camera), routing state, delivery."""

    CAM_WINDOW = (5.85, 6.28)    # classify while the item center crosses this
                                 # (ends before the escapement gate)

    def __init__(self, model, data, manifest, order, gaps, bus, perception_latency,
                 perceiver=None, spawn_rng=None, executive="arm", table=None):
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
        self.cages = P.cages_for(executive)
        self.active = {}                           # slug -> state dict
        self.done = {}                             # slug -> zone_actual

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

    def step(self, t):
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
            self.bus.publish("item_spawned", t=t, slug=slug)
            self.next_spawn_t = t + (self.gaps.pop(0) if self.gaps else 6.0)

        for slug, st in list(self.active.items()):
            e = self.entries[slug]
            qadr, dadr = self._adr(slug)
            pos = self.d.qpos[qadr:qadr + 3]
            vel = self.d.qvel[dadr:dadr + 3]
            # look-ahead classification at the vision station
            if not st["classified"]:
                if self.perceiver is None:
                    # oracle mode: ground truth stamped after a model latency
                    if t >= st["classify_t"]:
                        st["classified"] = True
                        st["zone"] = e["zone"]
                        self.bus.publish("item_classified", t=t, slug=slug,
                                         zone=e["zone"], zone_true=e["zone"],
                                         zone_raw=e["zone"], confidence=1.0, flags="")
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
                        st["next_capture_t"] = t + 0.12
                        res = self.perceiver.classify(self.d, x_hint=float(pos[0]))
                        reads = st.setdefault("cls_reads", [])
                        if res is not None:
                            reads.append(res)
                        leaving = pos[0] > self.CAM_WINDOW[1] - 0.06
                        # commit once the pose has stabilised (two consecutive
                        # agreeing reads after >=5) — rocking items keep being
                        # read through the gate dwell, up to a hard cap
                        stable = False
                        if len(reads) >= 2:
                            r1, r2 = reads[-2], reads[-1]
                            stable = (r1["zone"] == r2["zone"]
                                      and r1.get("dims_mm") and r2.get("dims_mm")
                                      and max(abs(a - b) for a, b in
                                              zip(r1["dims_mm"], r2["dims_mm"])) < 8.0)
                        if ((len(reads) >= 5 and stable) or len(reads) >= 30
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

    # ----------------------------------------------------------- table mode flow
    def _step_table_item(self, slug, st, pos, t):
        # assign the route as soon as the classified item commits to the table
        if st["classified"] and "routed_t" not in st and pos[0] > P.TABLE["x0"] - 0.25:
            self.table.routes[slug] = st["zone"]
            st["routed_t"] = t
            self.bus.publish("routing_cmd", t=t, slug=slug, zone=st["zone"])
        # delivered into a cage (rode the chute down)?
        zone = self._which_cage(pos)
        if zone is not None and pos[2] < 0.45:
            self._deliver(slug, zone, t)
            return
        # routing watchdog: jam -> escalate to the arm exception station
        if ("routed_t" in st and not st.get("jam_reported") and not st["picked"]
                and t - st["routed_t"] > P.JAM_TIMEOUT_S):
            if st.get("recoveries", 0) >= 2:
                self._deliver(slug, "MANUAL", t)   # give up: operator call-out
                return
            st["jam_reported"] = True
            self.bus.publish("cell_event", t=t, event="jam_detected", slug=slug,
                             zone=st["zone"], pos=[round(float(v), 3) for v in pos])

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
        isolated-circle policy fires only when persistent across reads."""
        st = self.active[slug]
        flags = []
        if not reads:
            zone_raw = zone = "D"              # sensor miss: manual stream
            rep = {"confidence": 0.0, "max_ratio": None, "dims_mm": None}
            flags.append("sensor_miss")
        else:
            rep = max(reads, key=lambda r: (r.get("max_ratio") is not None,
                                            r.get("max_ratio") or 0))
            dims = np.median(np.array([r["dims_mm"] for r in reads if r.get("dims_mm")]),
                             axis=0)
            minors = [r["minor_mm"] for r in reads if r.get("minor_mm") is not None]
            minor = float(np.median(minors)) if minors else None
            undersize = bool(np.any(dims < 10.0) or (minor is not None and minor < 10.0))
            oversize = bool(np.any(np.sort(dims)[::-1] > np.array([450.0, 320.0, 320.0])))
            circ_strong = any(r.get("circular") for r in reads)
            n_relaxed = sum(1 for r in reads if r.get("circular_relaxed"))
            n_isolated = sum(1 for r in reads if "isolated_end_circle" in r.get("flags", []))
            weak_persistent = (n_isolated * 2 >= len(reads)) or (n_relaxed >= 2)
            if undersize or oversize:
                zone_raw = zone = "C"
            elif circ_strong:
                zone_raw = zone = "D"
            elif weak_persistent:
                zone_raw, zone = "B", "D"      # policy: uncertain shape -> repack
                flags.append("policy_reroute_D")
            else:
                zone_raw = zone = "B"
            rep = dict(rep, dims_mm=[round(float(d), 1) for d in dims])
            flags = list(rep.get("flags", [])) + flags
        st["classified"] = True
        st["zone"] = zone
        self.bus.publish("item_classified", t=t, slug=slug,
                         zone=zone, zone_true=e["zone"], zone_raw=zone_raw,
                         confidence=rep.get("confidence"), ratio=rep.get("max_ratio"),
                         dims_mm=rep.get("dims_mm"), n_reads=len(reads),
                         flags=";".join(flags))

    def _which_cage(self, pos):
        for zone, cage in self.cages.items():
            cx, cy = cage["center"]
            if abs(pos[0] - cx) < cage["inner"][0] / 2 + 0.05 and \
               abs(pos[1] - cy) < cage["inner"][1] / 2 + 0.05 and pos[2] < 0.9:
                return zone
        return None

    def _deliver(self, slug, zone_actual, t):
        st = self.active.pop(slug)
        e = self.entries[slug]
        ok = zone_actual == e["zone"]
        self.done[slug] = zone_actual
        # park delivered B-items back off-cell so contacts stay cheap
        if zone_actual == "B":
            qadr, dadr = self._adr(slug)
            idx = list(self.entries).index(slug)
            self.d.qpos[qadr:qadr + 3] = [0.6 + idx * 0.85, -2.5, e["dims_m"][2] / 2 + 0.001]
            self.d.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
            self.d.qvel[dadr:dadr + 6] = 0
        self.bus.publish("item_delivered", t=t, slug=slug, zone=zone_actual, ok=ok)

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
        slugs += [name] * int(count)
    order = list(rng.permutation(slugs))
    gaps = list(rng.uniform(sc["spawn_gap_s"][0], sc["spawn_gap_s"][1], size=len(order)))

    perceiver = None
    if mode == "camera":
        from perception.pipeline import LookaheadPerception
        perceiver = LookaheadPerception(model)

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out) if args.out else ROOT / "runs" / f"{stamp}_seed{seed}_{mode}_{executive}"
    bus = Bus()
    metrics = Metrics(bus, out_dir)
    table = Table(model, manifest) if executive == "table" else None
    items = ItemManager(model, data, manifest, order, gaps, bus,
                        perception_latency=sc.get("perception_latency_s", 0.15),
                        perceiver=perceiver,
                        spawn_rng=np.random.default_rng(seed + 1000),
                        executive=executive, table=table)
    belts = Belts(model, manifest, mode=executive)
    ctrl = Controller(model, data, bus, mode=executive)
    inject = sc.get("inject_jam")      # e.g. {slug: helmet, at_x: 8.0}

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
                ok = m.get("event") == "job_done"
                bus.publish("cell_event", t=m["t"], event="recovery_done",
                            slug=m["slug"], ok=ok)
                st = items.active.get(m["slug"])
                if st is not None:
                    # re-arm the watchdog: if the item STILL fails to arrive,
                    # escalate again — at most twice, then flag for manual
                    st["recoveries"] = st.get("recoveries", 0) + 1
                    st["routed_t"] = m["t"]
                    st["jam_reported"] = False
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
        """Pre-gate hold condition: nobody between the hold line and the gate
        commit line — the vision window measures one item at a time."""
        for slug in items.active:
            if slug in belts.skip:
                continue
            x, y = items.pose(slug)[:2]
            front = x + items.entries[slug]["dims_m"][0] / 2
            if (P.BELT_A["hold2_x"] + 0.05 < front <= P.BELT_A["gate_x"] + 0.05
                    and abs(y - P.BELT_A["y"]) < 0.4):
                return False
        return True

    while len(items.done) < n_total and data.time < max_t:
        if step % decim == 0:
            t = data.time
            items.step(t)
            belts.gate_open = downstream_clear()
            belts.hold2_open = window_clear()
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
                        table.routes.pop(slug, None)
                        # a confidently dimension-gated item still belongs in C;
                        # everything else uncertain goes to D / manual review
                        rzone = "C" if st_r.get("zone") == "C" else "D"
                        bus.publish("cell_event", t=t, event="recovery_start",
                                    slug=slug, target=rzone)
                        ctrl.start_job(slug, rzone, items.entries[slug], t)
            else:
                if not ctrl.busy:
                    slug, st = items.ready_for_pick()
                    if slug is not None:
                        st["picked"] = True
                        ctrl.start_job(slug, st["zone"], items.entries[slug], t)
            ctrl.step(dt_ctrl, t)
        belts.step(data)
        if table is not None:
            table.step(data)
        mujoco.mj_step(model, data)
        step += 1
        if viewer_ctx is not None and step % 8 == 0:
            viewer_ctx.sync()
            if not viewer_ctx.is_running():
                break

    summary = metrics.finalize(data.time, extra={"seed": seed, "perception": mode,
                                                 "executive": executive,
                                                 "scenario": Path(args.scenario).name})
    print(json.dumps(summary, indent=2))
    misrouted = [s for s, z in items.done.items() if z != items.entries[s]["zone"]]
    unfinished = n_total - len(items.done)
    if misrouted:
        print(f"MISROUTED: {misrouted}")
    if unfinished:
        print(f"UNFINISHED: {unfinished} items (timeout at t={data.time:.0f}s)")
    return 1 if (misrouted or unfinished) else 0


if __name__ == "__main__":
    raise SystemExit(main())
