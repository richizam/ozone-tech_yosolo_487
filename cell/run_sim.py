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


class ItemManager:
    """Spawning, settling detection, delivery detection."""

    def __init__(self, model, data, manifest, order, gaps, bus, perception_latency):
        self.m, self.d, self.bus = model, data, bus
        self.entries = {e["slug"]: e for e in manifest}
        self.queue = list(order)                   # slugs to spawn
        self.gaps = list(gaps)
        self.next_spawn_t = 1.0
        self.latency = perception_latency
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
            self.d.qpos[qadr:qadr + 3] = [0.4, P.BELT_A["y"], P.BELT_A["top"] + e["dims_m"][2] / 2 + 0.003]
            self.d.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
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
            # look-ahead classification (oracle v0)
            if not st["classified"] and t >= st["classify_t"]:
                st["classified"] = True
                st["zone"] = e["zone"]
                self.bus.publish("item_classified", t=t, slug=slug, zone=e["zone"])
            # fell off the belt while nobody was carrying it? adjudicate as a
            # logged fault instead of stranding the run
            if (not st["picked"] and st["released_t"] is None
                    and pos[2] < P.BELT_A["top"] - 0.15
                    and not (abs(pos[0] - 0.4) < 0.3 and pos[1] < 0)):
                self._deliver(slug, "FLOOR", t)
                continue
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
            # delivered to B (belt B carried it past the exit line)?
            if pos[1] > P.BELT_B["y_delivered"] and abs(pos[0] - P.BELT_B["cx"]) < 0.3:
                self._deliver(slug, "B", t)
            elif st["released_t"] is not None:
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

    def _which_cage(self, pos):
        for zone, cage in (("C", P.CAGE_C), ("D", P.CAGE_D)):
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
    args = ap.parse_args(argv)

    sc = yaml.safe_load(Path(args.scenario).read_text(encoding="utf-8"))
    seed = args.seed if args.seed is not None else sc.get("seed", 42)
    rng = np.random.default_rng(seed)

    model, manifest, _ = make_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    slugs = []
    for name, count in sc["items"].items():
        slugs += [name] * int(count)
    order = list(rng.permutation(slugs))
    gaps = list(rng.uniform(sc["spawn_gap_s"][0], sc["spawn_gap_s"][1], size=len(order)))

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out) if args.out else ROOT / "runs" / f"{stamp}_seed{seed}"
    bus = Bus()
    metrics = Metrics(bus, out_dir)
    items = ItemManager(model, data, manifest, order, gaps, bus,
                        perception_latency=sc.get("perception_latency_s", 0.15))
    belts = Belts(model, manifest)
    ctrl = Controller(model, data, bus)

    bus.subscribe("cell_event", lambda **m: belts.skip.add(m["slug"]) if m.get("event") == "attached" else None)

    def on_released(**m):
        if m.get("event") == "released":
            belts.skip.discard(m["slug"])
            items.active[m["slug"]]["released_t"] = m["t"]
    bus.subscribe("cell_event", on_released)

    def on_abort(**m):
        if m.get("event") != "job_abort":
            return
        slug = m["slug"]
        belts.skip.discard(slug)
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
    print(f"scenario={Path(args.scenario).name} seed={seed} items={n_total} -> {out_dir}")

    def accumulator_clear():
        """Escapement-gate condition: nobody (not held by the arm) committed
        past the gate line."""
        for slug in items.active:
            if slug in belts.skip:
                continue
            x, y = items.pose(slug)[:2]
            front = x + items.entries[slug]["dims_m"][0] / 2
            if front > P.BELT_A["gate_x"] + 0.05 and abs(y - P.BELT_A["y"]) < 0.4:
                return False
        return True

    while len(items.done) < n_total and data.time < max_t:
        if step % decim == 0:
            items.step(data.time)
            belts.gate_open = accumulator_clear()
            if not ctrl.busy:
                slug, st = items.ready_for_pick()
                if slug is not None:
                    st["picked"] = True
                    ctrl.start_job(slug, st["zone"], items.entries[slug], data.time)
            ctrl.step(dt_ctrl, data.time)
        belts.step(data)
        mujoco.mj_step(model, data)
        step += 1
        if viewer_ctx is not None and step % 8 == 0:
            viewer_ctx.sync()
            if not viewer_ctx.is_running():
                break

    summary = metrics.finalize(data.time, extra={"seed": seed, "scenario": Path(args.scenario).name})
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
