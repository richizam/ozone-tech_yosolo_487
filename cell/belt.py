# -*- coding: utf-8 -*-
"""Kinematic conveyor model.

An item is driven at belt speed ONLY while it is in physical contact with the
belt geom (v0 simplification of a friction-driven belt: the drive writes the
linear velocity; belt friction is modeled as strong damping of the item's
angular velocity, which is what a real belt does to a tumbling light object).
The drive disengages near the accumulator stop wall so plain contact does the
stopping, and re-engages items queued behind."""
import numpy as np

from cell import params as P

SPIN_DAMP = 0.85          # per-control-step angular velocity retention on belt


class Belts:
    def __init__(self, model, item_entries, mode="arm"):
        self.m = model
        self.mode = mode
        self.gid_beltA = model.geom("beltA").id
        self.gid_beltB = model.geom("beltB").id
        self.items = []
        self.by_gid = {}
        for e in item_entries:
            jid = model.joint(f"fj_{e['slug']}").id
            it = {
                "slug": e["slug"],
                "gid": model.geom(f"g_{e['slug']}").id,
                "qadr": model.jnt_qposadr[jid],
                "dadr": model.jnt_dofadr[jid],
                "half_x": e["dims_m"][0] / 2,
            }
            self.items.append(it)
            self.by_gid[it["gid"]] = it
        self.skip = set()              # slugs currently attached (arm holds them)
        self.gate_open = True          # escapement gate before the accumulator
        self.hold2_open = True         # pre-gate hold (keeps the vision window
                                       # single-item while somebody is at the gate)

    def _belt_contacts(self, data):
        """gid -> set of belt geom ids the item touches this step."""
        touching = {}
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = c.geom1, c.geom2
            for gi, gb in ((g1, g2), (g2, g1)):
                if gb in (self.gid_beltA, self.gid_beltB) and gi in self.by_gid:
                    touching.setdefault(gi, set()).add(gb)
        return touching

    def step(self, data):
        a, b = P.BELT_A, P.BELT_B
        touching = self._belt_contacts(data)
        for it in self.items:
            if it["slug"] in self.skip:
                continue
            belts = touching.get(it["gid"])
            if not belts:
                continue
            x, y = data.qpos[it["qadr"]:it["qadr"] + 2]
            v = data.qvel[it["dadr"]:it["dadr"] + 3]
            w = data.qvel[it["dadr"] + 3:it["dadr"] + 6]
            if self.gid_beltA in belts:
                front = x + it["half_x"]
                if not self.gate_open and a["gate_x"] - 0.004 <= front < a["gate_x"] + 0.05:
                    # held at the escapement gate; items whose front crossed the
                    # commit line (gate_x + 0.05, same line the gate-state check
                    # uses) are committed downstream and keep moving
                    continue
                if not self.hold2_open and a["hold2_x"] - 0.004 <= front < a["hold2_x"] + 0.05:
                    continue           # held at the pre-gate line
                # arm mode ends belt A at a stop wall (decelerate into it);
                # table mode hands over to the table surface at full speed
                x_end = a["x_stop"] if self.mode == "arm" else None
                if x_end is None or front < x_end - 0.012:
                    taper = 1.0
                    if x_end is not None:
                        # items creep into the stop instead of slamming, and
                        # the drive cuts 12 mm early so the settle detector
                        # can see the item actually stop
                        taper = np.clip((x_end - front) / 0.30, 0.08, 1.0)
                    if not self.gate_open and front < a["gate_x"] + 0.05:
                        # approaching a closed gate: decelerate to a stop at it
                        taper = min(float(taper), float(np.clip((a["gate_x"] - front) / 0.30, 0.0, 1.0)))
                    if not self.hold2_open and front < a["hold2_x"] + 0.05:
                        taper = min(float(taper), float(np.clip((a["hold2_x"] - front) / 0.30, 0.0, 1.0)))
                    v[0] = a["speed"] * taper
                    v[1] = 0.8 * (a["y"] - y)
                    w *= SPIN_DAMP
            elif self.gid_beltB in belts:
                v[1] = b["speed"]
                v[0] = 0.8 * (b["cx"] - x)
                w *= SPIN_DAMP
