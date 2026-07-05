# -*- coding: utf-8 -*-
"""Tri-directional transfer table: kinematic controlled-surface model.

The physical concept is a powered multi-directional roller table (ARB-class):
items in contact with the table surface are driven by the commanded lane —
forward on the entry strip, then north (B connector), east (chute to cage C)
or south (chute to cage D) once inside the routing zone. Chutes are passive
(gravity). Same modelling idiom as the belts: the drive writes linear velocity
while contact exists, plus angular damping (what rollers do to a light item).

Fault injection: `freeze(slug)` makes the table stop driving that item
(a snag/jam) until `unfreeze` — used by the fault scenarios; the watchdog in
run_sim then triggers the arm recovery.
"""
import numpy as np

from cell import params as P

SPIN_DAMP = 0.85

# an item's trailing edge is past the gate plane here -> safe to close again
# (thresholds include the largest item half-extent + margin)
GATE_CLEAR = {"B": ("y", +1, 3.85), "C": ("x", +1, 8.90), "D": ("y", -1, 2.15)}


class Table:
    def __init__(self, model, item_entries):
        self.m = model
        self.gid_table = model.geom("table").id
        self.gid_connect = model.geom("connectB").id
        self.items = []
        self.by_gid = {}
        self.by_slug = {}
        for e in item_entries:
            jid = model.joint(f"fj_{e['slug']}").id
            it = {"slug": e["slug"], "gid": model.geom(f"g_{e['slug']}").id,
                  "qadr": model.jnt_qposadr[jid], "dadr": model.jnt_dofadr[jid]}
            self.items.append(it)
            self.by_gid[it["gid"]] = it
            self.by_slug[it["slug"]] = it
        self.routes = {}               # slug -> "B" | "C" | "D"
        self.skip = set()              # slugs held by the arm
        self.frozen = set()            # slugs with an injected snag (fault scenario)
        self.paused = False            # global pause during arm recovery
        self.hold_open = set()         # zones held open for an arm recovery path
        # normally-closed exit gates (position actuators added by scene.py)
        self.gate_aid = {z: model.actuator(f"ga{z}").id for z in "BCD"}
        self.gate_qadr = {z: model.jnt_qposadr[model.joint(f"gj{z}").id] for z in "BCD"}

    def gate_open_frac(self, data):
        """Per-zone gate opening 0..1 (for lights and telemetry)."""
        tr = P.GATES["travel"]
        return {z: float(np.clip(data.qpos[q] / tr, 0.0, 1.0))
                for z, q in self.gate_qadr.items()}

    def _drive_gates(self, data):
        """A gate opens only while a routed item still needs its exit (or an
        arm recovery is using it) — the table is otherwise fully bounded."""
        want = {z: z in self.hold_open for z in "BCD"}
        for slug, route in self.routes.items():
            if slug in self.skip or route not in want:
                continue
            it = self.by_slug[slug]
            x, y = data.qpos[it["qadr"]:it["qadr"] + 2]
            ax, sgn, thr = GATE_CLEAR[route]
            v = x if ax == "x" else y
            if sgn * (v - thr) < 0:    # trailing edge not yet past the plane
                want[route] = True
        tr = P.GATES["travel"]
        for z, open_ in want.items():
            data.ctrl[self.gate_aid[z]] = tr if open_ else 0.0

    def freeze(self, slug):
        self.frozen.add(slug)

    def unfreeze(self, slug):
        self.frozen.discard(slug)

    def _contacts(self, data):
        touching = {}
        for i in range(data.ncon):
            c = data.contact[i]
            for gi, gs in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if gs in (self.gid_table, self.gid_connect) and gi in self.by_gid:
                    touching.setdefault(gi, set()).add(gs)
        return touching

    def step(self, data):
        tb, cb = P.TABLE, P.CONNECT_B
        self._drive_gates(data)
        touching = self._contacts(data)
        for it in self.items:
            slug = it["slug"]
            if slug in self.skip:
                continue
            surfaces = touching.get(it["gid"])
            if not surfaces:
                continue
            x, y = data.qpos[it["qadr"]:it["qadr"] + 2]
            v = data.qvel[it["dadr"]:it["dadr"] + 3]
            w = data.qvel[it["dadr"] + 3:it["dadr"] + 6]
            if self.paused or slug in self.frozen:
                v[0] = v[1] = 0.0
                w *= SPIN_DAMP
                continue
            if self.gid_connect in surfaces:
                v[0] = 0.8 * (cb["cx"] - x)
                v[1] = cb["speed"]
            else:
                route = self.routes.get(slug)
                if x < tb["route_x"]:
                    v[0] = tb["speed"]
                    v[1] = 0.8 * (tb["y"] - y)
                elif route == "B":
                    v[0] = 0.8 * (tb["lane_B_cx"] - x)
                    v[1] = tb["speed"]
                elif route == "C":
                    v[0] = tb["speed"]
                    v[1] = 0.8 * (tb["lane_C_cy"] - y)
                elif route == "D":
                    v[0] = 0.8 * (tb["lane_D_cx"] - x)
                    v[1] = -tb["speed"]
                else:
                    # no route yet (should not happen: the gate releases only
                    # classified items) — hold at the routing-zone entry
                    v[0] = 0.0 if x > tb["route_x"] - 0.05 else tb["speed"]
                    v[1] = 0.8 * (tb["y"] - y)
            w *= SPIN_DAMP
