# -*- coding: utf-8 -*-
"""Kinematic conveyor model (belt A + knife nose, B connector, belt B).

An item is driven at belt speed ONLY while it is in physical contact with a
powered-surface geom (v0 simplification of a friction-driven belt: the drive
writes the linear velocity; belt friction is modeled as strong damping of the
item's angular velocity, which is what a real belt does to a tumbling light
object). Acceleration is limited to INDUCT["accel_mps2"] — a gate-held item
re-accelerates under belt friction exactly as the escapement's induction
timing model (cell/sorter.py offer()) assumes.

These writes are the CONVEYOR SURFACE DRIVE — the twin's analogue of the
Isaac build's PhysX kinematic surface velocity — and are counted separately
(drive_writes) from direct per-item scripted writes, which the sorter
executive has none of (direct_velocity_writes_nominal = 0): once an item
leaves the knife nose it is moved by tray contact and gravity only.

Holds are per-item escapements: the BELT never stops (conveyor_a_stop_count
is structurally 0), items decelerate into the normally-closed escapement
blade / pre-gate hold line while the belt runs on beneath them.
"""
import os

import numpy as np

from cell import params as P

SPIN_DAMP = 0.85          # per-step angular velocity retention on belt


class Belts:
    def __init__(self, model, item_entries, mode=None):
        self.m = model
        self.gid_beltA = model.geom("beltA").id
        self.gid_knife = model.geom("knife").id
        self.gid_beltB = model.geom("beltB").id
        self.gid_connect = model.geom("bconnect").id
        bc = P.B_CONNECT
        self._conn_vy = bc["speed"] * float(np.cos(np.arctan2(
            bc["z_top1"] - bc["z_top0"], bc["y1"] - bc["y0"])))
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
        self.gate_open = False         # escapement: NORMALLY CLOSED — opens only
                                       # for a synchronized release (cell/sorter)
        self.hold2_open = True         # pre-gate hold (keeps the vision window
                                       # single-item while somebody is at the gate)
        self.drive_writes = 0          # conveyor-surface drive writes (telemetry)

    def _belt_contacts(self, data):
        """gid -> set of powered-surface geom ids the item touches this step."""
        surfaces = (self.gid_beltA, self.gid_knife, self.gid_beltB,
                    self.gid_connect)
        touching = {}
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = c.geom1, c.geom2
            for gi, gb in ((g1, g2), (g2, g1)):
                if gb in surfaces and gi in self.by_gid:
                    touching.setdefault(gi, set()).add(gb)
        return touching

    QUEUE_GAP = 0.15           # enforced clearance between queued items:
                               # zone-accumulation (slug) spacing — queued items
                               # NEVER touch, so a light thin item cannot be
                               # shoved onto or under a neighbor, and clouds of
                               # queued items can never merge

    def _queue_lines(self, data):
        """Per-item stop lines while the pre-gate hold is closed.

        The front-most item between the hold and the gate commit line owns the
        measurement corridor (exempt — it must keep moving; holding it would
        deadlock the corridor on itself). Items behind it stop at staggered
        lines: hold2, then hold2 - (len+gap), ... — an accumulation zone."""
        a = P.BELT_A
        lines = {}
        on_a = []
        for it in self.items:
            if it["slug"] in self.skip:
                continue
            x, y = data.qpos[it["qadr"]:it["qadr"] + 2]
            if abs(y - a["y"]) > 0.4:
                continue
            front = x + it["half_x"]
            if front - 2 * it["half_x"] > a["gate_x"] + 0.05:
                continue                      # rear past the commit line:
                                              # committed downstream
            on_a.append((front, it))
        on_a.sort(key=lambda p: -p[0])
        if on_a and on_a[0][0] > a["hold2_x"] - 0.004:
            on_a.pop(0)                       # corridor owner: exempt
        line = a["hold2_x"]
        for front, it in on_a:
            # an item already past its computed line freezes where it is (the
            # `front >= line` hold in step()) — never exempt, or it would
            # tailgate the owner through the measurement corridor
            lines[it["gid"]] = line
            line -= 2 * it["half_x"] + self.QUEUE_GAP
        return lines

    def step(self, data, dt):
        a, b = P.BELT_A, P.BELT_B
        acc = P.INDUCT["accel_mps2"] * dt
        touching = self._belt_contacts(data)
        hold_lines = self._queue_lines(data) if not self.hold2_open else {}
        for it in self.items:
            if it["slug"] in self.skip:
                continue
            belts = touching.get(it["gid"], set())
            x, y, z = data.qpos[it["qadr"]:it["qadr"] + 3]
            # position-window fallback: a long item TEETERING over the knife
            # nose (rear lifted, belly caught on the arriving tray's lip)
            # loses belt contact but is still the conveyor's freight until
            # its CENTRE passes the nose — a real belt keeps dragging it
            # through the teeter (the pouf ground to 0.05 m/s here otherwise)
            on_a_zone = (0.0 < x < a["nose_x"] and abs(y - a["y"]) < 0.32
                         and z < 1.05
                         and not (belts & {self.gid_beltB, self.gid_connect}))
            if not belts and not on_a_zone:
                continue
            v = data.qvel[it["dadr"]:it["dadr"] + 3]
            w = data.qvel[it["dadr"] + 3:it["dadr"] + 6]
            if (self.gid_beltA in belts or self.gid_knife in belts
                    or on_a_zone):
                front = x + it["half_x"]
                if not self.gate_open and a["gate_x"] - 0.004 <= front < a["gate_x"] + 0.05:
                    # held at the escapement blade; items whose front crossed
                    # the commit line (gate_x + 0.05, same line the gate-state
                    # check uses) are committed downstream and keep moving
                    continue
                hold_at = hold_lines.get(it["gid"])
                if hold_at is not None and front >= hold_at - 0.004:
                    continue                  # holding at the queue stop line
                taper = 1.0
                if not self.gate_open and front < a["gate_x"] + 0.05:
                    # approaching a closed escapement: decelerate to a stop
                    # just short of the raised blade face
                    taper = float(np.clip((a["gate_x"] - 0.014 - front) / 0.30,
                                          0.0, 1.0))
                if hold_at is not None and front < hold_at:
                    taper = min(taper, float(np.clip((hold_at - front) / 0.30,
                                                     0.0, 1.0)))
                # belt-friction re-acceleration limit (matches INDUCT model)
                v[0] = min(a["speed"] * taper, max(v[0], 0.0) + acc)
                v[1] = 0.8 * (a["y"] - y)
                if x < a["knife_x0"]:
                    # spin-damp only on the full belt body: over the knife
                    # the natural nose-over pitch must develop freely
                    w *= SPIN_DAMP
                self.drive_writes += 1
            elif self.gid_connect in belts:
                # powered incline: drive the horizontal along-belt component;
                # the surface contact resolves the climb (vz)
                if os.environ.get("TWIN_TRACE"):
                    print(f"[conn] {it['slug']} v1={v[1]:.3f} -> "
                          f"{min(self._conn_vy, max(v[1], 0.0) + acc):.3f}",
                          flush=True)
                v[1] = min(self._conn_vy, max(v[1], 0.0) + acc)
                v[0] = 0.8 * (P.B_CONNECT["cx"] - x)
                w *= SPIN_DAMP
                self.drive_writes += 1
            elif self.gid_beltB in belts:
                v[1] = min(b["speed"], max(v[1], 0.0) + acc)
                v[0] = 0.8 * (b["cx"] - x)
                w *= SPIN_DAMP
                self.drive_writes += 1
