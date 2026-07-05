# -*- coding: utf-8 -*-
"""Runtime presentation state: the cell SHOWS what it is doing.

Pure rgba writes on visual-only geoms (zero physics effect), driven from the
control loop at 50 Hz:
  - lane markings / chevrons / chute flow arrows pulse in the active route's
    colour while an item is being driven down that lane;
  - destination beacons pulse on the active route;
  - gate lamps track the actual gate opening (dim = locked, bright = open);
  - the andon tower reads green (idle flow), amber (routing in progress),
    red pulsing (jam detected / arm recovery);
  - each item is tinted with its PERCEIVED category colour the moment the
    classifier commits — the jury literally watches the decision attach to
    the object and the matching coloured path light up.
"""
import numpy as np
import mujoco

from cell import params as P

PULSE_HZ = 1.6


class Visuals:
    def __init__(self, model):
        self.m = model
        self.lanes = {z: [] for z in "BCD"}
        self.beacon, self.glamp, self.tower = {}, {}, {}
        for gid in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            for z in "BCD":
                if name.startswith((f"lane{z}_", f"flow{z}")):
                    self.lanes[z].append(gid)
            if name.startswith("claneB"):
                self.lanes["B"].append(gid)
            if name.startswith("beacon") and "pole" not in name:
                self.beacon[name[-1]] = gid
            if name.startswith("glamp"):
                self.glamp[name[-1]] = gid
            if name.startswith("tower_") and "pole" not in name:
                self.tower[name[-1]] = gid
        self._item_orig = {}

    # ------------------------------------------------------------ per event
    def set_item_zone(self, slug, zone):
        """Tint an item with its perceived-category colour (decision made)."""
        try:
            gid = self.m.geom(f"g_{slug}").id
        except KeyError:
            return
        if gid not in self._item_orig:
            self._item_orig[gid] = self.m.geom_rgba[gid].copy()
        orig = self._item_orig[gid]
        col = P.ROUTE_RGBA.get(zone)
        if col is None:
            return
        self.m.geom_rgba[gid, :3] = 0.40 * orig[:3] + 0.60 * np.array(col)
        self.m.geom_rgba[gid, 3] = 1.0

    # ------------------------------------------------------------ every tick
    def update(self, t, active_zones, jam, gates_frac=None):
        pulse = 0.5 + 0.5 * float(np.sin(2 * np.pi * PULSE_HZ * t))
        for z in "BCD":
            col = np.array(P.ROUTE_RGBA[z])
            hot = z in active_zones
            k = (0.55 + 0.65 * pulse) if hot else 0.40
            rgb = np.clip(col * k, 0, 1)
            for gid in self.lanes[z]:
                self.m.geom_rgba[gid, :3] = rgb
                self.m.geom_rgba[gid, 3] = 0.95 if hot else 0.75
            bid = self.beacon.get(z)
            if bid is not None:
                bk = (0.75 + 0.45 * pulse) if hot else 0.30
                self.m.geom_rgba[bid, :3] = np.clip(col * bk, 0, 1)
                self.m.geom_rgba[bid, 3] = 1.0
            lid = self.glamp.get(z)
            if lid is not None and gates_frac is not None:
                fr = gates_frac.get(z, 0.0)
                self.m.geom_rgba[lid, :3] = np.clip(col * (0.30 + 0.95 * fr), 0, 1)
                self.m.geom_rgba[lid, 3] = 1.0
        # andon tower: green run / amber routing / red jam
        states = {"g": (not jam and not active_zones), "a": bool(active_zones) and not jam,
                  "r": jam}
        for key, on in states.items():
            gid = self.tower.get(key)
            if gid is None:
                continue
            a = (0.95 if key != "r" else 0.55 + 0.45 * pulse) if on else 0.22
            self.m.geom_rgba[gid, 3] = a
