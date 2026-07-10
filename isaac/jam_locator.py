# -*- coding: utf-8 -*-
"""Jam localization with a REAL camera (user requirement): when the routing
watchdog reports a stuck item, the cell must find WHERE it is from sensor
data, not from simulator state.

Method — classic industrial change detection on the routing-zone overhead
depth camera:
  1. background model: N averaged depth frames of the EMPTY cell captured at
     startup (before the first spawn);
  2. on jam: render, take depth, foreground = pixels sitting > tau above the
     background surface, back-project to world points;
  3. cluster foreground into connected components on a coarse grid (pure
     numpy BFS — no scipy) and drop clusters too small to be an item;
  4. pick the cluster nearest the jammed route's corridor and return its
     centroid + extents. The caller logs the offset vs the true sim pose as
     localization-accuracy evidence and hands the position to the recovery
     path (exception arm / operator call-out).
"""
from collections import deque

import numpy as np

from cell import params as P


# route corridors: polyline per route the stuck item should be on (train
# axis to the station, then down the station's chute / connector)
def _route_polyline(route):
    S = P.SORTER
    y = S["y"]
    if route == "B":
        bc = P.B_CONNECT
        return [(S["x_west"], y), (P.STATIONS["B"]["x"], y),
                (bc["cx"], bc["y1"]),
                (P.BELT_B["cx"], P.BELT_B["y_delivered"])]
    if route in ("C", "D"):
        cc = P.CHUTE_C if route == "C" else P.CHUTE_D
        y1, pad_end = P.chute_run(cc)
        return [(S["x_west"], y), (cc["cx"], y), (cc["cx"], cc["y0"]),
                (cc["cx"], pad_end)]
    if route == "REVIEW":
        cc = P.CHUTE_REVIEW
        y1, pad_end = P.chute_run(cc)
        return [(S["x_west"], y), (cc["cx"], y), (cc["cx"], cc["y0"]),
                (cc["cx"], pad_end)]
    return [(S["x_west"], y), (S["x_east"], y)]


def _dist_to_polyline(pt, poly):
    p = np.asarray(pt, float)
    best = np.inf
    for a, b in zip(poly[:-1], poly[1:]):
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-12 else float(np.clip((p - a) @ ab / L2, 0, 1))
        best = min(best, float(np.linalg.norm(a + t * ab - p)))
    return best


class JamLocator:
    """Background-subtraction localizer on the routing-zone depth camera."""

    def __init__(self, camera, fg_tau=0.008, cell_m=0.04, min_cells=4,
                 min_points=60, blade_lines=(), hood_zones=()):
        # fg_tau 8 mm: RTX depth is noise-free, and a 9 mm pen must clear the
        # threshold. Known blind spot: the chute discharge HOODS occlude the
        # overhead view — a hang-up under a guard is an operator call-out in
        # a real cell anyway (lockout/tagout); upgrade path = one angled
        # camera per chute.
        self.cam = camera
        self.fg_tau = fg_tau
        self.cell = cell_m
        self.min_cells = min_cells
        self.min_points = min_points
        self.blade_lines = list(blade_lines)       # x of pop-up stop blades
        self.hood_zones = list(hood_zones)         # (x0,x1,y0,y1) guarded
        self.bg = None

    # -------------------------------------------------- shared with perception
    @staticmethod
    def _cam_pose_usd(cam):
        from pxr import Usd, UsdGeom
        xf = UsdGeom.Xformable(cam.prim)
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        R = np.array([[m[0][0], m[0][1], m[0][2]],
                      [m[1][0], m[1][1], m[1][2]],
                      [m[2][0], m[2][1], m[2][2]]], dtype=float).T
        pos = np.array([m[3][0], m[3][1], m[3][2]], dtype=float)
        return pos, R

    def _depth(self):
        frame = self.cam.get_current_frame()
        d = frame.get("distance_to_image_plane")
        if d is None:
            return None
        d = np.asarray(d, dtype=np.float32)
        return d if d.ndim == 2 and d.size else None

    def build_background(self, n=5, renderer=None):
        """Average N depth frames of the EMPTY cell (call before first spawn).
        renderer: callable that renders one frame (e.g. world.render)."""
        acc = []
        for _ in range(n):
            if renderer is not None:
                renderer()
            d = self._depth()
            if d is not None:
                acc.append(d)
        if acc:
            self.bg = np.median(np.stack(acc, axis=0), axis=0)
        return self.bg is not None

    # ------------------------------------------------------------- localization
    def locate(self, route=None):
        """-> dict(pos_xyz, half_extents, n_points, n_clusters) or None.
        Foreground = closer-than-background by fg_tau (something new sits on
        top of the known static scene)."""
        if self.bg is None:
            return None
        d = self._depth()
        if d is None or d.shape != self.bg.shape:
            return None
        fg = (np.isfinite(d) & np.isfinite(self.bg)
              & (self.bg - d > self.fg_tau))
        if fg.sum() < self.min_points:
            return None
        H, W = d.shape
        K = np.asarray(self.cam.get_intrinsics_matrix(), dtype=float)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        pos, R = self._cam_pose_usd(self.cam)
        vs, us = np.nonzero(fg)
        D = d[vs, us]
        cam_pts = np.stack([(us - cx) / fx * D, -(vs - cy) / fy * D, -D],
                           axis=-1)
        world = pos + cam_pts @ R.T

        # grid clustering (connected components, 8-neighbour BFS)
        gx = np.floor(world[:, 0] / self.cell).astype(int)
        gy = np.floor(world[:, 1] / self.cell).astype(int)
        keys = {}
        for i, k in enumerate(zip(gx, gy)):
            keys.setdefault(k, []).append(i)
        labels = {}
        clusters = []
        for k in keys:
            if k in labels:
                continue
            comp = []
            q = deque([k])
            labels[k] = len(clusters)
            while q:
                c0 = q.popleft()
                comp.append(c0)
                for dx_ in (-1, 0, 1):
                    for dy_ in (-1, 0, 1):
                        nb = (c0[0] + dx_, c0[1] + dy_)
                        if nb in keys and nb not in labels:
                            labels[nb] = len(clusters)
                            q.append(nb)
            clusters.append(comp)
        # delivered items inside the roll-cages / review pen are legitimate
        # foreground — terminals, not routing surfaces: exclude their interiors
        cages = dict(P.cages_for())
        cages["REVIEW"] = P.REVIEW_PEN
        def in_cage(xy):
            for cage in cages.values():
                cx, cy2 = cage["center"]
                hx, hy = cage["inner"][0] / 2, cage["inner"][1] / 2
                if abs(xy[0] - cx) < hx + 0.05 and abs(xy[1] - cy2) < hy + 0.05:
                    return True
            return False

        cands = []
        for comp in clusters:
            if len(comp) < self.min_cells:
                continue
            idx = np.concatenate([keys[c] for c in comp])
            pts = world[idx]
            if len(pts) < self.min_points:
                continue
            ctr = pts.mean(axis=0)
            if in_cage(ctr[:2]):
                continue
            if ctr[2] > 1.10:
                # overhead hardware, not freight
                continue
            if 0.55 < ctr[2] < 0.90 and abs(ctr[1] - P.SORTER["y"]) < 0.36:
                # a TILTED TRAY over the train line is commanded machine
                # state, not a stuck item (trays differ from the flat-tray
                # background while discharging)
                continue
            half = (pts.max(axis=0) - pts.min(axis=0)) / 2
            # a RAISED stop blade is foreground too: an x-thin sliver sitting
            # exactly on a known blade line is hardware, not a stuck item
            if half[0] < 0.02 and any(abs(ctr[0] - bx) < 0.06
                                      for bx in self.blade_lines):
                continue
            # razor-thin sliver in ANY horizontal axis = an occlusion-edge
            # artifact (e.g. the chute hood's rim over an item stalled in
            # the hood blind spot), never freight
            if min(half[0], half[1]) < 0.018:
                continue
            # clusters centred inside a chute-hood footprint are the guarded
            # blind spot: partial item slices merged with hood-edge returns
            # give a biased fix — a hang-up under a guard is an operator
            # (lockout/tagout) case, never an arm dispatch
            if any(x0 <= ctr[0] <= x1 and y0 <= ctr[1] <= y1
                   for x0, x1, y0, y1 in self.hood_zones):
                continue
            cands.append({"pos": ctr, "half": half, "n": int(len(pts))})
        if not cands:
            return None
        if route is not None and len(cands) > 1:
            poly = _route_polyline(route)
            cands.sort(key=lambda c: _dist_to_polyline(c["pos"][:2], poly))
        else:
            cands.sort(key=lambda c: -c["n"])
        best = cands[0]
        return {"pos_xyz": [round(float(v), 3) for v in best["pos"]],
                "half_extents": [round(float(v), 3) for v in best["half"]],
                "n_points": best["n"], "n_clusters": len(cands)}
