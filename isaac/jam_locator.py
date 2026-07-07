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


# route corridors: polyline per route the stuck item should be on
def _route_polyline(route):
    tb = P.TABLE
    if route == "B":
        cb = P.CONNECT_B
        return [(tb["x0"], tb["y"]), (tb["route_x"], tb["y"]),
                (tb["lane_B_cx"], tb["y"]), (cb["cx"], cb["y1"]),
                (P.BELT_B["cx"], P.BELT_B["y_delivered"])]
    if route == "C":
        cc = P.CHUTE_C
        return [(tb["x0"], tb["y"]), (tb["route_x"], tb["y"]),
                (tb["x1"], cc["cy"]), (cc["pad_x1"], cc["cy"])]
    if route == "D":
        cd = P.CHUTE_D
        return [(tb["x0"], tb["y"]), (tb["route_x"], tb["y"]),
                (cd["cx"], cd["y0"]), (cd["cx"], cd["pad_y1"])]
    return [(tb["x0"], tb["y"]), (tb["x1"], tb["y"])]


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
                 min_points=60, blade_lines=()):
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
        # delivered items inside the roll-cages are legitimate foreground —
        # the cages are terminals, not routing surfaces: exclude their interiors
        cages = P.cages_for("table")
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
                # overhead hardware, not freight: an OPEN exit gate's panel
                # hangs at z ~1.4-1.7 and differs from the closed-state
                # background
                continue
            half = (pts.max(axis=0) - pts.min(axis=0)) / 2
            # a RAISED stop blade is foreground too: an x-thin sliver sitting
            # exactly on a known blade line is hardware, not a stuck item
            if half[0] < 0.02 and any(abs(ctr[0] - bx) < 0.06
                                      for bx in self.blade_lines):
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
