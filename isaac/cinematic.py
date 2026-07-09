# -*- coding: utf-8 -*-
"""Cinematic camera paths for the final defense reel (visual export only).

Each path maps a phase in [0, 1] (run progress) to (eye, target) world
points. All paths are slow, stay on the OPEN south/south-west side of the
cell, keep the action framed, and never put a wall (north y=6.35, east
x=10.6) or the folded arm (~8.5, 2.2) between the camera and the freight —
technical visibility is preserved (no blur, no whip pans, no occlusion).

pose_camera() writes the eye/look-at onto a camera prim each rendered frame.
"""
import math

import numpy as np
from pxr import Gf, UsdGeom


def _smooth(p):
    """Ease-in-out so the move starts and ends gently (no sudden motion)."""
    p = float(np.clip(p, 0.0, 1.0))
    return p * p * (3.0 - 2.0 * p)


def _orbit(p):
    # slow arc across the open south-west, looking at the cell heart
    a0, a1 = math.radians(203.0), math.radians(256.0)
    a = a0 + (a1 - a0) * _smooth(p)
    cx, cy, r, h = 5.2, 3.0, 6.9, 3.7
    eye = (cx + r * math.cos(a), cy + r * math.sin(a), h)
    return eye, (5.4, 3.0, 0.85)


def _dolly(p):
    # tracking dolly along the flow, from the south lane, gently rising
    s = _smooth(p)
    x = 1.1 + (8.7 - 1.1) * s
    eye = (x, 1.05, 1.95 + 0.25 * s)
    return eye, (min(x + 0.7, 9.0), 3.0, 0.72)


def _crane(p):
    # crane-up reveal from a low front to a high isometric
    s = _smooth(p)
    eye = (5.2, -1.1 - 1.6 * s, 1.5 + 3.3 * s)
    return eye, (6.1, 3.0, 0.8)


def _deck_push(p):
    # slow push onto the ARB deck + discharge from the open south-east
    s = _smooth(p)
    eye = (9.05 - 0.32 * s, 1.15 + 0.32 * s, 1.75 - 0.42 * s)
    return eye, (8.25, 3.0, 0.74)


PATHS = {"orbit": _orbit, "dolly": _dolly, "crane": _crane,
         "deck_push": _deck_push}


def _look_quat(eye, target):
    eye = np.asarray(eye, float)
    f = np.asarray(target, float) - eye
    f /= max(np.linalg.norm(f), 1e-9)
    z = -f                                    # USD camera looks along -Z
    x = np.cross((0.0, 0.0, 1.0), z)
    n = np.linalg.norm(x)
    x = np.array([1.0, 0.0, 0.0]) if n < 1e-6 else x / n
    y = np.cross(z, x)
    m = np.column_stack([x, y, z])
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s)
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s,
                 (m[0, 2] + m[2, 0]) / s)
        elif i == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s,
                 (m[1, 2] + m[2, 1]) / s)
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                 (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    return tuple(float(v) for v in q)


class CinematicCamera:
    """Re-poses a camera prim along a named path each rendered frame."""

    def __init__(self, stage, cam_path, path_name):
        self.fn = PATHS[path_name]
        prim = stage.GetPrimAtPath(cam_path)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        self.t_op = xf.AddTranslateOp()
        self.o_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

    def update(self, phase):
        eye, target = self.fn(phase)
        w, x, y, z = _look_quat(eye, target)
        self.t_op.Set(Gf.Vec3d(*[float(v) for v in eye]))
        self.o_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
