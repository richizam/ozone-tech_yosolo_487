# -*- coding: utf-8 -*-
"""2D geometry primitives for the perception pipeline (numpy/scipy only)."""
import numpy as np
from scipy.spatial import ConvexHull


def min_area_rect(pts_xy):
    """Minimum-area bounding rectangle of 2D points (rotating calipers over
    hull edges). Returns (length, width, angle_rad) with length >= width;
    angle is the direction of the LENGTH axis."""
    pts = np.asarray(pts_xy, dtype=float)
    hull = ConvexHull(pts)
    hp = pts[hull.vertices]
    best = None
    for i in range(len(hp)):
        e = hp[(i + 1) % len(hp)] - hp[i]
        n = np.linalg.norm(e)
        if n < 1e-12:
            continue
        ux, uy = e / n
        rot = np.array([[ux, uy], [-uy, ux]])
        q = hp @ rot.T
        w = q[:, 0].max() - q[:, 0].min()
        h = q[:, 1].max() - q[:, 1].min()
        area = w * h
        if best is None or area < best[0]:
            ang = np.arctan2(uy, ux)
            if w >= h:
                best = (area, w, h, ang)
            else:
                best = (area, h, w, ang + np.pi / 2)
    _, L, W, ang = best
    return float(L), float(W), float(ang)


def fit_circle(pts_2d):
    """Algebraic (Kasa) least-squares circle fit.
    Returns (center(2), radius, rms_residual)."""
    p = np.asarray(pts_2d, dtype=float)
    x, y = p[:, 0], p[:, 1]
    A = np.column_stack([x, y, np.ones_like(x)])
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = sol[0] / 2, sol[1] / 2
    r2 = sol[2] + cx * cx + cy * cy
    if r2 <= 0:
        return None
    r = float(np.sqrt(r2))
    d = np.hypot(x - cx, y - cy)
    rms = float(np.sqrt(np.mean((d - r) ** 2)))
    return (float(cx), float(cy)), r, rms


def upper_envelope(pts_2d, bin_w=0.003):
    """Upper envelope of (u, z) points: per u-bin maximum z.
    Returns (M,2) array ordered by u."""
    p = np.asarray(pts_2d, dtype=float)
    order = np.argsort(p[:, 0])
    p = p[order]
    bins = np.floor(p[:, 0] / bin_w).astype(int)
    env = {}
    for b, (u, z) in zip(bins, p):
        if b not in env or z > env[b][1]:
            env[b] = (u, z)
    out = np.array([env[b] for b in sorted(env)])
    return out


def extend_undercut(env, z_mid, k=3, slope_range=(1.0, 6.0), max_ext=0.015):
    """Reconstruct hidden mid-height corners of an upper envelope.

    If an outer end of the envelope descends outward with a finite slope
    (a prism slant seen from above), the section keeps widening below the
    last visible sample. Extend that slant line down to the mirror plane
    z_mid to recover the corner. Box sides are vertical (slope out of range:
    no extension); smooth convex bodies go tangent-vertical at their widest
    line (also no extension). Returns env plus any reconstructed corners.
    """
    env = np.asarray(env, dtype=float)
    extra = []
    for side in (-1, 1):
        pts = env[np.argsort(env[:, 0])][-k:] if side > 0 else env[np.argsort(env[:, 0])][:k]
        end = pts[np.argmax(pts[:, 0] * side)]
        if end[1] <= z_mid + 0.004:
            continue
        du = pts[:, 0].max() - pts[:, 0].min()
        if du < 1e-6:
            continue
        slope = np.polyfit(pts[:, 0], pts[:, 1], 1)[0] * side   # dz per outward du
        if not (-slope_range[1] <= slope <= -slope_range[0]):
            continue
        ext = min((end[1] - z_mid) / (-slope), max_ext)
        extra.append([end[0] + side * ext, z_mid])
    if extra:
        env = np.vstack([env, extra])
        env = env[np.argsort(env[:, 0])]
    return env


def arc_span_deg(pts_2d, center):
    """Angular coverage of points around a center, in degrees."""
    p = np.asarray(pts_2d, dtype=float)
    ang = np.arctan2(p[:, 1] - center[1], p[:, 0] - center[0])
    ang = np.sort(ang)
    if len(ang) < 2:
        return 0.0
    gaps = np.diff(np.concatenate([ang, [ang[0] + 2 * np.pi]]))
    return float(np.degrees(2 * np.pi - gaps.max()))
