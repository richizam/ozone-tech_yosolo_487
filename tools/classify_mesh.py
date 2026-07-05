# -*- coding: utf-8 -*-
"""Reference classifier — executable form of the official Track 3 rules.

Category decision (priority order per the task statement):
  1. Dimensions gate: any OBB dim < 10 mm (undersize) or sorted dims exceed
     450x320x320 mm (oversize)  -> C  ("Не подходит для сортировки по габаритам")
  2. Circle-in-cross-section: max over principal-axis sections of
     r_inscribed / R_circumscribed >= 0.8                      -> D  ("Не подходит без доупаковки")
  3. Otherwise                                                 -> B  ("Подходит для сортировки")

Geometry notes:
  - Dimensions are oriented-bounding-box extents (orientation-invariant).
  - Sections are taken perpendicular to the OBB principal axes at 19 stations.
  - Per section: outer contour approximated by the convex hull of the plane
    intersection; R = minimum enclosing circle radius (shapely),
    r = Chebyshev-center radius (exact LP solution for convex polygons).

Usage:
  python tools/classify_mesh.py path/to/model.stl
  python tools/classify_mesh.py --all path/to/stl_dir --json out.json
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import shapely
import trimesh
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon

MIN_DIM_MM = 10.0
MAX_DIMS_MM = np.array([450.0, 320.0, 320.0])  # sorted descending
RATIO_THRESHOLD = 0.8
SECTION_STATIONS = np.linspace(0.05, 0.95, 19)

CATEGORY_NAMES = {
    "B": "Подходит для сортировки",
    "C": "Не подходит для сортировки по габаритам",
    "D": "Не подходит для сортировки без доупаковки",
}


def _plane_basis(normal):
    n = normal / np.linalg.norm(normal)
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    return u, v


def chebyshev_radius(points_2d):
    """Exact max inscribed circle radius of the convex hull of 2D points."""
    hull = ConvexHull(points_2d)
    pts = points_2d[hull.vertices]
    a_ub, b_ub = [], []
    for i in range(len(pts)):
        p, q = pts[i], pts[(i + 1) % len(pts)]
        edge = q - p
        inward = np.array([-edge[1], edge[0]])
        norm = np.linalg.norm(inward)
        if norm < 1e-12:
            continue
        inward /= norm
        a_ub.append([-inward[0], -inward[1], 1.0])
        b_ub.append(-float(np.dot(inward, p)))
    res = linprog(
        c=[0.0, 0.0, -1.0],
        A_ub=np.array(a_ub),
        b_ub=np.array(b_ub),
        bounds=[(None, None), (None, None), (0.0, None)],
        method="highs",
    )
    return float(res.x[2]) if res.success else None


def section_ratio(mesh, origin, normal):
    """r_inscribed / R_circumscribed of one planar section, or None if empty."""
    segments = trimesh.intersections.mesh_plane(mesh, plane_normal=normal, plane_origin=origin)
    if segments is None or len(segments) == 0:
        return None
    pts3 = segments.reshape(-1, 3)
    u, v = _plane_basis(np.asarray(normal, dtype=float))
    rel = pts3 - origin
    pts2 = np.column_stack([rel @ u, rel @ v])
    if len(pts2) < 6:
        return None
    try:
        hull_poly = Polygon(pts2[ConvexHull(pts2).vertices])
    except Exception:
        return None
    big_r = shapely.minimum_bounding_radius(hull_poly)
    if big_r <= 1e-9:
        return None
    small_r = chebyshev_radius(pts2)
    return None if small_r is None else small_r / big_r


def classify_mesh(mesh):
    """Classify an in-memory trimesh.Trimesh (dimensions in mm). Core rule engine."""
    to_obb, extents = trimesh.bounds.oriented_bounds(mesh)
    mesh = mesh.copy()
    mesh.apply_transform(to_obb)
    extents = np.asarray(extents, dtype=float)
    dims_desc = np.sort(extents)[::-1]
    lo, hi = mesh.bounds

    undersize = bool(np.any(dims_desc < MIN_DIM_MM))
    oversize = bool(np.any(dims_desc > MAX_DIMS_MM))

    best_ratio, best_axis = 0.0, None
    axis_ratios = {}
    for axis in range(3):
        normal = np.zeros(3)
        normal[axis] = 1.0
        span = hi[axis] - lo[axis]
        ratios = []
        for station in SECTION_STATIONS:
            origin = np.zeros(3)
            origin[axis] = lo[axis] + span * station
            ratio = section_ratio(mesh, origin, normal)
            if ratio is not None:
                ratios.append(ratio)
        axis_max = max(ratios) if ratios else 0.0
        axis_ratios["XYZ"[axis]] = round(axis_max, 3)
        if axis_max > best_ratio:
            best_ratio, best_axis = axis_max, "XYZ"[axis]

    if undersize or oversize:
        zone = "C"
        reason = (
            f"undersize: min dim {dims_desc[-1]:.1f} mm < {MIN_DIM_MM:.0f} mm"
            if undersize
            else f"oversize: dims {dims_desc.round(1).tolist()} exceed {MAX_DIMS_MM.astype(int).tolist()}"
        )
    elif best_ratio >= RATIO_THRESHOLD:
        zone = "D"
        reason = f"circle in section: r_in/R_circ={best_ratio:.3f} >= {RATIO_THRESHOLD} (axis {best_axis})"
    else:
        zone = "B"
        reason = f"max r_in/R_circ={best_ratio:.3f} < {RATIO_THRESHOLD}"

    return {
        "dims_mm": [round(float(d), 1) for d in dims_desc],
        "undersize": undersize,
        "oversize": oversize,
        "axis_ratios": axis_ratios,
        "max_ratio": round(best_ratio, 3),
        "zone": zone,
        "category": CATEGORY_NAMES[zone],
        "reason": reason,
    }


def classify(path):
    """Classify a mesh file (STL/STEP-converted/OBJ...; dimensions in mm)."""
    mesh = trimesh.load(str(path), force="mesh")
    result = classify_mesh(mesh)
    return {"file": Path(path).name, **result}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", help="mesh file, or a directory with --all")
    parser.add_argument("--all", action="store_true", help="classify every .stl/.STL in the directory")
    parser.add_argument("--json", metavar="OUT", help="also write results to a JSON file")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8")
    target = Path(args.path)
    files = sorted(p for p in target.iterdir() if p.suffix.lower() == ".stl") if args.all else [target]

    results = []
    for f in files:
        try:
            result = classify(f)
        except Exception as exc:  # keep going: one bad mesh must not sink the batch
            result = {"file": f.name, "error": repr(exc)}
        results.append(result)
        print(json.dumps(result, ensure_ascii=False))

    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\nSaved -> {out}")


if __name__ == "__main__":
    main()
