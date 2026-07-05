# -*- coding: utf-8 -*-
"""Synthetic borderline test items — designed threshold attacks.

The official 11-item set already contains borderline cases (the 435 mm
hexagonal «Цилиндр», the 0.73-ratio detergent, the 9 mm pen). This script adds
five DESIGNED attacks on the exact decision boundaries, for the borderline
scenario suite (scenarios/borderline.yaml):

    bl_box_b   445x315x315 mm box     -> B  (1% inside every max limit)
    bl_box_c   460x330x330 mm box     -> C  (2-3% over every max limit)
    bl_pent_d  pentagonal prism       -> D  (r_in/R = cos 36deg = 0.809 —
                                             0.009 over the 0.8 threshold)
    bl_rsq_b   rounded-square prism   -> B  (corner radius tuned to
                                             ratio ~0.78 — 0.02 UNDER)
    bl_rod_b   220x16x12 mm bar       -> B  (2 mm above the 10 mm min limit)

Ground truth is COMPUTED by the reference classifier (tools/classify_mesh.py),
not asserted. Meshes land in cell/assets/meshes/, entries in
cell/assets/manifest_extra.json (merged into the scene by cell/scene.py).

    .venv/Scripts/python tools/make_borderline_items.py
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.classify_mesh import classify_mesh          # noqa: E402
from cell.prep_assets import obb_flat_transform, DENSITY, MASS_MIN, MASS_MAX  # noqa: E402

OUT_DIR = ROOT / "cell" / "assets" / "meshes"
MANIFEST = ROOT / "cell" / "assets" / "manifest_extra.json"


def convex_prism(section_2d, length_mm):
    """Prism from a CONVEX 2D section (fan-triangulated caps + side quads) —
    avoids trimesh's optional triangulation-engine dependency."""
    pts = np.asarray(section_2d, dtype=float)
    n = len(pts)
    lo = np.column_stack([pts, np.zeros(n)])
    hi = np.column_stack([pts, np.full(n, length_mm)])
    verts = np.vstack([lo, hi])
    faces = []
    for i in range(1, n - 1):                    # caps (fan; convex section)
        faces.append([0, i + 1, i])              # bottom, normal -z
        faces.append([n, n + i, n + i + 1])      # top, normal +z
    for i in range(n):                           # sides
        j = (i + 1) % n
        faces.append([i, j, n + i])
        faces.append([j, n + j, n + i])
    mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=True)
    mesh.fix_normals()
    return mesh


def rounded_square_prism(side_mm, corner_r_mm, length_mm):
    """Prism over a square with filleted corners: ratio = r_in/R where
    r_in = side/2, R = sqrt(2)*(side/2 - r) + r."""
    half, r = side_mm / 2.0, corner_r_mm
    c = half - r                                 # fillet center offset
    section = []
    corners = [(c, c, 0.0), (-c, c, np.pi / 2), (-c, -c, np.pi),
               (c, -c, 3 * np.pi / 2)]
    for cx, cy, a0 in corners:
        for a in np.linspace(a0, a0 + np.pi / 2, 24, endpoint=True):
            section.append((cx + r * np.cos(a), cy + r * np.sin(a)))
    return convex_prism(section, length_mm)


def build():
    items = [
        ("bl_box_b", trimesh.creation.box(extents=(445.0, 315.0, 315.0))),
        ("bl_box_c", trimesh.creation.box(extents=(460.0, 330.0, 330.0))),
        # sections=5 -> regular pentagonal prism; R chosen so the section
        # (110 mm circumcircle) sits well inside the max limits
        ("bl_pent_d", trimesh.creation.cylinder(radius=55.0, height=260.0, sections=5)),
        ("bl_rsq_b", rounded_square_prism(side_mm=100.0, corner_r_mm=16.0,
                                          length_mm=220.0)),
        ("bl_rod_b", trimesh.creation.box(extents=(220.0, 16.0, 12.0))),
    ]
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    manifest = []
    for slug, mesh_mm in items:
        verdict = classify_mesh(mesh_mm)               # official rules, in mm
        mesh, extents_mm = obb_flat_transform(mesh_mm)
        mesh.apply_scale(0.001)
        mesh.apply_translation(-mesh.bounding_box.centroid)
        out = OUT_DIR / f"{slug}.stl"
        mesh.export(out)
        dims_m = (extents_mm / 1000.0).round(4).tolist()
        volume = float(mesh.convex_hull.volume)
        mass = float(np.clip(volume * DENSITY, MASS_MIN, MASS_MAX))
        entry = {
            "slug": slug,
            "source": f"synthetic:{slug}",
            "file": out.name,
            "dims_m": dims_m,
            "half_z": dims_m[2] / 2.0,
            "mass_kg": round(mass, 3),
            "zone": verdict["zone"],
            "category": verdict["category"],
            "max_ratio": round(verdict.get("max_ratio", 0.0), 4),
            "faces": int(len(mesh.faces)),
        }
        manifest.append(entry)
        print(f"{slug:<10} dims={dims_m} ratio={entry['max_ratio']} "
              f"zone={entry['zone']} mass={entry['mass_kg']}kg")
    MANIFEST.write_text(json.dumps(manifest, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(f"\n{len(manifest)} synthetic borderline items -> {MANIFEST}")


if __name__ == "__main__":
    build()
