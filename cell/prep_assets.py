# -*- coding: utf-8 -*-
"""Prepare simulation assets from the official STL test set.

For each item: load STL (mm) -> rotate into its OBB frame with extents sorted
so the item lies flat and long-side-along-belt -> scale to meters -> export
the ORIGINAL surface. MuJoCo computes convex hulls internally for contacts,
but rays (our depth sensor) intersect the true triangles — hull proxies would
change the shape class of borderline items (the «Цилиндр» trap loses its
0.867-ratio section when hulled!). Mass is estimated from the hull volume.

Outputs:
  cell/assets/meshes/<slug>.stl       true mesh, meters, OBB-aligned, centered
  cell/assets/manifest.json           dims, mass estimate, ground-truth zone
"""
import json
import sys
import unicodedata
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

STL_DIR = ROOT / "extracted" / "doc-1782987733" / "Stl"
GT_JSON = ROOT / "docs" / "ground_truth" / "item_ground_truth.json"
OUT_DIR = Path(__file__).parent / "assets" / "meshes"
MANIFEST = Path(__file__).parent / "assets" / "manifest.json"

SLUGS = {  # ascii slugs for mujoco names / filenames
    "Бутылка": "bottle", "Короб 300х200х200": "box_s", "Короб 400х400х300": "box_l",
    "ЛанчБокс": "lunchbox", "Мешок": "sack", "Моющее средство": "detergent",
    "Пуфик": "pouf", "Ручка": "pen", "Тарелка": "plate", "Цилиндр": "cylinder",
    "Шлем": "helmet",
}
DENSITY = 250.0          # kg/m^3 — light e-commerce goods assumption
MASS_MIN, MASS_MAX = 0.05, 6.0


def obb_flat_transform(mesh):
    """Transform into OBB frame, axes permuted so extents are (x >= y >= z)."""
    to_obb, extents = trimesh.bounds.oriented_bounds(mesh)
    m = mesh.copy()
    m.apply_transform(to_obb)
    order = np.argsort(extents)[::-1]          # target x=largest ... z=smallest
    axes = np.eye(3)[:, order]
    if np.linalg.det(axes) < 0:                # keep it a rotation
        axes[:, 2] = -axes[:, 2]
    rot = np.eye(4)
    rot[:3, :3] = axes.T
    m.apply_transform(rot)
    return m, np.asarray(extents)[order]


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    gt = {e["file"]: e for e in json.loads(GT_JSON.read_text(encoding="utf-8"))}
    manifest = []
    for stl in sorted(STL_DIR.iterdir()):
        if stl.suffix.lower() != ".stl":
            continue
        stem = unicodedata.normalize("NFC", stl.stem)
        slug = SLUGS.get(stem)
        if slug is None:
            print(f"skip unknown item: {stl.name}")
            continue
        mesh = trimesh.load(stl, force="mesh")
        mesh, extents_mm = obb_flat_transform(mesh)
        mesh.apply_scale(0.001)                          # mm -> m
        mesh.apply_translation(-mesh.bounding_box.centroid)
        out = OUT_DIR / f"{slug}.stl"
        mesh.export(out)
        dims_m = (extents_mm / 1000.0).round(4).tolist()
        volume = float(mesh.convex_hull.volume)          # open meshes: hull volume
        mass = float(np.clip(volume * DENSITY, MASS_MIN, MASS_MAX))
        entry = {
            "slug": slug,
            "source": stl.name,
            "file": out.name,
            "dims_m": dims_m,                            # x >= y >= z after alignment
            "half_z": dims_m[2] / 2.0,
            "mass_kg": round(mass, 3),
            "zone": gt[stl.name]["zone"],
            "category": gt[stl.name]["category"],
            "faces": int(len(mesh.faces)),
        }
        manifest.append(entry)
        print(f"{slug:<10} dims={dims_m} mass={entry['mass_kg']}kg zone={entry['zone']} faces={entry['faces']}")
    MANIFEST.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n{len(manifest)} items -> {MANIFEST}")


if __name__ == "__main__":
    main()
