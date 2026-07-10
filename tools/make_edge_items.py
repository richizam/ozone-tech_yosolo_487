# -*- coding: utf-8 -*-
"""Synthetic EDGE-CASE items for the tilt-tray rebuild validation.

These prove the headline property of the cross-belt/tilt-tray executive:
size-independent divert. A roller/ARB deck has a practical ~50-75 mm minimum
product footprint; the tilt tray carries an 11 mm cube exactly like a 489 mm
pouf. The set attacks the undersize rule boundary:

    edge_cube11  11x11x11 mm cube  -> B  (above the 10 mm minimum: MUST be
                                          carried and delivered to the main
                                          sorter — the rebuild's proof item)
    edge_cube10  10x10x10 mm cube  -> C  (exactly at the limit: does NOT pass
                                          the strictly-greater gate)
    edge_rod9    120x9x9 mm rod    -> C  (undersize -> C)
    edge_card2   150x100x2 mm card -> C  (very thin flat item: thin-item
                                          measurement + tray handling)

Ground truth is COMPUTED by the reference classifier (tools/classify_mesh.py).
Meshes land in cell/assets/meshes/, entries in cell/assets/manifest_edge.json
(merged by isaac.scene_usd.load_manifest(extra=True) / --manifest-extra).

    .venv/Scripts/python tools/make_edge_items.py
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
MANIFEST = ROOT / "cell" / "assets" / "manifest_edge.json"


def build():
    items = [
        ("edge_cube11", trimesh.creation.box(extents=(11.0, 11.0, 11.0))),
        ("edge_cube10", trimesh.creation.box(extents=(10.0, 10.0, 10.0))),
        ("edge_rod9", trimesh.creation.box(extents=(120.0, 9.0, 9.0))),
        ("edge_card2", trimesh.creation.box(extents=(150.0, 100.0, 2.0))),
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
        # small parcels: keep the physical mass (a clip to 50 g on an 11 mm
        # cube would be a lead die); floor at 5 g for solver health
        mass = float(np.clip(volume * DENSITY, 0.005, MASS_MAX))
        entry = {
            "slug": slug,
            "source": f"synthetic:{slug}",
            "file": out.name,
            "dims_m": dims_m,
            "half_z": dims_m[2] / 2.0,
            "mass_kg": round(mass, 4),
            "zone": verdict["zone"],
            "category": verdict["category"],
            "max_ratio": round(verdict.get("max_ratio", 0.0), 4),
            "faces": int(len(mesh.faces)),
        }
        manifest.append(entry)
        print(f"{slug:<12} dims={dims_m} ratio={entry['max_ratio']} "
              f"zone={entry['zone']} mass={entry['mass_kg']}kg  ({verdict['reason']})")
    MANIFEST.write_text(json.dumps(manifest, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(f"\n{len(manifest)} synthetic edge items -> {MANIFEST}")


if __name__ == "__main__":
    build()
