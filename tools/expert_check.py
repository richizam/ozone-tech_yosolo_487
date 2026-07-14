# -*- coding: utf-8 -*-
"""EXPERT ENTRY POINT — bring-your-own-STL check (Track 3 requirement:
experts must be able to load their own test models and see how the system
classifies them and hands the result to the executive).

Two levels:

1) CLASSIFY ONLY (seconds, CPU): apply the official Track-3 rules
   (dimension gate 10..450x320x320 mm, then circle-in-section r/R >= 0.8)
   to any STL and print category + full reasoning:

       python tools/expert_check.py path/to/model.stl

2) REGISTER FOR SIMULATION (--register): convert the mesh, add it to
   cell/assets/manifest_custom.json (auto-merged by BOTH simulators when
   present; absent by default so validated runs are untouched) and print
   the exact commands that push the item through the PHYSICAL loop:

       python tools/expert_check.py path/to/model.stl --register --mass 0.4
       # CPU digital twin (no GPU needed):
       .venv/Scripts/python -m cell.run_sim --scenario scenarios/base.yaml --seed 42
       # Isaac Sim (RTX perception in the loop):
       /isaac-sim/python.sh isaac/run_isaac.py --items custom_<name> --seed 42

Units: official STLs are in millimetres. Meshes whose max extent exceeds
5.0 units are treated as mm and scaled to metres for the simulators
(override with --units mm|m). The reference category computed here is
stored as the item's ground truth; the simulators' own perception then
measures and classifies it independently — disagreement shows up in the
run summary, which is exactly what the check is for.
"""
import argparse
import json
import re
import shutil
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))

from classify_mesh import classify_mesh, CATEGORY_NAMES  # noqa: E402

import trimesh  # noqa: E402

ASSETS = REPO / "cell" / "assets"
MESHES = ASSETS / "meshes"
CUSTOM_MANIFEST = ASSETS / "manifest_custom.json"


def register(stl_path, mass, units):
    mesh = trimesh.load(str(stl_path), force="mesh")
    ext = np.asarray(mesh.bounding_box_oriented.primitive.extents, float)
    if units == "auto":
        units = "mm" if float(ext.max()) > 5.0 else "m"
    scale = 0.001 if units == "mm" else 1.0

    # classify in mm (the official rule space)
    mm_mesh = mesh.copy()
    if units == "m":
        mm_mesh.apply_scale(1000.0)
    verdict = classify_mesh(mm_mesh)

    slug = "custom_" + re.sub(r"[^a-z0-9]+", "_",
                              Path(stl_path).stem.lower()).strip("_")
    out_mesh = MESHES / f"{slug}.stl"
    m_mesh = mesh.copy()
    if scale != 1.0:
        m_mesh.apply_scale(scale)
    m_mesh.export(str(out_mesh))

    dims_m = sorted((np.asarray(
        m_mesh.bounding_box_oriented.primitive.extents, float)).tolist(),
        reverse=True)
    if mass is None:
        vol = abs(float(m_mesh.volume)) if m_mesh.is_watertight else \
            float(np.prod(dims_m)) * 0.4
        mass = float(np.clip(vol * 400.0, 0.05, 3.0))   # ~packing density

    entry = {
        "slug": slug,
        "source": Path(stl_path).name,
        "file": out_mesh.name,
        "dims_m": [round(d, 4) for d in dims_m],
        "half_z": round(min(dims_m) / 2, 4),
        "mass_kg": round(mass, 3),
        "zone": verdict["zone"],
        "category": verdict["category"],
        "faces": int(len(m_mesh.faces)),
    }
    entries = []
    if CUSTOM_MANIFEST.exists():
        entries = json.loads(CUSTOM_MANIFEST.read_text(encoding="utf-8"))
        entries = [e for e in entries if e["slug"] != slug]
    entries.append(entry)
    CUSTOM_MANIFEST.write_text(
        json.dumps(entries, ensure_ascii=False, indent=1), encoding="utf-8")
    return slug, entry, verdict


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl", help="path to the expert's STL model")
    ap.add_argument("--register", action="store_true",
                    help="also register the item for the simulators")
    ap.add_argument("--mass", type=float, default=None,
                    help="item mass in kg (default: volume-based estimate)")
    ap.add_argument("--units", choices=("auto", "mm", "m"), default="auto")
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    stl = Path(args.stl)
    if not stl.is_file():
        sys.exit(f"no such file: {stl}")

    if not args.register:
        mesh = trimesh.load(str(stl), force="mesh")
        ext = np.asarray(mesh.bounding_box_oriented.primitive.extents, float)
        if args.units == "m" or (args.units == "auto"
                                 and float(ext.max()) <= 5.0):
            mesh.apply_scale(1000.0)
        v = classify_mesh(mesh)
        print(json.dumps({"file": stl.name, **v}, ensure_ascii=False,
                         indent=1))
        return

    slug, entry, verdict = register(stl, args.mass, args.units)
    print(json.dumps({"registered": slug, "entry": entry,
                      "official_rule_verdict": verdict},
                     ensure_ascii=False, indent=1))
    print()
    print("Run it through the PHYSICAL loop:")
    print("  # CPU digital twin (no GPU): the custom item joins the flow")
    print("  .venv/Scripts/python -m cell.run_sim "
          "--scenario scenarios/base.yaml --seed 42")
    print("  # Isaac Sim (RTX perception in the loop), only this item:")
    print(f"  /isaac-sim/python.sh isaac/run_isaac.py --items {slug} "
          "--seed 42 --record")
    print()
    print("The run summary (summary.json) reports the perceived dims, the")
    print("category the LIVE perception assigned, the zone it was")
    print("physically delivered to, and containment — compare against the")
    print("official_rule_verdict above.")


if __name__ == "__main__":
    main()
