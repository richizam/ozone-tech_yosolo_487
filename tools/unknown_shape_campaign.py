# -*- coding: utf-8 -*-
"""Unknown-shape robustness campaign — the private-set question, answered
with numbers instead of architecture talk.

The official set is 11 items, and the shape-evidence thresholds were fitted
against them. The organizers may test with their own models, so this
campaign generates procedurally random solids the cell has NEVER seen
(rounded boxes, cylinders/cones/capsules at any pose, hex/oct prisms, domed
tubs, random convex hulls), sized across every rule threshold, and asks:

  reference verdict  = tools/classify_mesh.py on the MESH   (the official
                       rules, computed exactly — this is ground truth)
  measured verdict   = the twin's ray-cast perception on the SETTLED item
                       (what the cell actually decides in the loop)

Agreement is reported, and every disagreement is classified by DIRECTION:
  safe        — measured is more conservative than the rules (B -> C/D/REVIEW)
  UNSAFE      — measured is more permissive (C/D -> B): freight that the
                rules exclude would reach the main sorter. Must be zero.

Output: docs/report/unknown_shape_campaign.{md,json}
Usage:  python tools/unknown_shape_campaign.py [--n 24] [--seed 11]
"""
import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from tools.classify_mesh import classify_mesh                   # noqa: E402

ASSETS = REPO / "cell" / "assets"
MESHES = ASSETS / "meshes"
CUSTOM = ASSETS / "manifest_custom.json"
PERMISSIVE = "B"          # the only zone freight must never reach by error


def _rounded_box(rng, w, d, h):
    m = trimesh.creation.box(extents=(w, d, h))
    if rng.random() < 0.4:                       # bevelled variant
        m = trimesh.util.concatenate(
            [m, trimesh.creation.icosphere(radius=min(w, d, h) * 0.55)]
        ).convex_hull
    return m


MAX_INBOUND_M = 0.5      # official max item on conveyor A (500x500x500)


def gen_shape(rng, i):
    """A solid the official set does not contain, sized across thresholds.

    Sizes stay inside the official 500x500x500 inbound envelope: freight
    larger than that cannot physically arrive on conveyor A, so measuring it
    would test a case the line never sees (and the measuring volume, capped
    at the same envelope like a real dimensioner's specified volume, would
    legitimately truncate it)."""
    kind = rng.choice(["rbox", "cyl", "cone", "capsule", "hex", "oct",
                       "dome_tub", "hull", "thin_plate", "rod"])
    # sizes deliberately straddle 10 mm / 450x320x320 mm and the r/R = 0.8 line
    s = float(rng.choice([0.008, 0.012, 0.05, 0.12, 0.25, 0.34, 0.44, 0.48]))
    if kind == "rbox":
        m = _rounded_box(rng, s, s * rng.uniform(0.4, 1.0),
                         s * rng.uniform(0.3, 0.9))
    elif kind == "cyl":
        m = trimesh.creation.cylinder(radius=s / 2, height=s * rng.uniform(0.5, 2.0))
    elif kind == "cone":
        m = trimesh.creation.cone(radius=s / 2, height=s * rng.uniform(0.8, 1.8))
    elif kind == "capsule":
        m = trimesh.creation.capsule(radius=s / 2.5, height=s)
    elif kind in ("hex", "oct"):
        n = 6 if kind == "hex" else 8
        poly = trimesh.creation.annulus(r_min=0.0, r_max=s / 2,
                                        height=s * rng.uniform(0.4, 1.2),
                                        sections=n)
        m = poly
    elif kind == "dome_tub":                       # domed cap on a prism body
        body = trimesh.creation.box(extents=(s, s * 0.8, s * 0.5))
        cap = trimesh.creation.icosphere(radius=s * 0.42)
        cap.apply_translation((0, 0, s * 0.25))
        m = trimesh.util.concatenate([body, cap]).convex_hull
    elif kind == "hull":                           # irregular convex blob
        pts = rng.normal(0, s / 3, (rng.integers(12, 40), 3))
        pts[:, 2] *= rng.uniform(0.3, 1.0)
        m = trimesh.points.PointCloud(pts).convex_hull
    elif kind == "thin_plate":
        m = trimesh.creation.box(extents=(s, s * rng.uniform(0.5, 1.0),
                                          rng.uniform(0.002, 0.02)))
    else:                                          # rod
        m = trimesh.creation.cylinder(radius=rng.uniform(0.004, 0.03),
                                      height=s * rng.uniform(1.0, 2.5))
    m.apply_transform(trimesh.transformations.random_rotation_matrix(
        rng.random(3)))
    m.apply_translation(-m.centroid)
    # clamp into the official inbound envelope (see docstring)
    ext = np.asarray(m.bounding_box_oriented.primitive.extents, float)
    if ext.max() > MAX_INBOUND_M:
        m.apply_scale(MAX_INBOUND_M / float(ext.max()) * 0.98)
    return kind, m


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--poses", type=int, default=2)
    args = ap.parse_args(argv)
    rng = np.random.default_rng(args.seed)

    prev = CUSTOM.read_text(encoding="utf-8") if CUSTOM.exists() else None
    entries, refs = [], {}
    for i in range(args.n):
        kind, m = gen_shape(rng, i)
        slug = f"unk_{i:02d}_{kind}"
        mm = m.copy()
        mm.apply_scale(1000.0)
        verdict = classify_mesh(mm)                # OFFICIAL RULES = ground truth
        MESHES.mkdir(parents=True, exist_ok=True)
        m.export(str(MESHES / f"{slug}.stl"))
        dims = sorted(np.asarray(m.bounding_box_oriented.primitive.extents,
                                 float).tolist(), reverse=True)
        vol = abs(float(m.volume)) if m.is_watertight else float(np.prod(dims)) * 0.4
        entries.append({
            "slug": slug, "source": f"{slug}.stl", "file": f"{slug}.stl",
            "dims_m": [round(d, 4) for d in dims],
            "half_z": round(min(dims) / 2, 4),
            "mass_kg": round(float(np.clip(vol * 400.0, 0.05, 3.0)), 3),
            "zone": verdict["zone"], "category": verdict["category"],
            "faces": int(len(m.faces)),
        })
        refs[slug] = verdict
        print(f"  {slug:22s} dims={['%.0f' % (d*1000) for d in dims]} "
              f"-> rules: {verdict['zone']}")

    CUSTOM.write_text(json.dumps(entries, indent=2), encoding="utf-8")

    # ---- measured verdicts: the twin's ray-cast perception, item settled
    import mujoco                                              # noqa: E402
    from cell.scene import make_model                          # noqa: E402
    from perception.pipeline import LookaheadPerception         # noqa: E402
    from perception.validate import settle_item, park_item      # noqa: E402

    model, manifest, _ = make_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    percep = LookaheadPerception(model)

    rows = []
    for idx, e in enumerate(manifest):
        slug = e["slug"]
        if slug not in refs:
            continue
        for k in range(args.poses):
            # same presentation rule as perception/validate.py: freight whose
            # footprint diagonal approaches the belt width arrives aligned by
            # the upstream infeed (a 490 mm rod cannot lie at 45 deg across a
            # 500 mm belt), everything else at any yaw
            diag = float(np.hypot(e["dims_m"][0], e["dims_m"][1]))
            yaw = (float(rng.uniform(-0.09, 0.09)) if diag > 0.48
                   else float(rng.uniform(0, 2 * np.pi)))
            settle_item(model, data, slug, e, float(rng.uniform(-0.04, 0.04)),
                        yaw)
            res = percep.classify(data)
            ref = refs[slug]["zone"]
            got = (res or {}).get("zone", "NONE")
            # a sensor miss is a designed safe terminal (manual review), not
            # a permissive error
            safe = (got != PERMISSIVE) if got != ref else True
            rows.append({"slug": slug, "pose": k, "zone_rules": ref,
                         "zone_measured": got, "agree": got == ref,
                         "safe": bool(safe),
                         "dims_mm": (res or {}).get("dims_mm"),
                         "max_ratio": (res or {}).get("max_ratio")})
            park_item(model, data, slug, idx, e)

    if prev is None:
        CUSTOM.unlink(missing_ok=True)
    else:
        CUSTOM.write_text(prev, encoding="utf-8")
    for e in entries:                       # campaign meshes are disposable
        (MESHES / e["file"]).unlink(missing_ok=True)

    n = len(rows)
    agree = sum(r["agree"] for r in rows)
    unsafe = [r for r in rows if not r["safe"]]
    conservative = [r for r in rows if not r["agree"] and r["safe"]]
    summary = {
        "shapes": args.n, "poses_per_shape": args.poses, "seed": args.seed,
        "classifications": n,
        "agreement_with_official_rules": round(agree / n, 4) if n else None,
        "conservative_disagreements": len(conservative),
        "unsafe_disagreements": len(unsafe),
        "unsafe_detail": unsafe[:10],
    }
    out = REPO / "docs" / "report"
    (out / "unknown_shape_campaign.json").write_text(
        json.dumps({"summary": summary, "rows": rows}, indent=2),
        encoding="utf-8")

    md = [
        "# Unknown-shape robustness campaign",
        "",
        "> Generated by `tools/unknown_shape_campaign.py` — regenerate with",
        f"> `python tools/unknown_shape_campaign.py --n {args.n} "
        f"--seed {args.seed}`.",
        "",
        "**The question this answers:** the official set is 11 items and the",
        "auxiliary shape-evidence thresholds were fitted against them. If the",
        "jury tests with their own models, does the cell still behave?",
        "",
        "**Method.** Procedurally random solids the cell has never seen",
        "(rounded boxes, cylinders, cones, capsules, hex/oct prisms, domed",
        "tubs, random convex hulls, thin plates, rods), sized across every",
        "rule threshold, each settled on the belt at random yaw. Two verdicts",
        "per pose: the **official rules computed exactly from the mesh**",
        "(`tools/classify_mesh.py` — ground truth) versus **what the cell's",
        "ray-cast perception measures and decides** in the loop.",
        "",
        f"| Metric | Value |",
        "|---|---|",
        f"| Unseen shapes × poses | {args.n} × {args.poses} = {n} classifications |",
        f"| Agreement with the official rules | **{agree}/{n} = "
        f"{agree / n:.1%}** |" if n else "| Agreement | n/a |",
        f"| Disagreements, conservative direction (→ C/D/REVIEW) | "
        f"{len(conservative)} |",
        f"| Disagreements, permissive direction (→ B) | **{len(unsafe)}** |",
        "",
        "The number that matters is the last one: a permissive error means",
        "freight the rules exclude would reach the main sorter. Conservative",
        "disagreements are the designed behaviour — measurement uncertainty",
        "always resolves toward C/D/REVIEW.",
        "",
    ]
    if unsafe:
        md += ["## Permissive errors (must be empty)", ""]
        for r in unsafe[:10]:
            md.append(f"- `{r['slug']}` pose {r['pose']}: rules "
                      f"{r['zone_rules']} → measured {r['zone_measured']}, "
                      f"dims {r['dims_mm']}")
        md.append("")
    (out / "unknown_shape_campaign.md").write_text("\n".join(md) + "\n",
                                                   encoding="utf-8")
    print(f"\n{agree}/{n} agree with the official rules | "
          f"conservative: {len(conservative)} | UNSAFE: {len(unsafe)}")
    print(f"wrote {out / 'unknown_shape_campaign.md'}")
    return 1 if unsafe else 0


if __name__ == "__main__":
    raise SystemExit(main())
