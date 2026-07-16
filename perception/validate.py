# -*- coding: utf-8 -*-
"""Perception validation campaign: every official item x K randomized poses,
classified from CAMERA DATA ONLY, compared against the mesh ground truth.

    python -m perception.validate [--poses 8] [--seed 5]

Writes docs/metrics/perception_validation.csv + .json.

Exit code 0 iff BOTH hold:
  * official-set category accuracy >= 0.95 (per item >= 0.75), and
  * ZERO permissive errors — nothing the rules exclude may be perceived as
    sorter-bound (B). This is the safety property the whole cell rests on.
The borderline bl_* attack items sit within 0.02 of a rule threshold by
construction, so a conservative verdict on them is the designed outcome and
is reported separately rather than scored as a miss.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402

from cell import params as P  # noqa: E402
from cell.scene import make_model  # noqa: E402
from perception.pipeline import LookaheadPerception  # noqa: E402

CAM_X = 6.0


def settle_item(model, data, slug, entry, y_off, yaw, steps=200):
    """Teleport one item under the camera and let physics settle it."""
    jid = model.joint(f"fj_{slug}").id
    qa, da = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
    data.qpos[qa:qa + 3] = [CAM_X, P.BELT_A["y"] + y_off, P.BELT_A["top"] + entry["dims_m"][2] / 2 + 0.004]
    data.qpos[qa + 3:qa + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    data.qvel[da:da + 6] = 0
    mujoco.mj_forward(model, data)
    for _ in range(steps):
        mujoco.mj_step(model, data)


def park_item(model, data, slug, idx, entry):
    jid = model.joint(f"fj_{slug}").id
    qa, da = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
    data.qpos[qa:qa + 3] = [0.6 + idx * 0.85, -1.2, entry["dims_m"][2] / 2 + 0.001]
    data.qpos[qa + 3:qa + 7] = [1, 0, 0, 0]
    data.qvel[da:da + 6] = 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=int, default=8)
    ap.add_argument("--seed", type=int, default=5)
    args = ap.parse_args(argv)

    rng = np.random.default_rng(args.seed)
    model, manifest, _ = make_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    percep = LookaheadPerception(model)

    rows = []
    for idx, e in enumerate(manifest):
        slug = e["slug"]
        diag = float(np.hypot(e["dims_m"][0], e["dims_m"][1]))
        for k in range(args.poses):
            # big items arrive pre-aligned by the upstream infeed (see report);
            # everything else may arrive at any yaw
            yaw = float(rng.uniform(-0.09, 0.09)) if diag > 0.48 else float(rng.uniform(0, 2 * np.pi))
            y_off = float(rng.uniform(-0.05, 0.05))
            settle_item(model, data, slug, e, y_off, yaw)
            res = percep.classify(data)
            true_dims = np.sort(np.array(e["dims_m"]) * 1000)[::-1]
            if res is None:
                rows.append({"slug": slug, "pose": k, "ok": False, "zone_true": e["zone"],
                             "zone_perceived": "NONE", "dims_err_mm": None})
            else:
                derr = float(np.max(np.abs(np.array(res["dims_mm"]) - true_dims)))
                rows.append({"slug": slug, "pose": k, "ok": res["zone"] == e["zone"],
                             "zone_true": e["zone"], "zone_perceived": res["zone"],
                             "max_ratio": res["max_ratio"], "dims_err_mm": round(derr, 1),
                             "confidence": res["confidence"], "flags": ";".join(res["flags"])})
            park_item(model, data, slug, idx, e)
        acc_item = np.mean([r["ok"] for r in rows if r["slug"] == slug])
        last = [r for r in rows if r["slug"] == slug]
        errs = [r["dims_err_mm"] for r in last if r.get("dims_err_mm") is not None]
        print(f"{slug:<10} acc={acc_item:.2f}  dims_err_max={max(errs) if errs else '—'}mm  "
              f"perceived={[r['zone_perceived'] for r in last]}")

    out_dir = ROOT / "docs" / "metrics"
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "perception_validation.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["slug", "pose", "ok", "zone_true", "zone_perceived",
                                          "max_ratio", "dims_err_mm", "confidence", "flags"],
                           extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    # OFFICIAL set vs BORDERLINE attack set (bl_*): the borderline items are
    # built to sit within 0.02 of a rule threshold, so a conservative verdict
    # on them is the DESIGNED outcome, not a miss (bl_rsq_b: true r/R 0.78,
    # measured 0.803 -> D). Scoring them as errors would punish the cell for
    # doing exactly what the report promises.
    official = [r for r in rows if not r["slug"].startswith("bl_")]
    accuracy = float(np.mean([r["ok"] for r in official]))
    errs = np.array([r["dims_err_mm"] for r in rows if r.get("dims_err_mm") is not None])
    # the safety property: freight the rules exclude must never be perceived
    # as sorter-bound. This is the gate that actually matters.
    permissive = [r for r in rows
                  if r["zone_perceived"] == "B" and r["zone_true"] != "B"]
    conservative = [r for r in rows
                    if not r["ok"] and r["zone_perceived"] != "B"]
    summary = {
        "poses_per_item": args.poses, "seed": args.seed,
        "classifications": len(rows),
        "official_items": len({r["slug"] for r in official}),
        "category_accuracy_official": round(accuracy, 4),
        "permissive_errors": len(permissive),
        "conservative_deviations": len(conservative),
        "dims_err_mean_mm": round(float(errs.mean()), 2) if len(errs) else None,
        "dims_err_p95_mm": round(float(np.percentile(errs, 95)), 2) if len(errs) else None,
        "dims_err_max_mm": round(float(errs.max()), 2) if len(errs) else None,
        "permissive_detail": [
            {k: r.get(k) for k in ("slug", "pose", "zone_true",
                                   "zone_perceived", "max_ratio")}
            for r in permissive],
        "conservative_detail": [
            {k: r.get(k) for k in ("slug", "pose", "zone_true",
                                   "zone_perceived", "max_ratio")}
            for r in conservative],
    }
    per_item = {}
    for r in official:
        per_item.setdefault(r["slug"], []).append(r["ok"])
    min_item_acc = min(np.mean(v) for v in per_item.values()) if per_item else 0.0
    summary["min_per_item_accuracy_official"] = round(float(min_item_acc), 3)
    (out_dir / "perception_validation.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    # gate: (1) the official set must classify; (2) NOTHING the rules exclude
    # may be perceived as B — a hard zero, on both the official and the
    # borderline attack set
    gate = (accuracy >= 0.95 and min_item_acc >= 0.75
            and len(permissive) == 0)
    if permissive:
        print(f"GATE FAIL: {len(permissive)} permissive error(s) — freight the "
              f"rules exclude was perceived as sorter-bound")
    print("EXIT GATE:", "PASS" if gate else "FAIL")
    return 0 if gate else 1


if __name__ == "__main__":
    raise SystemExit(main())
