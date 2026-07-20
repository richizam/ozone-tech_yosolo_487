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


def settle_item(model, data, slug, entry, y_off, yaw, steps=200,
                max_retries=6):
    """Teleport one item under the camera and let physics settle it.

    A settled pose must be BELT-REALISTIC: freight on a 1 m/s conveyor
    cannot rest leaning against the measuring-station furniture (drag
    topples it instantly), but a static drop CAN produce such poses — a
    490 mm rod once settled propped against the gantry, reading 424 mm
    tall and certifying as 'fits' while its true OBB is oversize. If the
    settled item is in contact with anything but the belt (or another
    item), re-drop with a nudged pose; if it still leans after the
    retries, fall back to an axis-aligned flat drop."""
    jid = model.joint(f"fj_{slug}").id
    bid = model.joint(f"fj_{slug}").bodyid[0]
    geoms = {g for g in range(model.ngeom) if model.geom_bodyid[g] == bid}
    belt = {g for g in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "")
            .startswith("beltA")}
    # bodies of OTHER freight items (free joints fj_*): resting against
    # another parcel is belt-realistic; everything else — blades, gantry,
    # guards, any actuated machine body — is furniture
    freight_bodies = set()
    for j in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        if nm.startswith("fj_"):
            freight_bodies.add(int(model.jnt_bodyid[j]))
    qa, da = model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    # small freight is measured by the close-range macro head; the harness
    # reads it at the macro station, where the moving flow's captures land
    # (in run_sim the item passes under the head and reads accumulate there)
    cam_x = (P.VIRTUAL_SENSOR["macro_pos"][0]
             if max(entry["dims_m"]) < 2.0 * P.VIRTUAL_SENSOR["macro_engage_mm"]
             / 1000.0 else CAM_X)

    def drop(y_o, yw):
        # spawn clearance from the LONGEST extent, not dims_m[2]: a mesh
        # whose local frame is not OBB-aligned (the campaign generator
        # exports some solids diagonally) otherwise spawns EMBEDDED in the
        # belt, gets ejected, and the camera snaps it mid-tumble — a
        # 490 mm rod once measured 424x310x40 while flying
        data.qpos[qa:qa + 3] = [cam_x, P.BELT_A["y"] + y_o,
                                P.BELT_A["top"] + max(entry["dims_m"]) / 2
                                + 0.004]
        data.qpos[qa + 3:qa + 7] = [np.cos(yw / 2), 0, 0, np.sin(yw / 2)]
        data.qvel[da:da + 6] = 0
        mujoco.mj_forward(model, data)
        # settle to REST, not for a fixed step count: tall spawns need the
        # fall + any rolling to finish before the camera reads
        for k in range(max(steps, 2500)):
            mujoco.mj_step(model, data)
            if k % 50 == 0 and k > 100:
                v = data.qvel[da:da + 6]
                if float(np.linalg.norm(v[:3])) < 1e-3 \
                        and float(np.linalg.norm(v[3:])) < 1e-2:
                    break

    def leans_on_statics():
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in geoms or g2 in geoms:
                other = g2 if g1 in geoms else g1
                if other in belt \
                        or int(model.geom_bodyid[other]) in freight_bodies:
                    continue                    # belt or another parcel
                return True
        return False

    drop(y_off, yaw)
    tries = 0
    while leans_on_statics() and tries < max_retries:
        tries += 1
        drop(y_off * 0.5, yaw + 0.9 * tries)
    if leans_on_statics():
        drop(0.0, 0.0)                          # axis-aligned flat fallback


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
