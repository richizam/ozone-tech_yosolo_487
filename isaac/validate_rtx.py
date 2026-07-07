# -*- coding: utf-8 -*-
"""Calibrate + validate the RTX depth-camera classifier on the 11 official
items, using REAL rendered depth (not ground truth).

For each item: drop it onto a belt-height surface under the overhead RTX
camera, let it settle into a stable rest pose, render the depth frame, run
isaac/perception_rtx.py, and compare the depth-derived verdict to
docs/ground_truth/item_ground_truth.json.

Run inside the container:
  /isaac-sim/python.sh isaac/validate_rtx.py --out /tmp/sortmaster_out/rtxval
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from cell import params as P                      # noqa: E402


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/sortmaster_out/rtxval")
    ap.add_argument("--settle-steps", type=int, default=150)
    ap.add_argument("--yaws", default="0",
                    help="comma list of extra yaw angles (deg) per item")
    ap.add_argument("--dome-tau", type=float, default=0.75)
    ap.add_argument("--save-depth", action="store_true")
    return ap.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        from isaacsim import SimulationApp
    except ImportError:
        from isaacsim.simulation_app import SimulationApp
    sim_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

    from isaacsim.core.api import World
    try:
        from isaacsim.core.prims import SingleRigidPrim
    except ImportError:
        from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
    from isaacsim.sensors.camera import Camera
    from pxr import UsdGeom, UsdLux, Gf

    from isaac.scene_usd import SceneBuilder, load_manifest
    from isaac.perception_rtx import RTXPerception

    manifest = load_manifest(REPO)
    # ground-truth zone per slug (same values as docs/ground_truth JSON)
    slug_zone = {e["slug"]: e["zone"] for e in manifest}

    world = World(physics_dt=1.0 / 240.0, rendering_dt=1.0 / 240.0,
                  stage_units_in_meters=1.0, backend="numpy")
    stage = world.stage
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    builder = SceneBuilder(stage, REPO)
    # minimal rig: floor + a belt-height pad under the camera + overhead camera
    m_belt = builder.phys_material("belt", 0.80, 0.72, combine="average")
    m_item = builder.phys_material("item", 0.90, 0.85, combine="average")
    builder.add_box("floor", (6.0, 3.0, -0.01), (2.0, 1.5, 0.01),
                    color=(0.85, 0.85, 0.88), mat=m_belt)
    builder.add_box("pad", (6.0, 3.0, P.BELT_A["top"] / 2),
                    (0.9, P.BELT_A["width"] / 2, P.BELT_A["top"] / 2),
                    color=(0.35, 0.42, 0.55), mat=m_belt)
    dome = UsdLux.DomeLight.Define(stage, "/World/lights/dome")
    dome.CreateIntensityAttr(500.0)
    cam_path = builder.build_camera("lookahead", (6.0, 3.0, 2.2),
                                    (1, 0, 0), (0, 1, 0))
    # side profiler heads per VIRTUAL_SENSOR (cell/params.py): the section
    # below its widest line is invisible from overhead — the side heads close
    # the hull so hex-vs-box is decidable at DWS sampling density
    vs = P.VIRTUAL_SENSOR
    off, zh = vs["side_head_offset_m"], vs["side_head_z_m"]
    side_paths = [
        builder.build_camera_lookat("side_a", (6.0, P.BELT_A["y"] - off, zh),
                                    (6.0, P.BELT_A["y"], P.BELT_A["top"] + 0.10)),
        builder.build_camera_lookat("side_b", (6.0, P.BELT_A["y"] + off, zh),
                                    (6.0, P.BELT_A["y"], P.BELT_A["top"] + 0.10)),
    ]
    item_info = {}
    for i, e in enumerate(manifest):
        path, park = builder.build_item(e, i, m_item)
        item_info[e["slug"]] = path

    world.reset()

    rps = {}
    for slug, path in item_info.items():
        rp = SingleRigidPrim(path, name=f"rp_{slug}")
        if hasattr(rp, "initialize"):
            rp.initialize()
        rps[slug] = rp

    cam = Camera(prim_path=cam_path, resolution=(1024, 768))
    cam.initialize()
    cam.add_distance_to_image_plane_to_frame()
    side_cams = []
    for sp in side_paths:
        sc = Camera(prim_path=sp, resolution=(768, 576))
        sc.initialize()
        sc.add_distance_to_image_plane_to_frame()
        side_cams.append(sc)
    perc = RTXPerception([cam] + side_cams, dome_tau=args.dome_tau)

    for _ in range(20):
        world.step(render=True)

    yaws = [float(v) for v in args.yaws.split(",")]
    rows = []
    for e in manifest:
        slug = e["slug"]
        gt_zone = slug_zone[slug]
        for yaw in yaws:
            # park everyone far away, then place this one above the pad
            for s2, rp2 in rps.items():
                rp2.set_world_pose(np.array([0.6 + list(rps).index(s2) * 0.9, -1.5,
                                             0.2]), np.array([1.0, 0, 0, 0]))
                rp2.set_linear_velocity(np.zeros(3))
                rp2.set_angular_velocity(np.zeros(3))
            for _ in range(5):
                world.step(render=False)
            qz = (math.cos(math.radians(yaw) / 2), 0, 0, math.sin(math.radians(yaw) / 2))
            rps[slug].set_world_pose(
                np.array([6.0, P.BELT_A["y"], P.BELT_A["top"] + e["dims_m"][2] / 2 + 0.02]),
                np.array(qz))
            rps[slug].set_linear_velocity(np.zeros(3))
            rps[slug].set_angular_velocity(np.zeros(3))
            for _ in range(args.settle_steps):
                world.step(render=False)
            # flush the RTX annotator with the CURRENT scene state: the
            # replicator pipeline lags 1-2 rendered frames behind, and the
            # settle loop above rendered nothing (stale-frame bug found in
            # calibration: every verdict was the PREVIOUS item's geometry)
            for _ in range(6):
                world.step(render=True)
            res = perc.measure(debug=True)
            if args.save_depth:
                fr = cam.get_current_frame().get("distance_to_image_plane")
                if fr is not None:
                    np.save(out_dir / f"depth_{slug}_{int(yaw)}.npy",
                            np.asarray(fr, dtype=np.float32))
            if res is None:
                rows.append({"slug": slug, "yaw": yaw, "gt": gt_zone,
                             "pred": "MISS", "ok": False})
                print(f"{slug:10s} yaw={yaw:5.0f} gt={gt_zone} -> SENSOR MISS", flush=True)
                continue
            ok = res["zone"] == gt_zone
            rows.append({"slug": slug, "yaw": yaw, "gt": gt_zone,
                         "pred": res["zone"], "ok": ok, **res})
            print(f"{slug:10s} yaw={yaw:5.0f} gt={gt_zone} pred={res['zone']} "
                  f"{'OK ' if ok else 'XX '} dims={res['dims_mm']} "
                  f"circ={res['footprint_circularity']:.2f} "
                  f"dome={res['dome_score']:.2f} sect={res['section_ratio']:.2f} "
                  f"n={res['n_points']} [{res['reason']}]", flush=True)

    n_ok = sum(1 for r in rows if r["ok"])
    summary = {"dome_tau": args.dome_tau, "n": len(rows), "n_ok": n_ok,
               "accuracy": round(n_ok / max(1, len(rows)), 4), "rows": rows}
    (out_dir / "rtx_validation.json").write_text(json.dumps(summary, indent=2),
                                                 encoding="utf-8")
    print(f"\nRTX perception accuracy: {n_ok}/{len(rows)} = "
          f"{100 * n_ok / max(1, len(rows)):.1f}%  (dome_tau={args.dome_tau})",
          flush=True)
    # feature separation table to pick dome_tau
    print("\n-- dome_score by class (for threshold tuning) --", flush=True)
    for zc in ("B", "C", "D"):
        ds = [r["dome_score"] for r in rows if r.get("gt") == zc and "dome_score" in r]
        cs = [r["footprint_circularity"] for r in rows
              if r.get("gt") == zc and "footprint_circularity" in r]
        if ds:
            print(f"  {zc}: dome[min={min(ds):.2f} max={max(ds):.2f}] "
                  f"circ[min={min(cs):.2f} max={max(cs):.2f}]", flush=True)
    sim_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
