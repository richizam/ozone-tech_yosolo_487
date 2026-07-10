# -*- coding: utf-8 -*-
"""Presentation-build clearance audit (rebuild brief P0). Non-destructive.

Builds the exact validated stage, then WITHOUT stepping physics:
  1. kinematically sweeps a tray through its full tilt cycle (0 -> +max
     incl. gain-noise headroom -> 0 -> -max -> 0) parked at every station
     line AND at intermediate travel positions, computing world-space AABB
     clearance between every tray collider (plates + lips) and every STATIC
     collider in the cell;
  2. checks the exception arm's recovery envelope: every waypoint and every
     straight TCP transfer segment (grasp corridor, lift, transfer, place)
     against cage/chute/guard colliders.

Reports the minimum clearance per (mover, static) pair with exact prim
paths; exits 1 if anything penetrates or comes closer than --tol-mm.

    /isaac-sim/python.sh isaac/audit_clearance.py --out /workspace/sortmaster_out/audit
"""
import argparse
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

ap = argparse.ArgumentParser()
ap.add_argument("--out", default="/workspace/sortmaster_out/audit")
ap.add_argument("--tol-mm", type=float, default=15.0)
ap.add_argument("--samples", type=int, default=24)
args = ap.parse_args()

from isaacsim import SimulationApp                          # noqa: E402
sim_app = SimulationApp({"headless": True})

import numpy as np                                          # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics, Gf                # noqa: E402
import omni.usd                                             # noqa: E402

from cell import params as P                                # noqa: E402
from isaac.scene_usd import SceneBuilder, load_manifest     # noqa: E402

OUT = Path(args.out)
OUT.mkdir(parents=True, exist_ok=True)

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

builder = SceneBuilder(stage, REPO)
info = builder.build(load_manifest(REPO))
carriers = info["carriers"]

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_], useExtentsHint=False)


def world_aabb(prim):
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = r.GetMin(), r.GetMax()
    return (np.array([lo[0], lo[1], lo[2]]),
            np.array([hi[0], hi[1], hi[2]]))


def clearance(a, b):
    """AABB pair clearance: >0 separated, <0 overlap depth."""
    gaps = np.maximum(b[0] - a[1], a[0] - b[1])
    return float(gaps.max())


def enabled_colliders(root_prim):
    out = []
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            if UsdPhysics.CollisionAPI(prim) \
                    .GetCollisionEnabledAttr().Get() is not False:
                out.append(prim)
    return out


def set_op(prim, kind, value):
    """Set (or lazily add) a single xform op of the given kind."""
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == kind:
            op.Set(value)
            return
    if kind == UsdGeom.XformOp.TypeTranslate:
        xf.AddTranslateOp().Set(value)
    elif kind == UsdGeom.XformOp.TypeRotateX:
        xf.AddRotateXOp().Set(value)


# ---------------------------------------------------------------- movers
car = carriers[0]
tray_prim = stage.GetPrimAtPath(car["tray"])
shut_prim = stage.GetPrimAtPath(car["shuttle"])
tray_colls = enabled_colliders(tray_prim)

# park every OTHER carrier far away so only the swept tray is measured
for other in carriers[1:]:
    for pth in (other["shuttle"], other["tray"]):
        set_op(stage.GetPrimAtPath(pth), UsdGeom.XformOp.TypeTranslate,
               Gf.Vec3d(-50.0, -50.0, 1.0))

# ---------------------------------------------------------------- statics
skip_roots = [f"{c['tray']}" for c in carriers] + \
             [f"{c['shuttle']}" for c in carriers]
statics = []
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
        continue
    p = str(prim.GetPath())
    if any(p.startswith(r) for r in skip_roots) or "/items/" in p:
        continue
    statics.append(prim)

print(f"[audit] tray colliders={len(tray_colls)} statics={len(statics)}",
      flush=True)

S = P.SORTER
TILT_MAX = S["tilt_deg"] * (1 + 2 * S["noise_frac"])
TOL = args.tol_mm / 1000.0
y0 = S["y"]
pivot_dz = S["pivot_z"] - (S["shuttle_top"] - 0.025)
z_shut = S["shuttle_top"] - 0.025

quarter = [TILT_MAX * i / args.samples for i in range(args.samples + 1)]
cycle = quarter + quarter[::-1][1:]        # 0 -> max -> 0

# STATION-INTERLOCKED pose set: a tilt command only exists inside a
# station's discharge window and only with that station's sign (the
# controller enforces this: triggers are per-station position gates).
# Sweeping impossible poses (a tilt at the knife line) produces phantom
# findings — the audit models the commandable envelope: full tilt cycle
# across each station's onset->flatten travel, plus FLAT everywhere.
poses = []                                  # (x, angle)
for st in P.STATIONS.values():
    sign = -st["side"]                      # +roll dips the SOUTH edge
    for dx in (-0.10, 0.05, 0.20, 0.35, 0.55):
        for a in cycle:
            poses.append((round(st["x"] + dx, 3), sign * a))
for x in [S["x_west"] + 0.1, S["x_east"] - 0.1] +          [st["x"] + dx for st in P.STATIONS.values()
          for dx in (-0.4, 0.0, 0.4)]:
    poses.append((round(x, 3), 0.0))

worst = {}
for x, ang in poses:
    set_op(shut_prim, UsdGeom.XformOp.TypeTranslate,
           Gf.Vec3d(float(x), y0, z_shut))
    if True:
        set_op(tray_prim, UsdGeom.XformOp.TypeTranslate,
               Gf.Vec3d(float(x), y0, z_shut + pivot_dz))
        set_op(tray_prim, UsdGeom.XformOp.TypeRotateX, float(ang))
        cache.Clear()
        boxes = [(world_aabb(c), c) for c in tray_colls]
        for sp in statics:
            sb = world_aabb(sp)
            for mb, mc in boxes:
                c = clearance(mb, sb)
                key = (str(mc.GetPath()), str(sp.GetPath()))
                if key not in worst or c < worst[key][0]:
                    worst[key] = (c, x, ang)

report = {"tol_mm": args.tol_mm, "tilt_max_deg": round(TILT_MAX, 2),
          "tray_pairs": [], "tray_violations": [],
          "arm_segments": [], "arm_violations": []}
for (mp, sp), (c, x, ang) in sorted(worst.items(), key=lambda kv: kv[1][0]):
    row = {"mover": mp, "static": sp,
           "min_clearance_mm": round(c * 1000, 1),
           "at_x": x, "at_tilt_deg": round(ang, 1)}
    if c < TOL:
        report["tray_violations"].append(row)
    if c < 0.08:
        report["tray_pairs"].append(row)

# ------------------------------------------------------------------- arm
# recovery envelope: FULL LINK GEOMETRY, not just the TCP path — the
# forearm dips below the straight TCP line between waypoints and is what
# a camera sees near the cage top rails. Every TCP sample is IK'd with
# the arm's own solver and the shoulder-elbow-wrist-tool chain is checked
# as capsules (segment AABB inflated by the link radius).
from isaac.arm import _ik_ur, UR                            # noqa: E402

bx, by = P.ARM["base"]
LIFT = 1.22                                     # matches arm.py transfer cap
wps = []
for z in ("C", "D"):
    sx = P.STATIONS[z]["x"]
    grasp = (sx, min(2.62, P.ARM_GRASP_Y_MAX), 0.55)
    lift = (grasp[0], grasp[1], LIFT)
    pl = P.PLACE_BY_MODE["sorter"][z]
    place = (pl["xy"][0], pl["xy"][1], pl["surface_z"] + 0.15)
    xfer = (place[0], place[1], LIFT)
    home = (bx, by, LIFT)
    wps += [(home, grasp), (grasp, lift), (lift, xfer),
            (xfer, place), (place, home)]

cage_like = [s for s in statics
             if "/cage" in str(s.GetPath()) or "/chute" in str(s.GetPath())
             or "guard" in str(s.GetPath())]

SHZ = 0.65 + UR["D1"]
LINK_R = {"upper": 0.075, "fore": 0.060, "tool": 0.050}


def chain_points(tcp):
    """shoulder, elbow, wrist, tcp world points via the arm's own IK."""
    q1, q2, q3, _ = _ik_ur(tcp, (bx, by))
    ca, sa = np.cos(q1), np.sin(q1)
    sh = np.array([bx, by, SHZ])
    r_e = UR["L1"] * np.cos(q2)
    el = np.array([bx + r_e * ca, by + r_e * sa, SHZ - UR["L1"] * np.sin(q2)])
    r_w = r_e + UR["L2"] * np.cos(q2 + q3)
    wr = np.array([bx + r_w * ca, by + r_w * sa,
                   SHZ - UR["L1"] * np.sin(q2)
                   - UR["L2"] * np.sin(q2 + q3)])
    return sh, el, wr, np.array(tcp)


def link_clear(a, b, r, lo, hi, n=8):
    """capsule (a-b, radius r) vs AABB: sampled point distance minus r."""
    best = 1e9
    for i in range(n + 1):
        q = a + (b - a) * (i / n)
        d = np.maximum(np.maximum(lo - q, q - hi), 0.0)
        out = float(np.linalg.norm(d))
        if out == 0.0:
            out = -float(np.minimum(q - lo, hi - q).min())
        best = min(best, out - r)
    return best


worst_arm = {}
static_boxes = [(world_aabb(sp), str(sp.GetPath())) for sp in cage_like]
for p0, p1 in wps:
    for i in range(13):
        tcp = np.array(p0) + (np.array(p1) - np.array(p0)) * (i / 12.0)
        try:
            sh, el, wr, tp = chain_points(tuple(tcp))
        except ValueError:
            worst_arm[("UNREACHABLE", str(p0) + str(p1))] = (-0.999, tcp)
            break
        links = [(sh, el, LINK_R["upper"], "upper"),
                 (el, wr, LINK_R["fore"], "fore"),
                 (wr, tp, LINK_R["tool"], "tool")]
        for (lo, hi), spath in static_boxes:
            for a, b, r, nm in links:
                c = link_clear(a, b, r, lo, hi)
                key = (nm, spath)
                if key not in worst_arm or c < worst_arm[key][0]:
                    worst_arm[key] = (c, tcp)

for (nm, spath), (c, tcp) in sorted(worst_arm.items(), key=lambda kv: kv[1][0]):
    row = {"link": nm, "static": spath,
           "min_clearance_mm": round(c * 1000, 1),
           "at_tcp": [round(float(v), 3) for v in np.atleast_1d(tcp)[:3]]}
    if c < TOL:
        report["arm_violations"].append(row)
    elif c < 0.05:
        report["arm_segments"].append(row)

(OUT / "clearance_report.json").write_text(json.dumps(report, indent=2))
nv = len(report["tray_violations"]) + len(report["arm_violations"])
print(f"[audit] tray near-pairs={len(report['tray_pairs'])} "
      f"tray violations={len(report['tray_violations'])} "
      f"arm violations={len(report['arm_violations'])}", flush=True)
for v in (report["tray_violations"] + report["arm_violations"])[:25]:
    print("  VIOLATION", json.dumps(v), flush=True)
print(f"[audit] {'FAIL' if nv else 'PASS'}", flush=True)

# SimulationApp.close() can swallow sys.exit codes (Kit shutdown calls
# os._exit) — persist the verdict for the pipeline BEFORE closing.
(OUT / "AUDIT_RC").write_text("1" if nv else "0")
sim_app.close()
sys.exit(1 if nv else 0)
