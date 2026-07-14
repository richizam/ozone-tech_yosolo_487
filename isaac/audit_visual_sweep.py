# -*- coding: utf-8 -*-
"""VISUAL-INCLUSIVE mechanism sweep audit (hard-correction gate).

audit_clearance.py audits PhysX COLLIDERS only (prims with CollisionAPI).
That left a blind spot: visual-only geometry (collide=False chassis
dressing, portal furniture, sign quads, brand strips) was never checked
against the tray's swept volume — and "no collision event" says nothing
about render meshes clipping. This audit closes that gap:

  1. TRAY SWEEP vs EVERYTHING: poses a carrier at every discharge station
     at 0/25/50/75/100% of commanded tilt in the station's direction,
     plus the +-(tilt+8 deg) JOINT-LIMIT fault pose, across the
     onset->flatten travel window; measures TRUE oriented min distance
     between every tray prim (plates, lips, visual chamfer strips, hinge
     parts) and EVERY other prim in the stage — Cube, Cylinder, and Mesh
     (sign quads), visual or collider alike. Flat poses along the whole
     top run and a RETURN-LEG pose (chassis vs floor hardware) included.
  2. ADJACENT CARRIER: a neighbour parked at chain pitch — the tilted
     tray must never touch the next carrier (the 10 mm pitch gap is a
     designed interface, reported explicitly).
  3. ITEM FALL CORRIDORS: an OBB prism hugging each chute slope (mouth ->
     cage aperture, full chute width, 0.44 m headroom) and the B incline;
     ANY non-whitelisted prim intersecting a corridor is a violation —
     nothing may stand where freight travels.

Whitelists are DOCUMENTED DESIGN INTERFACES only: the mouth labyrinth
(cheeks/chamfer), the B-handoff plane, and hinge bore pairs (shaft inside
saddle/knuckle). Everything else under tolerance FAILS.

    /isaac-sim/python.sh isaac/audit_visual_sweep.py \
        --out /workspace/sortmaster_out/audit_sweep
Exit 0 = clean; 1 = violations (JSON + table in --out).
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

ap = argparse.ArgumentParser()
ap.add_argument("--out", default="/workspace/sortmaster_out/audit_sweep")
ap.add_argument("--tol-mm", type=float, default=12.0)
ap.add_argument("--report-mm", type=float, default=25.0)
args = ap.parse_args()

from isaacsim import SimulationApp                          # noqa: E402
sim_app = SimulationApp({"headless": True})

import itertools                                            # noqa: E402
import numpy as np                                          # noqa: E402
from pxr import Usd, UsdGeom, Gf                            # noqa: E402
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


# ------------------------------------------------------------- OBB helpers
def _box_samples_unit():
    pts = set()
    for sx, sy, sz in itertools.product((-1, 0, 1), repeat=3):
        if sx or sy or sz:
            pts.add((float(sx), float(sy), float(sz)))
    for axis in range(3):
        for side in (-1, 1):
            for a in (-0.5, 0.0, 0.5):
                for b in (-0.5, 0.0, 0.5):
                    q = [0.0, 0.0, 0.0]
                    q[axis] = float(side)
                    o = [i for i in range(3) if i != axis]
                    q[o[0]], q[o[1]] = a * 2, b * 2
                    pts.add(tuple(q))
    return np.array(sorted(pts))


UNIT_SAMPLES = _box_samples_unit()


def prim_obb(prim):
    """(origin, unit_rows, half) world OBB of ANY boundable prim, from its
    LOCAL aligned bound (Cube: +-1*scale; Cylinder: (r,r,h/2)*scale; Mesh:
    its point extent). Returns None for unboundable/empty prims."""
    lb = cache.ComputeLocalBound(prim).GetRange()
    if lb.IsEmpty():
        return None
    lo = np.array(lb.GetMin())
    hi = np.array(lb.GetMax())
    c_loc = (lo + hi) / 2
    e_loc = (hi - lo) / 2
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    m = np.array([[xf[i][j] for j in range(4)] for i in range(4)])
    rs = m[:3, :3]
    rl = np.linalg.norm(rs, axis=1)
    if (rl < 1e-12).any() or (e_loc < 0).any():
        return None
    unit = rs / rl[:, None]
    h = e_loc * rl
    origin = c_loc @ rs + m[3, :3]
    return origin, unit, np.maximum(h, 1e-6)


def _signed_pts_box(pts_world, origin, unit, h):
    rel = (pts_world - origin) @ unit.T
    d_out = np.maximum(np.abs(rel) - h, 0.0)
    outside = np.linalg.norm(d_out, axis=1)
    inner = (h - np.abs(rel)).min(axis=1)
    return float(np.where(outside > 0, outside, -inner).min())


def obb_gap(a, b):
    ao, au, ah = a
    bo, bu, bh = b
    a_pts = (UNIT_SAMPLES * ah) @ au + ao
    b_pts = (UNIT_SAMPLES * bh) @ bu + bo
    return min(_signed_pts_box(a_pts, bo, bu, bh),
               _signed_pts_box(b_pts, ao, au, ah))


def obb_aabb(o):
    origin, unit, h = o
    ext = np.abs(unit * h[:, None]).sum(axis=0)
    return origin - ext, origin + ext


def aabb_gap(a, b):
    return float(np.maximum(b[0] - a[1], a[0] - b[1]).max())


def set_op(prim, kind, value):
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == kind:
            op.Set(value)
            return
    if kind == UsdGeom.XformOp.TypeTranslate:
        xf.AddTranslateOp().Set(value)
    elif kind == UsdGeom.XformOp.TypeRotateX:
        xf.AddRotateXOp().Set(value)


# ------------------------------------------------------- mover / world sets
S = P.SORTER
y0 = S["y"]
pivot_dz = S["pivot_z"] - (S["shuttle_top"] - 0.025)
z_shut = S["shuttle_top"] - 0.025
z_ret = S["return_z"] - 0.05
PITCH = S["pitch"]
TILT = S["tilt_deg"]
TILT_LIM = TILT + 8.0                       # revolute joint limit (fault)

car0, car1 = carriers[0], carriers[1]
tray_prim = stage.GetPrimAtPath(car0["tray"])
shut_prim = stage.GetPrimAtPath(car0["shuttle"])
n_tray = stage.GetPrimAtPath(car1["tray"])
n_shut = stage.GetPrimAtPath(car1["shuttle"])

# park all remaining carriers far away
for other in carriers[2:]:
    for pth in (other["shuttle"], other["tray"]):
        set_op(stage.GetPrimAtPath(pth), UsdGeom.XformOp.TypeTranslate,
               Gf.Vec3d(-50.0, -50.0, 1.0))

GPRIM_TYPES = ("Cube", "Cylinder", "Mesh", "Capsule", "Cone", "Sphere")


def gprims_under(root_path):
    root = stage.GetPrimAtPath(root_path)
    return [p for p in Usd.PrimRange(root)
            if p.GetTypeName() in GPRIM_TYPES]


tray_parts = gprims_under(car0["tray"])
own_chassis = gprims_under(car0["shuttle"])
neigh_parts = gprims_under(car1["tray"]) + gprims_under(car1["shuttle"])

statics = []
mover_roots = [car0["tray"], car0["shuttle"], car1["tray"], car1["shuttle"]] \
    + [c["tray"] for c in carriers[2:]] + [c["shuttle"] for c in carriers[2:]]
for prim in stage.Traverse():
    if prim.GetTypeName() not in GPRIM_TYPES:
        continue
    p = str(prim.GetPath())
    if any(p.startswith(r) for r in mover_roots):
        continue
    if "/items/" in p or "/lights/" in p or "/arm" in p.lower():
        continue
    statics.append(prim)

print(f"[sweep] tray_parts={len(tray_parts)} own_chassis={len(own_chassis)} "
      f"statics={len(statics)}", flush=True)

# DESIGNED-INTERFACE whitelists (documented; reported, not failed)
LAB_RE = re.compile(r"chute(C|D|REVIEW)_(cheek_|chamfer)")
HANDOFF_RE = re.compile(r"/bconnect($|_rail)")
KNIFE_RE = re.compile(r"/conveyors/knife|knife_(cheek|drum)")
BORE_RE = re.compile(r"(pivot_shaft|saddle|knuckle|bearing)")
NEIGH_NOTE = "designed chain-pitch interface (pitch - tray_l = 10 mm)"


def pair_class(mp, sp):
    if BORE_RE.search(mp) and BORE_RE.search(sp):
        return "bore"
    if LAB_RE.search(sp) or LAB_RE.search(mp):
        return "labyrinth"
    if HANDOFF_RE.search(sp) or KNIFE_RE.search(sp):
        return "handoff"
    return "hard"


# --------------------------------------------------------------- pose sweep
def pose_carrier(shut, tray, x, z, ang):
    set_op(shut, UsdGeom.XformOp.TypeTranslate, Gf.Vec3d(float(x), y0, z))
    set_op(tray, UsdGeom.XformOp.TypeTranslate,
           Gf.Vec3d(float(x), y0, z + pivot_dz))
    set_op(tray, UsdGeom.XformOp.TypeRotateX, float(ang))


poses = []          # (x, angle_deg, kind)


def _clamp_run(x):
    """Carriers exist only on the physical top run: the chain wraps at
    x_east — poses beyond it are unreachable even in a stuck-tilt fault
    (the first sweep flagged phantom clips at x 9.60 > x_east 9.42)."""
    return min(max(x, S["x_west"]), S["x_east"])


for st in P.STATIONS.values():
    sgn = -st["side"]
    for dx in (-0.10, 0.05, 0.20, 0.35, 0.55):
        for frac in (0.0, 0.25, 0.50, 0.75, 1.0):
            poses.append((_clamp_run(st["x"] + dx), sgn * frac * TILT,
                          "commanded"))
    # stuck-tilt fault: full tilt persists to the wrap line itself
    poses.append((S["x_east"], sgn * TILT, "commanded"))
    poses.append((_clamp_run(st["x"] + 0.20), sgn * TILT_LIM, "joint_limit"))
    poses.append((_clamp_run(st["x"] + 0.20), -sgn * TILT_LIM,
                  "joint_limit_wrong_side"))
for x in np.arange(S["x_west"] + 0.05, S["x_east"] - 0.04, 0.30):
    poses.append((float(x), 0.0, "flat_run"))
poses.append((S["x_east"], 0.0, "flat_run"))
poses.append((S["x_west"], 0.0, "flat_run"))

worst = {}
static_obbs = None
for x, ang, kind in poses:
    pose_carrier(shut_prim, tray_prim, x, z_shut, ang)
    pose_carrier(n_shut, n_tray, x + PITCH, z_shut, 0.0)
    cache.Clear()
    if static_obbs is None:      # statics never move: build reprs once
        static_obbs = []
        for sp in statics:
            o = prim_obb(sp)
            if o is not None:
                static_obbs.append((sp, o, obb_aabb(o)))
    movers = []
    for mp in tray_parts:
        o = prim_obb(mp)
        if o is not None:
            movers.append((mp, o, obb_aabb(o)))
    targets = [(sp, so, sa, "static") for sp, so, sa in static_obbs
               if abs(so[0][0] - x) < 1.4]
    for cp in own_chassis:
        o = prim_obb(cp)
        if o is not None:
            targets.append((cp, o, obb_aabb(o), "own_chassis"))
    for npr in neigh_parts:
        o = prim_obb(npr)
        if o is not None:
            targets.append((npr, o, obb_aabb(o), "neighbour"))
    for mp, mo, ma in movers:
        for sp, so, sa, grp in targets:
            if aabb_gap(ma, sa) > args.report_mm / 1000.0:
                continue
            g = obb_gap(mo, so)
            if g > args.report_mm / 1000.0:
                continue
            key = (str(mp.GetPath()), str(sp.GetPath()))
            if key not in worst or g < worst[key][0]:
                worst[key] = (g, round(float(x), 3), round(float(ang), 1),
                              kind, grp)

# ------------------------------------------------- return-leg chassis check
# x 8.05 = mid-run under the C/D chutes; 6.77 / 9.37 = the end-module
# over-run strips (the first sweep missed end hardware on the return leg)
for rx in (6.77, 8.05, 9.37):
    pose_carrier(shut_prim, tray_prim, rx, z_ret, 0.0)
    pose_carrier(n_shut, n_tray, rx + PITCH, z_ret, 0.0)
    cache.Clear()
    for mp in tray_parts + own_chassis:
        o = prim_obb(mp)
        if o is None:
            continue
        ma = obb_aabb(o)
        # floor plane z=0
        zmin = float(ma[0][2])
        if zmin < 0.004:
            key = (str(mp.GetPath()), "/floor(z=0)")
            worst[key] = (zmin, rx, 0.0, "return_leg", "floor")
        for sp, so, sa in static_obbs:
            if abs(so[0][0] - rx) > 1.4:
                continue
            if aabb_gap(ma, sa) > args.report_mm / 1000.0:
                continue
            g = obb_gap(o, so)
            if g <= args.report_mm / 1000.0:
                key = (str(mp.GetPath()), str(sp.GetPath()))
                if key not in worst or g < worst[key][0]:
                    worst[key] = (g, rx, 0.0, "return_leg", "static")

# ---------------------------------------------------- item fall corridors
def corridor_obbs(zone, cc):
    """TWO OBB prisms hugging the slope: a tall MOUTH section (0.44 m
    headroom — items can tumble at discharge) and a lower TAIL section
    (0.30 m — by mid-slide items have settled; freight cannot grow).
    A uniform 0.44 prism false-flagged the parallel B incline hardware
    high above the tail of the REVIEW slope."""
    d = float(cc["dir"])
    y_start, z_start = cc["y0"], cc["z0"]
    y_end, _pad = P.chute_run(cc)
    tan32 = math.tan(math.radians(32.0))
    cos32 = math.cos(math.radians(32.0))
    ang = -d * math.radians(32.0)
    out = []
    # f0 0.02: the corridor begins just past the mouth edge — freight
    # crosses the mouth plane airborne ABOVE the chamfer lip; the wedge
    # under the tray line south of the mouth is not a freight volume
    for f0, f1, head in ((0.02, 0.55, 0.44), (0.55, 1.0, 0.30)):
        ya = y_start + (y_end - y_start) * f0
        yb = y_start + (y_end - y_start) * f1
        za = z_start - abs(ya - y_start) * tan32
        zb = z_start - abs(yb - y_start) * tan32
        run = math.hypot(yb - ya, za - zb)
        cy, cz = (ya + yb) / 2, (za + zb) / 2 + (head / 2) * cos32 + 0.012
        origin = np.array([cc["cx"], cy, cz])
        ca, sa = math.cos(ang), math.sin(ang)
        unit = np.array([[1.0, 0, 0], [0, ca, sa], [0, -sa, ca]])
        half = np.array([cc["width"] / 2 - 0.012, run / 2, head / 2 - 0.012])
        out.append((origin, unit, half))
    return out


CORR_WL = {
    "C": (r"chuteC", r"cageC", r"/tray\d", r"/car\d"),
    "D": (r"chuteD", r"cageD", r"/tray\d", r"/car\d"),
    "REVIEW": (r"chuteREVIEW", r"cageREVIEW", r"/tray\d", r"/car\d",
               r"review"),
}
corridor_hits = []
for zone, cc in (("C", P.CHUTE_C), ("D", P.CHUTE_D),
                 ("REVIEW", P.CHUTE_REVIEW)):
    wl = [re.compile(w, re.I) for w in CORR_WL[zone]]
    seen = {}
    for co in corridor_obbs(zone, cc):
        ca_box = obb_aabb(co)
        for sp, so, sa in static_obbs:
            if aabb_gap(ca_box, sa) > 0.0:
                continue
            p = str(sp.GetPath())
            if any(w.search(p) for w in wl):
                continue
            g = obb_gap(co, so)
            if g < 0.0 and (-g) > seen.get(p, 0.0):
                seen[p] = -g
    for p, depth in seen.items():
        corridor_hits.append({"corridor": zone, "prim": p,
                              "depth_mm": round(depth * 1000, 1)})
# B incline corridor (tray lip line -> belt B infeed)
bc = P.B_CONNECT
ang_b = math.atan2(bc["z_top1"] - bc["z_top0"], bc["y1"] - bc["y0"])
run_b = math.hypot(bc["y1"] - bc["y0"], bc["z_top1"] - bc["z_top0"])
origin_b = np.array([bc["cx"], (bc["y0"] + bc["y1"]) / 2,
                     (bc["z_top0"] + bc["z_top1"]) / 2 + 0.20])
ca_, sa_ = math.cos(ang_b), math.sin(ang_b)
unit_b = np.array([[1.0, 0, 0], [0, ca_, sa_], [0, -sa_, ca_]])
half_b = np.array([bc["width"] / 2 - 0.012, run_b / 2, 0.19])
co_b = (origin_b, unit_b, half_b)
cb_box = obb_aabb(co_b)
wl_b = [re.compile(w, re.I) for w in
        (r"bconnect", r"bnose", r"cheekCn", r"drumCn", r"bconn_",
         r"/tray\d", r"/car\d", r"beltB", r"/belt_b", r"bdrive")]
for sp, so, sa in static_obbs:
    if aabb_gap(cb_box, sa) > 0.0:
        continue
    p = str(sp.GetPath())
    if any(w.search(p) for w in wl_b):
        continue
    g = obb_gap(co_b, so)
    if g < 0.0:
        corridor_hits.append({"corridor": "B", "prim": p,
                              "depth_mm": round(-g * 1000, 1)})

# -------------------------------------------------------------- reporting
TOL = args.tol_mm / 1000.0
rows, violations = [], []
for (mp, sp), (g, x, ang, kind, grp) in sorted(
        worst.items(), key=lambda kv: kv[1][0]):
    cls = pair_class(mp, sp)
    if cls == "bore":
        continue                      # intended concentric fit
    row = {"mover": mp, "static": sp, "min_clearance_mm": round(g * 1000, 1),
           "at_x": x, "at_tilt_deg": ang, "pose_kind": kind, "group": grp,
           "class": cls}
    rows.append(row)
    lim = 0.004 if cls in ("labyrinth", "handoff") else \
        (0.008 if grp == "neighbour" or kind.startswith("joint_limit")
         else TOL)
    if g < lim:
        violations.append(row)

report = {
    "tol_mm": args.tol_mm,
    "tilt_deg": TILT, "joint_limit_deg": TILT_LIM,
    "pairs_reported": rows,
    "violations": violations,
    "corridor_violations": corridor_hits,
    "notes": {
        "labyrinth": "mouth cheeks/chamfer designed tight throat seal "
                     "(>=4 mm at commanded tilt incl. noise)",
        "handoff": "B incline designed close discharge plane",
        "neighbour": NEIGH_NOTE,
        "joint_limit": "fault-envelope pose (drive limit), pass >= 8 mm",
    },
}
(OUT / "sweep_report.json").write_text(json.dumps(report, indent=1),
                                       encoding="utf-8")
with (OUT / "sweep_report.txt").open("w", encoding="utf-8") as f:
    f.write(f"visual sweep audit  tol={args.tol_mm}mm  "
            f"tilt={TILT} limit={TILT_LIM}\n")
    f.write(f"VIOLATIONS: {len(violations)}  "
            f"CORRIDOR: {len(corridor_hits)}\n")
    for r in violations:
        f.write(f"  VIOL {r['min_clearance_mm']:8.1f}mm  {r['mover']}  ~  "
                f"{r['static']}  @x={r['at_x']} tilt={r['at_tilt_deg']} "
                f"({r['pose_kind']},{r['class']})\n")
    for c in corridor_hits:
        f.write(f"  CORR {c['depth_mm']:8.1f}mm INSIDE corridor "
                f"{c['corridor']}: {c['prim']}\n")
    f.write("\nall close pairs (<= report window):\n")
    for r in rows[:400]:
        f.write(f"  {r['min_clearance_mm']:8.1f}mm  [{r['class']:9s}] "
                f"{r['mover']}  ~  {r['static']} @tilt {r['at_tilt_deg']}\n")

print(f"[sweep] violations={len(violations)} corridor={len(corridor_hits)} "
      f"reported_pairs={len(rows)}", flush=True)
rc = 0 if not violations and not corridor_hits else 1
(OUT / "SWEEP_RC").write_text(str(rc), encoding="utf-8")
sim_app.close()
sys.exit(rc)
