# -*- coding: utf-8 -*-
"""USD scene builder for the Isaac Sim port of the SortMaster cell.

Same single source of truth as the MuJoCo build: every dimension comes from
cell/params.py (table-mode executive). Statics are USD Cube colliders; the
three exit gates are rigid bodies on prismatic joints with linear drives;
items are rigid bodies whose collision shape is a PhysX convex hull of the
official STL surface (matching MuJoCo's maxhullvert=64 idiom).

Runs INSIDE Isaac Sim's Python (pxr available). No trimesh/importers needed:
binary STLs are parsed with numpy directly into UsdGeom.Mesh vertex soup.
"""
import json
import math
from pathlib import Path

import numpy as np
from pxr import (Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade, Vt)

from cell import params as P

STEEL = (0.55, 0.57, 0.62)
DARK = (0.22, 0.24, 0.28)
BELT_COL = (0.085, 0.088, 0.095)       # dark rubber belt surface
TABLE_COL = (0.11, 0.115, 0.13)        # dark roller deck
RAIL_COL = (0.95, 0.78, 0.06)          # safety yellow guards
CHUTE_COL = (0.50, 0.51, 0.55)         # brushed steel chute
ITEM_COL = (0.75, 0.72, 0.65)

ROOT = "/World"


# --------------------------------------------------------------------- helpers
def _sanitize(name):
    return name.replace("-", "_").replace(" ", "_").replace("+", "p").replace(".", "_")


def read_stl(path):
    """Binary STL -> (points Nx3 float32, n_faces). Vertex soup (per-face
    vertices) — exactly what the STL stores; fine for rendering and for PhysX
    convex-hull cooking."""
    raw = Path(path).read_bytes()
    n = int(np.frombuffer(raw, dtype="<u4", count=1, offset=80)[0])
    rec = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    body = np.frombuffer(raw, dtype=rec, count=n, offset=84)
    pts = body["v"].reshape(-1, 3).astype(np.float32)
    return pts, n


def mat_to_quat(R):
    """3x3 rotation matrix (columns = axes) -> quaternion (w, x, y, z),
    plain python floats (Gf constructors reject numpy scalars)."""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s)
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s,
                 (m[0, 2] + m[2, 0]) / s)
        elif i == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s,
                 (m[1, 2] + m[2, 1]) / s)
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                 (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    return tuple(float(v) for v in q)


class SceneBuilder:
    def __init__(self, stage, repo_root, friction_mult=1.0, mass_mult=1.0):
        self.stage = stage
        self.repo = Path(repo_root)
        self.friction_mult = float(friction_mult)   # material-sweep knobs
        self.mass_mult = float(mass_mult)
        self.mats = {}
        UsdGeom.Xform.Define(stage, ROOT)
        UsdGeom.Xform.Define(stage, f"{ROOT}/statics")
        UsdGeom.Xform.Define(stage, f"{ROOT}/gates")
        UsdGeom.Xform.Define(stage, f"{ROOT}/items")
        UsdGeom.Xform.Define(stage, f"{ROOT}/cams")
        UsdGeom.Xform.Define(stage, f"{ROOT}/conveyors")
        UsdGeom.Xform.Define(stage, f"{ROOT}/blades")

    # ---------------------------------------------------------------- materials
    def phys_material(self, name, static_f, dynamic_f, restitution=0.0,
                      combine="min"):
        path = f"{ROOT}/PhysMats/{name}"
        if name in self.mats:
            return self.mats[name]
        mat = UsdShade.Material.Define(self.stage, path)
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(float(static_f))
        api.CreateDynamicFrictionAttr(float(dynamic_f))
        api.CreateRestitutionAttr(float(restitution))
        px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
        px.CreateFrictionCombineModeAttr(combine)
        px.CreateRestitutionCombineModeAttr("min")
        self.mats[name] = mat
        return mat

    def _bind_phys(self, prim, mat):
        if mat is None:
            return
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # ------------------------------------------------------------------- solids
    def add_box(self, name, center, half, euler_rad=(0, 0, 0), color=STEEL,
                opacity=1.0, collide=True, mat=None, parent="statics"):
        """Static box collider. euler_rad follows the MuJoCo values 1:1
        (single-axis rotations only in this cell, so order is irrelevant)."""
        path = f"{ROOT}/{parent}/{_sanitize(name)}"
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(2.0)                       # +-1 * scale = half
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(c) for c in center]))
        if any(abs(e) > 1e-9 for e in euler_rad):
            deg = [math.degrees(e) for e in euler_rad]
            xf.AddRotateXYZOp().Set(Gf.Vec3f(*deg))
        xf.AddScaleOp().Set(Gf.Vec3f(*[float(h) for h in half]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if opacity < 1.0:
            cube.CreateDisplayOpacityAttr([float(opacity)])
        if collide:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            # MuJoCo runs margin=0; PhysX's default ~2-4 cm contactOffset makes
            # speculative contacts fire across designed clearances (the hood
            # edge caught a flat box_l over a real 1 cm gap). Keep it tight.
            pxc = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            pxc.CreateContactOffsetAttr(0.005)
            pxc.CreateRestOffsetAttr(0.0)
            self._bind_phys(cube.GetPrim(), mat)
        return cube.GetPrim()

    # --------------------------------------------------------------- conveyors
    def make_conveyor(self, name, center, half, velocity=(0.0, 0.0, 0.0),
                      color=BELT_COL, mat=None, euler_rad=(0, 0, 0)):
        """Surface-velocity conveyor: kinematic rigid body whose contact
        friction drives whatever rests on it — the same PhysX mechanism the
        Isaac Conveyor Belt utility applies (RigidBodyAPI + CollisionAPI +
        PhysxSurfaceVelocityAPI). Items are carried by physics, never by
        scripted per-item velocity writes.

        `velocity` is the desired surface speed in m/s along the prim's
        LOCAL axes. PhysX applies surfaceVelocity in the local frame SCALED
        by the prim's xform scale (measured: a belt with half-length 3.45
        commanded 1.0 dragged items at exactly 3.450 m/s), so the attribute
        is written pre-divided by the scale — the belt then really moves
        freight at the designed speed (official belt spec: 1 m/s)."""
        path = f"{ROOT}/conveyors/{_sanitize(name)}"
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(2.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(c) for c in center]))
        if any(abs(e) > 1e-9 for e in euler_rad):
            xf.AddRotateXYZOp().Set(Gf.Vec3f(*[math.degrees(e) for e in euler_rad]))
        xf.AddScaleOp().Set(Gf.Vec3f(*[float(h) for h in half]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateKinematicEnabledAttr(True)
        sv = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
        sv.CreateSurfaceVelocityAttr(Gf.Vec3f(*[float(v) / float(h)
                                                for v, h in zip(velocity, half)]))
        self._bind_phys(prim, mat)
        return path

    def build_blade(self, name, x, y, width, along="y", filter_paths=()):
        """Pop-up stop blade (zone accumulation / escapement / induction):
        a thin panel on a vertical prismatic drive, normally LOWERED flush
        below the belt surface, raised to block the flow — the physical
        mechanism real accumulation conveyors use. Returns the joint path."""
        top = P.BELT_A["top"]
        h2, t2 = 0.06, 0.008
        # park DEEP below the surface: a lip only a few mm under a 1 m/s belt
        # is a seam that kicks passing items (speculative contacts on the
        # panel's top edge launched pouf clean off belt A)
        zc = top - h2 - 0.046
        body_path = f"{ROOT}/blades/{_sanitize(name)}"
        xform = UsdGeom.Xform.Define(self.stage, body_path)
        UsdGeom.Xformable(xform.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(float(x), float(y), zc))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr(2.0)
        geom = UsdGeom.Cube.Define(self.stage, f"{body_path}/panel")
        geom.CreateSizeAttr(2.0)
        half = ((t2, width / 2, h2) if along == "y" else (width / 2, t2, h2))
        UsdGeom.Xformable(geom.GetPrim()).AddScaleOp().Set(Gf.Vec3f(*half))
        # industrial pneumatic blade stop: dark anodized body, thin hazard
        # strip on the crest, side actuator cylinders (visual only)
        geom.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.17, 0.20)])
        strip = UsdGeom.Cube.Define(self.stage, f"{body_path}/strip")
        strip.CreateSizeAttr(2.0)
        sxf = UsdGeom.Xformable(strip.GetPrim())
        sxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, float(h2)))
        sxf.AddScaleOp().Set(Gf.Vec3f(half[0] + 0.001, half[1] + 0.001, 0.006))
        strip.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.78, 0.06)])
        for sgn in (-1, 1):
            act = UsdGeom.Cylinder.Define(self.stage,
                                          f"{body_path}/act{'lr'[sgn > 0]}")
            act.CreateRadiusAttr(0.014)
            act.CreateHeightAttr(float(h2 * 1.6))
            act.CreateAxisAttr("Z")
            axf = UsdGeom.Xformable(act.GetPrim())
            off = (half[1] if along == "y" else half[0]) - 0.02
            axf.AddTranslateOp().Set(
                Gf.Vec3d(0, sgn * off, -0.02) if along == "y"
                else Gf.Vec3d(sgn * off, 0, -0.02))
            act.CreateDisplayColorAttr([Gf.Vec3f(0.62, 0.64, 0.68)])
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        if filter_paths:
            # the lowered blade parks inside the conveyor volume — filter that
            # pair so the kinematic belt never fights the blade body
            fp = UsdPhysics.FilteredPairsAPI.Apply(geom.GetPrim())
            rel = fp.CreateFilteredPairsRel()
            for fpath in filter_paths:
                rel.AddTarget(Sdf.Path(fpath))
        joint = UsdPhysics.PrismaticJoint.Define(self.stage,
                                                 f"{body_path}_joint")
        joint.CreateAxisAttr("Z")
        joint.CreateLowerLimitAttr(-0.002)
        joint.CreateUpperLimitAttr(0.28)
        joint.CreateBody1Rel().SetTargets([Sdf.Path(body_path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(float(x), float(y), zc))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(3.0e4)
        drive.CreateDampingAttr(2.0e3)
        drive.CreateMaxForceAttr(900.0)
        drive.CreateTargetPositionAttr(0.0)
        return joint.GetPrim().GetPath().pathString

    def add_cylinder(self, name, center, radius, half_h, color=STEEL,
                     axis="Z", parent="statics"):
        """Visual-only cylinder (frame posts etc.)."""
        path = f"{ROOT}/{parent}/{_sanitize(name)}"
        cyl = UsdGeom.Cylinder.Define(self.stage, path)
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(half_h * 2))
        cyl.CreateAxisAttr(axis)
        xf = UsdGeom.Xformable(cyl.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(c) for c in center]))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        return cyl.GetPrim()

    # -------------------------------------------------------------------- cages
    def build_cage(self, zone, cage, mat_wall, mat_mat):
        cx, cy = cage["center"]
        ix, iy = cage["inner"]
        t, h = cage["wall_t"], cage["wall_h"]
        hx, hy, hh = ix / 2, iy / 2, h / 2
        col = P.ROUTE_RGBA[zone]
        self.add_box(f"cage{zone}_floor", (cx, cy, t / 2), (hx + t, hy + t, t / 2),
                     color=tuple(c * 0.55 for c in col), mat=mat_wall)
        walls = {"+y": (0, hy + t / 2, hx + t, t / 2), "-y": (0, -(hy + t / 2), hx + t, t / 2),
                 "+x": (hx + t / 2, 0, t / 2, hy), "-x": (-(hx + t / 2), 0, t / 2, hy)}
        open_side = cage.get("open_side")
        for side, (dx, dy, sx, sy) in walls.items():
            sname = side.replace("+", "p").replace("-", "m")
            if side == open_side:
                aw2 = cage["aperture_w"] / 2
                top = cage["aperture_top"]
                skirt_h2 = 0.165
                hdr_h = (t + h - top) / 2
                if side in ("+x", "-x"):
                    for sgn, nm in ((1, "a"), (-1, "b")):
                        fl = (sy - aw2) / 2
                        self.add_box(f"cage{zone}_fl{nm}",
                                     (cx + dx, cy + sgn * (aw2 + fl), t + hh),
                                     (sx, fl, hh), color=col, opacity=0.45, mat=mat_wall)
                    if hdr_h > 0.005:
                        self.add_box(f"cage{zone}_hdr", (cx + dx, cy, top + hdr_h),
                                     (sx, aw2, hdr_h), color=col, opacity=0.45, mat=mat_wall)
                    self.add_box(f"cage{zone}_skirt", (cx + dx, cy, skirt_h2),
                                 (sx, aw2, skirt_h2), color=col, opacity=0.45, mat=mat_wall)
                else:
                    for sgn, nm in ((1, "a"), (-1, "b")):
                        fl = (sx - aw2) / 2
                        self.add_box(f"cage{zone}_fl{nm}",
                                     (cx + sgn * (aw2 + fl), cy + dy, t + hh),
                                     (fl, sy, hh), color=col, opacity=0.45, mat=mat_wall)
                    if hdr_h > 0.005:
                        self.add_box(f"cage{zone}_hdr", (cx, cy + dy, top + hdr_h),
                                     (aw2, sy, hdr_h), color=col, opacity=0.45, mat=mat_wall)
                    self.add_box(f"cage{zone}_skirt", (cx, cy + dy, skirt_h2),
                                 (aw2, sy, skirt_h2), color=col, opacity=0.45, mat=mat_wall)
            else:
                self.add_box(f"cage{zone}_w{sname}", (cx + dx, cy + dy, t + hh),
                             (sx, sy, hh), color=col, opacity=0.45, mat=mat_wall)
        if open_side:
            self.add_box(f"cage{zone}_mat", (cx, cy, t + 0.004), (hx, hy, 0.004),
                         color=(0.15, 0.15, 0.17), mat=mat_mat)
        for sx_ in (-1, 1):                       # visual frame posts
            for sy_ in (-1, 1):
                self.add_cylinder(f"cage{zone}_p{sx_}{sy_}",
                                  (cx + sx_ * (hx + t), cy + sy_ * (hy + t), (t + h) / 2),
                                  0.022, (t + h) / 2, color=col)

    # -------------------------------------------------------------------- chute
    def build_chute(self, zone, cc, axis, wall_at, mat_chute, mat_pad, mat_hood):
        if axis == "x":
            p0, p1, z0, z1 = cc["x0"], cc["x1"], cc["z0"], cc["z1"]
            pad_end, fixed = cc["pad_x1"], cc["cy"]
        else:
            p0, p1, z0, z1 = cc["y0"], cc["y1"], cc["z0"], cc["z1"]
            pad_end, fixed = cc["pad_y1"], cc["cx"]
        length = float(np.hypot(p1 - p0, z0 - z1))
        ang = float(np.arctan2(z0 - z1, abs(p1 - p0)))
        euler = (0, ang, 0) if axis == "x" else (ang, 0, 0)
        mid, zmid = (p0 + p1) / 2, (z0 + z1) / 2 - 0.015
        if axis == "x":
            center, half = (mid, fixed, zmid), (length / 2, cc["width"] / 2, 0.015)
        else:
            center, half = (fixed, mid, zmid), (cc["width"] / 2, length / 2, 0.015)
        self.add_box(f"chute{zone}", center, half, euler, CHUTE_COL, mat=mat_chute)
        # side rails to the cage wall plane
        rail_t = 0.015
        col = P.ROUTE_RGBA[zone]
        r1 = wall_at
        rmid = (p0 + r1) / 2
        rlen = float(np.hypot(r1 - p0, (z0 - z1) * abs(r1 - p0) / abs(p1 - p0)))
        for sgn, nm in ((1, "l"), (-1, "r")):
            off = sgn * (cc["width"] / 2 + rail_t)
            rz = z0 - (z0 - z1) * abs(rmid - p0) / abs(p1 - p0) + P.GUIDE_H / 2
            if axis == "x":
                rc, rh = (rmid, fixed + off, rz), (rlen / 2, rail_t, P.GUIDE_H / 2)
            else:
                rc, rh = (fixed + off, rmid, rz), (rail_t, rlen / 2, P.GUIDE_H / 2)
            self.add_box(f"chute{zone}_rail_{nm}", rc, rh, euler, col, mat=mat_chute)
        # brake pad
        pz = z1 - 0.005
        pmid = (p1 + pad_end) / 2
        plen = abs(pad_end - p1)
        if axis == "x":
            pc, ph = (pmid, fixed, pz - 0.015), (plen / 2, cc["width"] / 2, 0.015)
        else:
            pc, ph = (fixed, pmid, pz - 0.015), (cc["width"] / 2, plen / 2, 0.015)
        self.add_box(f"chute{zone}_pad", pc, ph, color=(0.30, 0.31, 0.35), mat=mat_pad)
        # hood parallel to the slope + brow strip on the wall plane.
        # PORT NOTE: PhysX convex-hull contact + non-zero contact offset eats
        # into the razor-thin MuJoCo hood clearance, so a flat oversize box
        # (box_l, 0.30 m tall) catches its top corner on the hood mouth before
        # it has descended the slope. Lift the hood 0.12 m in the Isaac build —
        # anti-fly-out is preserved (measured cage_max_z <= 0.55 m << aperture
        # top 0.83 m, so no escape window is opened) and the box noses cleanly
        # under. Geometry-only change; the MuJoCo canonical hood is untouched.
        HOOD_LIFT = 0.12
        hood = dict(P.HOOD, clearance=P.HOOD["clearance"] + HOOD_LIFT)
        tanang = float(np.tan(ang))
        dirn = 1.0 if axis == "x" else -1.0
        aw2 = 0.40

        def surf(c):
            return z0 - abs(c - p0) * tanang

        n_up = float(np.cos(ang))
        s0 = wall_at - dirn * hood["up"]
        s1 = wall_at + dirn * hood["into"]
        hmid = (s0 + s1) / 2
        hlen = float(np.hypot(s1 - s0, abs(s1 - s0) * tanang))
        zc = surf(hmid) + hood["clearance"] * n_up
        if axis == "x":
            hcz, hh_ = (hmid, fixed, zc), (hlen / 2, aw2, 0.012)
        else:
            hcz, hh_ = (fixed, hmid, zc), (aw2, hlen / 2, 0.012)
        self.add_box(f"hood{zone}", hcz, hh_, euler, col, opacity=0.45, mat=mat_hood)
        # brow strip on the wall plane: seals the window between the LIFTED
        # hood (z ~0.90 at the wall) and open air. It must start ABOVE the
        # entry sweep: at the true 0.8 m/s discharge a tall box (box_l,
        # 0.3 m) arrives still pitched nose-down and its top-front corner
        # passes the wall plane at ~0.83 m — the old 0.80..0.83 band was a
        # catch face exactly there (box_l wedged, watchdog -> operator
        # call-out). 0.86..0.98 clears the sweep and keeps the fly-out seal
        # (measured in-cage apex 0.54 m never approaches it).
        bz0, bz1 = 0.86, 0.98
        bh2 = (bz1 - bz0) / 2
        if axis == "x":
            bc, bh_ = (wall_at, fixed, bz0 + bh2), (0.015, aw2, bh2)
        else:
            bc, bh_ = (fixed, wall_at, bz0 + bh2), (aw2, 0.015, bh2)
        self.add_box(f"hood{zone}_brow", bc, bh_, color=col, opacity=0.45, mat=mat_hood)

    # -------------------------------------------------------------------- gates
    def build_gate(self, zone, mat_gate):
        gp = P.GATES[zone]
        tb = P.TABLE
        span = (gp["c1"] - gp["c0"]) / 2
        mid = (gp["c0"] + gp["c1"]) / 2
        t2, h2 = P.GATES["panel_t"] / 2, P.GATES["panel_h"] / 2
        zc = tb["top"] + P.GATES["gap"] + h2
        if gp["axis"] == "x":
            px, py = mid, gp["line"]
            half = (span, t2, h2)
            posts = [(gp["c0"] - 0.035, gp["line"]), (gp["c1"] + 0.035, gp["line"])]
        else:
            px, py = gp["line"], mid
            half = (t2, span, h2)
            posts = [(gp["line"], gp["c0"] - 0.035), (gp["line"], gp["c1"] + 0.035)]
        body_path = f"{ROOT}/gates/gate{zone}"
        xform = UsdGeom.Xform.Define(self.stage, body_path)
        UsdGeom.Xformable(xform.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(px, py, zc))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(xform.GetPrim())
        mass.CreateMassAttr(6.0)
        geom = UsdGeom.Cube.Define(self.stage, f"{body_path}/panel")
        geom.CreateSizeAttr(2.0)
        UsdGeom.Xformable(geom.GetPrim()).AddScaleOp().Set(Gf.Vec3f(*half))
        col = P.ROUTE_RGBA[zone]
        geom.CreateDisplayColorAttr([Gf.Vec3f(*[c * 0.85 for c in col])])
        geom.CreateDisplayOpacityAttr([0.9])
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        self._bind_phys(geom.GetPrim(), mat_gate)
        # prismatic joint to the world, axis Z, normally closed
        joint = UsdPhysics.PrismaticJoint.Define(self.stage, f"{body_path}_joint")
        joint.CreateAxisAttr("Z")
        joint.CreateLowerLimitAttr(-0.002)
        joint.CreateUpperLimitAttr(float(P.GATES["travel"]))
        joint.CreateBody1Rel().SetTargets([Sdf.Path(body_path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(px, py, zc))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(2.0e4)
        drive.CreateDampingAttr(2.0e3)
        drive.CreateMaxForceAttr(800.0)
        drive.CreateTargetPositionAttr(0.0)
        for i, (qx, qy) in enumerate(posts):     # visual frame
            top_z = tb["top"] + P.GATES["travel"] + P.GATES["panel_h"] + 0.06
            self.add_cylinder(f"gate{zone}_post{i}", (qx, qy, top_z / 2), 0.025,
                              top_z / 2, color=STEEL)
        return joint.GetPrim().GetPath().pathString, zc

    # -------------------------------------------------------------------- items
    def build_item(self, e, index, mat_item, damping=(0.0, 0.05)):
        slug = e["slug"]
        park = (0.6 + index * 0.85, -1.2, e["dims_m"][2] / 2 + 0.003)
        body_path = f"{ROOT}/items/item_{_sanitize(slug)}"
        xform = UsdGeom.Xform.Define(self.stage, body_path)
        UsdGeom.Xformable(xform.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*park))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(xform.GetPrim())
        mass.CreateMassAttr(float(e["mass_kg"]) * self.mass_mult)
        pxrb = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.GetPrim())
        # material-class damping: soft items (sack/pouf) absorb energy
        pxrb.CreateLinearDampingAttr(float(damping[0]))
        pxrb.CreateAngularDampingAttr(float(damping[1]))
        pxrb.CreateSolverPositionIterationCountAttr(8)
        # never sleep: the conveyor drive writes velocities every control tick,
        # and PhysX silently ignores velocity writes on sleeping bodies — a
        # briefly-stopped item would freeze forever (found on the crest bridge)
        pxrb.CreateSleepThresholdAttr(0.0)
        pxrb.CreateStabilizationThresholdAttr(0.0)

        pts, n = read_stl(self.repo / "cell" / "assets" / "meshes" / e["file"])
        mesh = UsdGeom.Mesh.Define(self.stage, f"{body_path}/geom")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        mesh.CreateFaceVertexIndicesAttr(
            Vt.IntArray.FromNumpy(np.arange(3 * n, dtype=np.int32)))
        mesh.CreateFaceVertexCountsAttr(
            Vt.IntArray.FromNumpy(np.full(n, 3, dtype=np.int32)))
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*ITEM_COL)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        mcoll = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        mcoll.CreateApproximationAttr("convexHull")
        pxrb.CreateEnableCCDAttr(True)           # thin/fast items tunnel-proof
        pxc = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        pxc.CreateContactOffsetAttr(0.003 if min(e["dims_m"]) < 0.02 else 0.005)
        pxc.CreateRestOffsetAttr(0.0)
        self._bind_phys(mesh.GetPrim(), mat_item)
        return body_path, park

    # ------------------------------------------------------------------ cameras
    def build_camera_lookat(self, name, pos, target, fovy_deg=45.0):
        """Camera at pos looking at target (world z-up)."""
        pos = np.asarray(pos, float)
        f = np.asarray(target, float) - pos
        f /= np.linalg.norm(f)
        z = -f                                     # USD camera looks along -Z
        x = np.cross((0.0, 0.0, 1.0), z)
        n = np.linalg.norm(x)
        if n < 1e-6:                               # looking straight down/up
            x = np.array([1.0, 0.0, 0.0])
        else:
            x /= n
        y = np.cross(z, x)
        return self.build_camera(name, tuple(pos), tuple(x), tuple(y),
                                 fovy_deg=fovy_deg)

    def build_camera(self, name, pos, xaxis, yaxis, fovy_deg=45.0):
        path = f"{ROOT}/cams/{name}"
        cam = UsdGeom.Camera.Define(self.stage, path)
        x = np.array(xaxis, float); x /= np.linalg.norm(x)
        y = np.array(yaxis, float); y /= np.linalg.norm(y)
        z = np.cross(x, y)                        # MuJoCo & USD: look along -Z
        R = np.column_stack([x, y, z])
        w, qx, qy, qz = mat_to_quat(R)
        xf = UsdGeom.Xformable(cam.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        op.Set(Gf.Quatd(w, Gf.Vec3d(qx, qy, qz)))
        cam.CreateFocalLengthAttr(15.2908 / (2 * math.tan(math.radians(fovy_deg) / 2)))
        cam.CreateHorizontalApertureAttr(15.2908 * 1280.0 / 720.0)
        cam.CreateVerticalApertureAttr(15.2908)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
        return path

    # -------------------------------------------------------------------- build
    def build(self, manifest):
        st = self.stage
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(st, 1.0)

        # lights
        dome = UsdLux.DomeLight.Define(st, f"{ROOT}/lights/dome")
        dome.CreateIntensityAttr(400.0)
        sun = UsdLux.DistantLight.Define(st, f"{ROOT}/lights/sun")
        sun.CreateIntensityAttr(2500.0)
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-35, 20, 0))

        # physics materials (chute dictates via combine=min, like MuJoCo
        # priority; the brake pad dictates via combine=max so a low-friction
        # item still brakes before the cage)
        m_belt = self.phys_material("belt", 0.80, 0.72, combine="average")
        m_item = self.phys_material("item", *P.MATERIALS["_default"][:2],
                                    restitution=P.MATERIALS["_default"][2],
                                    combine="average")
        m_chute = self.phys_material("chute", 0.40, 0.40, combine="min")
        m_pad = self.phys_material("pad", 0.45, 0.45, combine="max")
        m_mat = self.phys_material("cage_mat", 0.90, 0.90, combine="max")
        m_wall = self.phys_material("wall", 0.30, 0.30, combine="min")
        m_gate = self.phys_material("gate", 0.20, 0.20, combine="min")

        a, b, tb, cb = P.BELT_A, P.BELT_B, P.TABLE, P.CONNECT_B
        cc, cd = P.CHUTE_C, P.CHUTE_D

        # floor: dark industrial concrete (less white wash in the frame)
        self.add_box("floor", (5, 3, -0.01), (12, 8, 0.01),
                     color=(0.40, 0.42, 0.45), mat=m_belt)

        # conveyors: surface-velocity kinematic bodies (Conveyor Belt utility
        # mechanism). The transfer table splits into a fixed entry strip and a
        # switchable-vector ROUTING ZONE (ARB sorter): the zone's surface
        # velocity is commanded per active route at runtime.
        conveyors = {
            "beltA": self.make_conveyor(
                "beltA", ((a["x0"] + tb["x0"]) / 2, a["y"], a["top"] / 2),
                ((tb["x0"] - a["x0"]) / 2, a["width"] / 2, a["top"] / 2),
                velocity=(a["speed"], 0, 0), color=BELT_COL, mat=m_belt),
            "beltB": self.make_conveyor(
                "beltB", (b["cx"], (b["y0"] + b["y1"]) / 2, b["top"] / 2),
                (b["width"] / 2, (b["y1"] - b["y0"]) / 2, b["top"] / 2),
                velocity=(0, b["speed"], 0), color=BELT_COL, mat=m_belt),
            "entry": self.make_conveyor(
                "entry", ((tb["x0"] + tb["route_x"]) / 2, tb["y"], tb["top"] / 2),
                ((tb["route_x"] - tb["x0"]) / 2, tb["width"] / 2, tb["top"] / 2),
                velocity=(tb["speed"], 0, 0), color=TABLE_COL, mat=m_belt),
            "connectB": self.make_conveyor(
                "connectB", (cb["cx"], (cb["y0"] + cb["y1"]) / 2, cb["top"] / 2),
                (cb["width"] / 2, (cb["y1"] - cb["y0"]) / 2, cb["top"] / 2),
                velocity=(0, cb["speed"], 0), color=TABLE_COL, mat=m_belt),
        }
        # ARB ROUTING DECK (Priority-1 mechanism): the routing zone is a
        # matrix of LOCAL actuator patches, each its own kinematic body with
        # its own surface velocity — commanded per patch at runtime by
        # isaac/arb_deck.ArbDeck with latency/ramp/saturation/noise. Spawned
        # feeding forward (a dead strip parks straddlers).
        arb_patches = []
        for pt in P.arb_patches():
            path = self.make_conveyor(
                f"zone_r{pt['r']}c{pt['c']}",
                (pt["cx"], pt["cy"], tb["top"] / 2),
                (pt["hx"], pt["hy"], tb["top"] / 2),
                velocity=(tb["speed"], 0, 0), color=(0.15, 0.155, 0.17),
                mat=m_belt)
            arb_patches.append({**pt, "path": path})
        # powered nose-overs at both crest handoffs: inclined conveyor rollers
        # whose surface velocity PULLS the discharging item's nose down the
        # slope on contact — guided nose-first discharge. These are discharge
        # roller beds, not just crest lips: they run through the aperture so
        # the first delivered item clears the cage mouth before the next item
        # arrives. Without this, pile-up at the roll-cage entrance is a real
        # physical bottleneck for box_l and flat circular items.
        ang_c = float(np.arctan2(P.CHUTE_C["z0"] - P.CHUTE_C["z1"],
                                 P.CHUTE_C["x1"] - P.CHUTE_C["x0"]))
        s_len_c = 0.88
        ccx = tb["x1"] + s_len_c * math.cos(ang_c) / 2
        ccz = tb["top"] - s_len_c * math.sin(ang_c) / 2 - 0.012
        # local +x is the down-slope axis after the (0, ang_c, 0) rotation
        conveyors["noseC"] = self.make_conveyor(
            "noseC", (ccx, P.CHUTE_C["cy"], ccz),
            (s_len_c / 2, P.CHUTE_C["width"] / 2, 0.012),
            velocity=(tb["speed"], 0, 0),
            color=(0.48, 0.50, 0.56), mat=m_belt, euler_rad=(0, ang_c, 0))
        ang_d = float(np.arctan2(P.CHUTE_D["z0"] - P.CHUTE_D["z1"],
                                 P.CHUTE_D["y0"] - P.CHUTE_D["y1"]))
        s_len_d = 0.88
        dcy = (tb["y"] - tb["width"] / 2) - s_len_d * math.cos(ang_d) / 2
        dcz = tb["top"] - s_len_d * math.sin(ang_d) / 2 - 0.012
        # local -y is the down-slope axis after the (ang_d, 0, 0) rotation
        conveyors["noseD"] = self.make_conveyor(
            "noseD", (P.CHUTE_D["cx"], dcy, dcz),
            (P.CHUTE_D["width"] / 2, s_len_d / 2, 0.012),
            velocity=(0, -tb["speed"], 0),
            color=(0.48, 0.50, 0.56), mat=m_belt, euler_rad=(ang_d, 0, 0))
        # pop-up stop blades (physical flow discipline): escapement gate,
        # pre-gate hold, two zone-accumulation stops, table induction stop
        beltA_p = conveyors["beltA"]
        blades = {
            "egate": self.build_blade("egate", a["gate_x"] + 0.02, a["y"],
                                      a["width"] - 0.02, filter_paths=[beltA_p]),
            "hold2": self.build_blade("hold2", a["hold2_x"] + 0.02, a["y"],
                                      a["width"] - 0.02, filter_paths=[beltA_p]),
            "zoneq1": self.build_blade("zoneq1", a["hold2_x"] - 0.60, a["y"],
                                       a["width"] - 0.02, filter_paths=[beltA_p]),
            "zoneq2": self.build_blade("zoneq2", a["hold2_x"] - 1.20, a["y"],
                                       a["width"] - 0.02, filter_paths=[beltA_p]),
            "induct": self.build_blade("induct", tb["route_x"] - 0.02, tb["y"],
                                       a["width"] + 0.10,
                                       filter_paths=[conveyors["entry"]]),
        }

        # low side guides along the full belt A (real belts carry them; they
        # keep items centred without any scripted lateral velocity)
        for sgn, nm in ((1, "l"), (-1, "r")):
            self.add_box(f"beltA_guide_{nm}",
                         (3.34, a["y"] + sgn * (a["width"] / 2 + 0.015), a["top"] + 0.04),
                         (3.34, 0.015, 0.05), color=STEEL, mat=m_wall)

        # funnel rails on the last stretch of belt A
        rail_h, rail_t = 0.08, 0.015
        rail_z = a["top"] + 0.08
        self.add_box("feed_rail_l", (tb["x0"] - 0.12, a["y"] + a["width"] / 2 + 0.015,
                                     rail_z), (0.12, 0.015, 0.08), color=RAIL_COL, mat=m_wall)
        self.add_box("feed_rail_r", (tb["x0"] - 0.12, a["y"] - a["width"] / 2 - 0.015,
                                     rail_z), (0.12, 0.015, 0.08), color=RAIL_COL, mat=m_wall)

        # table edge rails, open only at the exits
        ty0, ty1 = tb["y"] - tb["width"] / 2, tb["y"] + tb["width"] / 2
        bx0, bx1 = cb["cx"] - cb["width"] / 2, cb["cx"] + cb["width"] / 2
        self.add_box("trail_n1", ((tb["x0"] + bx0) / 2, ty1 + rail_t, rail_z),
                     ((bx0 - tb["x0"]) / 2, rail_t, rail_h), color=RAIL_COL, mat=m_wall)
        if tb["x1"] > bx1:
            self.add_box("trail_n2", ((bx1 + tb["x1"]) / 2, ty1 + rail_t, rail_z),
                         ((tb["x1"] - bx1) / 2, rail_t, rail_h), color=RAIL_COL, mat=m_wall)
        dx0, dx1 = cd["cx"] - cd["width"] / 2, cd["cx"] + cd["width"] / 2
        self.add_box("trail_s1", ((tb["x0"] + dx0) / 2, ty0 - rail_t, rail_z),
                     ((dx0 - tb["x0"]) / 2, rail_t, rail_h), color=RAIL_COL, mat=m_wall)
        if tb["x1"] > dx1:
            self.add_box("trail_s2", ((dx1 + tb["x1"]) / 2, ty0 - rail_t, rail_z),
                         ((tb["x1"] - dx1) / 2, rail_t, rail_h), color=RAIL_COL, mat=m_wall)
        cy0, cy1 = cc["cy"] - cc["width"] / 2, cc["cy"] + cc["width"] / 2
        self.add_box("trail_e1", (tb["x1"] + rail_t, (ty0 + cy0) / 2, rail_z),
                     (rail_t, (cy0 - ty0) / 2, rail_h), color=RAIL_COL, mat=m_wall)
        self.add_box("trail_e2", (tb["x1"] + rail_t, (cy1 + ty1) / 2, rail_z),
                     (rail_t, (ty1 - cy1) / 2, rail_h), color=RAIL_COL, mat=m_wall)
        self.add_box("trail_w1", (tb["x0"] - rail_t, (ty0 + a["y"] - a["width"] / 2) / 2,
                                  rail_z),
                     (rail_t, (a["y"] - a["width"] / 2 - ty0) / 2, rail_h),
                     color=RAIL_COL, mat=m_wall)
        self.add_box("trail_w2", (tb["x0"] - rail_t, (a["y"] + a["width"] / 2 + ty1) / 2,
                                  rail_z),
                     (rail_t, (ty1 - a["y"] - a["width"] / 2) / 2, rail_h),
                     color=RAIL_COL, mat=m_wall)
        # connector side rails
        self.add_box("crail_l", (bx0 - rail_t, (cb["y0"] + cb["y1"]) / 2, rail_z),
                     (rail_t, (cb["y1"] - cb["y0"]) / 2, rail_h), color=RAIL_COL, mat=m_wall)
        self.add_box("crail_r", (bx1 + rail_t, (cb["y0"] + cb["y1"]) / 2, rail_z),
                     (rail_t, (cb["y1"] - cb["y0"]) / 2, rail_h), color=RAIL_COL, mat=m_wall)

        # chutes into the cages
        cages_t = P.cages_for("table")
        wall_c = cages_t["C"]["center"][0] - cages_t["C"]["inner"][0] / 2 - cages_t["C"]["wall_t"] / 2
        wall_d = cages_t["D"]["center"][1] + cages_t["D"]["inner"][1] / 2 + cages_t["D"]["wall_t"] / 2
        self.build_chute("C", cc, "x", wall_c, m_chute, m_pad, m_wall)
        self.build_chute("D", cd, "y", wall_d, m_chute, m_pad, m_wall)

        # cages with apertures
        for zone, cage in cages_t.items():
            self.build_cage(zone, cage, m_wall, m_mat)

        # exit gates (normally closed)
        gate_info = {}
        for zone in ("B", "C", "D"):
            jpath, z0 = self.build_gate(zone, m_gate)
            gate_info[zone] = {"joint": jpath, "z0": z0,
                               "body": f"{ROOT}/gates/gate{zone}"}

        # the articulated exception arm is built by isaac.arm.build_arm()

        # vision-station gantry + camera housings live in isaac/dressing.py

        # items: one physics material per slug from the material table
        # (cell/params.MATERIALS), scaled by the sweep multipliers
        item_info = {}
        fm = self.friction_mult
        for i, e in enumerate(manifest):
            sf, df, rest, ld, ad = P.MATERIALS.get(e["slug"],
                                                   P.MATERIALS["_default"])
            mat_i = self.phys_material(f"item_{_sanitize(e['slug'])}",
                                       max(0.05, sf * fm), max(0.05, df * fm),
                                       restitution=rest, combine="average")
            path, park = self.build_item(e, i, mat_i, damping=(ld, ad))
            item_info[e["slug"]] = {
                "path": path, "park": park,
                "material": {"static_friction": round(max(0.05, sf * fm), 3),
                             "dynamic_friction": round(max(0.05, df * fm), 3),
                             "restitution": rest, "linear_damping": ld,
                             "angular_damping": ad,
                             "mass_kg": round(e["mass_kg"] * self.mass_mult, 3)}}

        # cameras (from the MuJoCo presentation cameras)
        cams = {
            "overview": self.build_camera("overview", (2.4, -2.4, 4.6),
                                          (0.7006, -0.7136, 0), (0.3288, 0.3229, 0.8880)),
            "top_view": self.build_camera("top_view", (7.7, 3.0, 6.5),
                                          (1, 0, 0), (0, 1, 0)),
            "routing": self.build_camera("routing", (8.3, 0.0, 3.4),
                                         (0.9999, 0.0167, 0), (-0.0112, 0.6688, 0.7432)),
            "lookahead": self.build_camera("lookahead", (6.0, 3.0, 2.2),
                                           (1, 0, 0), (0, 1, 0)),
        }
        # DWS side profiler heads (VIRTUAL_SENSOR): the vision station is
        # overhead + two side depth heads — the section below its widest line
        # is invisible from overhead
        vs = P.VIRTUAL_SENSOR
        off, zh = vs["side_head_offset_m"], vs["side_head_z_m"]
        a_y, a_top = P.BELT_A["y"], P.BELT_A["top"]
        cams["side_a"] = self.build_camera_lookat(
            "side_a", (6.0, a_y - off, zh), (6.0, a_y, a_top + 0.10))
        cams["side_b"] = self.build_camera_lookat(
            "side_b", (6.0, a_y + off, zh), (6.0, a_y, a_top + 0.10))
        # jam-localization camera: straight-down over the routing zone +
        # chutes; background-subtraction on its depth finds a stuck item
        cams["jamcam"] = self.build_camera("jamcam", (8.3, 2.6, 3.8),
                                           (1, 0, 0), (0, 1, 0))

        # industrial presentation layer (visuals only — physics/sensor-safe)
        from isaac.dressing import dress_scene
        viz = dress_scene(st)
        return {"gates": gate_info, "items": item_info, "cams": cams,
                "conveyors": conveyors, "blades": blades, "viz": viz,
                "arb_patches": arb_patches}


def load_manifest(repo_root):
    p = Path(repo_root) / "cell" / "assets" / "manifest.json"
    return json.loads(p.read_text(encoding="utf-8"))
