# -*- coding: utf-8 -*-
"""USD scene builder for the Isaac Sim port of the SortMaster cell.

Same single source of truth as the MuJoCo build: every dimension comes from
cell/params.py (tilt-tray sorter executive). Statics are USD Cube colliders;
the sorter carriers are REAL articulated machines: each carrier body rides a
world prismatic joint with a velocity drive (the traction chain), and each
tray is a dynamic rigid body on a revolute joint with an angular position
drive (the tilt actuator). Items are rigid bodies whose collision shape is a
PhysX convex hull of the official STL surface (matching MuJoCo's
maxhullvert=64 idiom). Freight is moved by contact physics only.

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

from isaac.materials import preset as _mat_preset

STEEL = (0.55, 0.57, 0.62)
DARK = (0.22, 0.24, 0.28)
# named presentation finishes (isaac.materials.PRESETS) — visual only
BELT_COL, BELT_R, _BELT_M = _mat_preset("belt_rubber")     # matte belt rubber
RAIL_COL, RAIL_R, RAIL_M = _mat_preset("safety_yellow_worn")   # guards
CHUTE_COL, CHUTE_R, CHUTE_M = _mat_preset("brushed_steel")     # chute shells
POWDER_COL, POWDER_R, POWDER_M = _mat_preset("powder_steel_dark")
GALV_COL, GALV_R, GALV_M = _mat_preset("galvanized")
TRAY_COL = (0.32, 0.33, 0.36)          # powder-coated steel tray plate
LIP_COL = (0.21, 0.22, 0.25)           # darker steel tray lips (were gold)
SHUTTLE_COL = (0.10, 0.11, 0.13)       # carrier chassis
FRAME_COL = (0.17, 0.18, 0.21)         # sorter chassis frame
ITEM_COL = (0.75, 0.72, 0.65)


def soft(col, k=0.34, base=0.045):
    """Route/marking colour -> industrial powder-coat: darker, duller and
    desaturated toward a neutral grey-paint base (saturated primaries read
    as toy plastic under RTX)."""
    m = sum(float(v) for v in col) / 3.0
    return tuple(base + k * (0.55 * float(v) + 0.45 * m) for v in col)

ROOT = "/World"


# --------------------------------------------------------------------- helpers
def _sanitize(name):
    return name.replace("-", "_").replace(" ", "_").replace("+", "p").replace(".", "_")


def read_stl(path):
    """Binary STL -> (points Nx3 float32, n_faces)."""
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
        UsdGeom.Xform.Define(stage, f"{ROOT}/items")
        UsdGeom.Xform.Define(stage, f"{ROOT}/cams")
        UsdGeom.Xform.Define(stage, f"{ROOT}/conveyors")
        UsdGeom.Xform.Define(stage, f"{ROOT}/blades")
        UsdGeom.Xform.Define(stage, f"{ROOT}/sorter")

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

    # ------------------------------------------------------- visual materials
    def vis_material(self, color, roughness=0.72, metallic=0.12, seed=0):
        """Matte industrial PBR for statics — prims with bare displayColor
        render as glossy toy plastic under RTX."""
        jit = (0.90, 1.0, 1.08)[seed % 3]
        c = tuple(min(1.0, float(v) * jit) for v in color)
        key = ("vis", round(c[0], 3), round(c[1], 3), round(c[2], 3),
               roughness, metallic)
        if key in self.mats:
            return self.mats[key]
        path = f"{ROOT}/VisMats/m_{len(self.mats)}"
        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, f"{path}/pbr")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*c))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
            float(roughness))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
            float(metallic))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),
                                                  "surface")
        self.mats[key] = mat
        return mat

    def _ensure_pbr(self):
        """Lazily build the OmniPBR grunge library (world-triplanar dirt +
        roughness, no per-prim UVs)."""
        if getattr(self, "pbr", "unset") != "unset":
            return self.pbr
        self.pbr = None
        try:
            from isaac.materials import make_grunge_textures, PbrLibrary
            ta, tr = make_grunge_textures("/tmp/sortmaster_signs")
            self.pbr = PbrLibrary(self.stage, ROOT, tex_albedo=ta,
                                  tex_rough=tr)
        except Exception as exc:
            print(f"[materials] OmniPBR unavailable ({exc}); "
                  f"flat preview surfaces", flush=True)
        return self.pbr

    def _bind_vis(self, prim, color, opacity=1.0, roughness=0.82,
                  metallic=0.05, textured=True):
        if opacity < 1.0:
            return                       # translucent panels keep displayColor
        seed = hash(prim.GetPath().pathString) & 0x7fffffff
        pbr = self._ensure_pbr()
        if pbr is not None:
            try:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    pbr.get(color, roughness=roughness, metallic=metallic,
                            textured=textured, seed=seed))
                return
            except Exception:
                pass
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            self.vis_material(color, roughness=roughness, metallic=metallic,
                              seed=seed))

    # ------------------------------------------------------------------- solids
    def add_box(self, name, center, half, euler_rad=(0, 0, 0), color=STEEL,
                opacity=1.0, collide=True, mat=None, parent="statics",
                roughness=0.82, metallic=0.05):
        """Static box collider. roughness/metallic feed the VISUAL binding
        only (physics comes from `mat`)."""
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
        self._bind_vis(cube.GetPrim(), color, opacity, roughness=roughness,
                       metallic=metallic)
        if collide:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            # MuJoCo runs margin=0; PhysX's default ~2-4 cm contactOffset makes
            # speculative contacts fire across designed clearances. Keep tight.
            pxc = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            pxc.CreateContactOffsetAttr(0.005)
            pxc.CreateRestOffsetAttr(0.0)
            self._bind_phys(cube.GetPrim(), mat)
        return cube.GetPrim()

    # --------------------------------------------------------------- conveyors
    def make_conveyor(self, name, center, half, velocity=(0.0, 0.0, 0.0),
                      color=BELT_COL, mat=None, euler_rad=(0, 0, 0)):
        """Surface-velocity conveyor: kinematic rigid body whose contact
        friction drives whatever rests on it (RigidBodyAPI + CollisionAPI +
        PhysxSurfaceVelocityAPI). `velocity` is the desired surface speed in
        m/s along the prim's LOCAL axes; PhysX applies surfaceVelocity scaled
        by the prim's xform scale, so it is written pre-divided by the
        half-extents (measured: a belt half-length 3.45 commanded 1.0 dragged
        items at exactly 3.450 m/s)."""
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
        # matte belt-rubber visual (noses stay visible)
        self._bind_vis(prim, color, roughness=BELT_R, metallic=0.0)
        return path

    def build_blade(self, name, x, y, width, along="y", filter_paths=()):
        """Pop-up stop blade (escapement / pre-gate hold): a thin panel on a
        vertical prismatic drive, normally LOWERED flush below the belt
        surface, raised to block the flow — the physical mechanism real
        accumulation conveyors use. Returns the joint path."""
        top = P.BELT_A["top"]
        h2, t2 = 0.06, 0.008
        zc = top - h2 - 0.046
        if along == "y":
            hh = (0.038, width / 2 + 0.012, 0.014)
        else:
            hh = (width / 2 + 0.012, 0.038, 0.014)
        self.add_box(f"{name}_slot", (x, y, top - 0.006), hh,
                     color=(0.05, 0.055, 0.065), collide=False)
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
                     axis="Z", parent="statics", roughness=0.82,
                     metallic=0.05):
        """Visual-only cylinder (frame posts etc.)."""
        path = f"{ROOT}/{parent}/{_sanitize(name)}"
        cyl = UsdGeom.Cylinder.Define(self.stage, path)
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(half_h * 2))
        cyl.CreateAxisAttr(axis)
        xf = UsdGeom.Xformable(cyl.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(c) for c in center]))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        self._bind_vis(cyl.GetPrim(), color, roughness=roughness,
                       metallic=metallic)
        return cyl.GetPrim()

    # -------------------------------------------------------------------- cages
    def build_cage(self, zone, cage, mat_wall, mat_mat, tag=None):
        """Aperture-walled containment: the open side carries a low sill
        (skirt), two flanks and (if the aperture stops below the wall top) a
        header. The discharge chute crosses the wall plane INSIDE the
        aperture, so a routed item can roll or bounce inside the cage but
        cannot leave it again.

        PRESENTATION: the collider walls keep their exact geometry but
        render as NEAR-INVISIBLE dark glass (displayColor dark + low
        opacity) — the visible cage is the non-colliding galvanized
        roll-cage shell from dressing.cages_detail(), so the destinations
        read as wire-mesh roll containers instead of solid plastic boxes."""
        tag = tag or f"cage{zone}"
        cx, cy = cage["center"]
        ix, iy = cage["inner"]
        t, h = cage["wall_t"], cage["wall_h"]
        hx, hy, hh = ix / 2, iy / 2, h / 2
        col = (0.09, 0.10, 0.12)         # ghost wall tint (was route colour)
        ghost = 0.10                     # near-invisible collider walls
        self.add_box(f"{tag}_floor", (cx, cy, t / 2), (hx + t, hy + t, t / 2),
                     color=(0.13, 0.14, 0.16), mat=mat_wall)
        walls = {"+y": (0, hy + t / 2, hx + t, t / 2), "-y": (0, -(hy + t / 2), hx + t, t / 2),
                 "+x": (hx + t / 2, 0, t / 2, hy), "-x": (-(hx + t / 2), 0, t / 2, hy)}
        open_side = cage.get("open_side")
        for side, (dx, dy, sx, sy) in walls.items():
            sname = side.replace("+", "p").replace("-", "m")
            if side == open_side:
                aw2 = cage["aperture_w"] / 2
                top = cage["aperture_top"]
                sill_top = cage.get("sill_top", 0.33)
                skirt_h2 = (sill_top - t) / 2
                skirt_zc = (sill_top + t) / 2
                hdr_h = (t + h - top) / 2
                if side in ("+x", "-x"):
                    for sgn, nm in ((1, "a"), (-1, "b")):
                        fl = (sy - aw2) / 2
                        self.add_box(f"{tag}_fl{nm}",
                                     (cx + dx, cy + sgn * (aw2 + fl), t + hh),
                                     (sx, fl, hh), color=col, mat=mat_wall,
                                     opacity=ghost)
                    if hdr_h > 0.005:
                        self.add_box(f"{tag}_hdr", (cx + dx, cy, top + hdr_h),
                                     (sx, aw2, hdr_h), color=col,
                                     mat=mat_wall, opacity=ghost)
                    self.add_box(f"{tag}_skirt", (cx + dx, cy, skirt_zc),
                                 (sx, aw2, skirt_h2), color=col,
                                 mat=mat_wall, opacity=ghost)
                else:
                    for sgn, nm in ((1, "a"), (-1, "b")):
                        fl = (sx - aw2) / 2
                        self.add_box(f"{tag}_fl{nm}",
                                     (cx + sgn * (aw2 + fl), cy + dy, t + hh),
                                     (fl, sy, hh), color=col, mat=mat_wall,
                                     opacity=ghost)
                    if hdr_h > 0.005:
                        self.add_box(f"{tag}_hdr", (cx, cy + dy, top + hdr_h),
                                     (aw2, sy, hdr_h), color=col,
                                     mat=mat_wall, opacity=ghost)
                    self.add_box(f"{tag}_skirt", (cx, cy + dy, skirt_zc),
                                 (aw2, sy, skirt_h2), color=col,
                                 mat=mat_wall, opacity=ghost)
            else:
                self.add_box(f"{tag}_w{sname}", (cx + dx, cy + dy, t + hh),
                             (sx, sy, hh), color=col, mat=mat_wall,
                             opacity=ghost)
        if open_side:
            self.add_box(f"{tag}_mat", (cx, cy, t + 0.004), (hx, hy, 0.004),
                         color=(0.15, 0.15, 0.17), mat=mat_mat)
        for sx_ in (-1, 1):     # visual corner posts — galvanized tube frame
            for sy_ in (-1, 1):
                self.add_cylinder(f"{tag}_p{sx_}{sy_}",
                                  (cx + sx_ * (hx + t), cy + sy_ * (hy + t), (t + h) / 2),
                                  0.022, (t + h) / 2, color=GALV_COL,
                                  roughness=GALV_R, metallic=GALV_M)

    # -------------------------------------------------------------------- chute
    def build_chute(self, zone, cc, wall_at, mat_chute, mat_pad, mat_hood):
        """32-deg gravity brake chute along the y axis. cc["dir"] = -1 runs
        south (C/D cages), +1 runs north (review pen). Starts just below the
        tilted tray lip (z0 ~0.43) and passes through the destination's wall
        aperture onto a high-friction brake pad inside."""
        d = float(cc["dir"])
        y0, z0, z1 = cc["y0"], cc["z0"], cc["z1"]
        y1, pad_end = P.chute_run(cc)
        cx = cc["cx"]
        length = float(np.hypot(y1 - y0, z0 - z1))
        # rotX(+32) raises the +y side: a south-running chute descends with
        # +32, a north-running one with -32
        ang = -d * math.radians(32.0)
        euler = (ang, 0, 0)
        mid, zmid = (y0 + y1) / 2, (z0 + z1) / 2 - 0.015
        self.add_box(f"chute{zone}", (cx, mid, zmid),
                     (cc["width"] / 2, length / 2, 0.015), euler, CHUTE_COL,
                     mat=mat_chute, roughness=CHUTE_R, metallic=CHUTE_M)
        # MOUTH CHAMFER: steep infill strip at the mouth edge (see
        # params.CHUTE) — thin flat freight lands FLAT instead of tipping
        # onto its rim over the 50 mm deep-drop, and the ballistic under-fly
        # window narrows. Top edge clears the tilted-lip tip trace.
        ch_rise, ch_run = cc["chamfer_rise"], cc["chamfer_run"]
        ch_top_y = y0 - d * cc["chamfer_top_inset"]
        ch_base_y = ch_top_y + d * ch_run
        ch_ang = -d * math.atan2(ch_rise + 0.003, ch_run)
        ch_len = math.hypot(ch_run, ch_rise + 0.003)
        self.add_box(f"chute{zone}_chamfer",
                     (cx, (ch_top_y + ch_base_y) / 2,
                      z0 + (ch_rise - 0.003) / 2),
                     (cc["width"] / 2, ch_len / 2, 0.002),
                     (ch_ang, 0, 0), CHUTE_COL, mat=mat_chute,
                     roughness=CHUTE_R, metallic=CHUTE_M)
        # solid filler under the thin working face (twin parity: an item
        # edge that penetrates the shell meets backing, not a wedge cavity)
        fill_y0 = ch_top_y + d * 0.004
        fill_y1 = ch_base_y - d * 0.002
        self.add_box(f"chute{zone}_chamfill",
                     (cx, (fill_y0 + fill_y1) / 2, z0 - 0.017),
                     (cc["width"] / 2, abs(fill_y1 - fill_y0) / 2, 0.014),
                     color=CHUTE_COL, mat=mat_chute, roughness=CHUTE_R,
                     metallic=CHUTE_M)
        # MOUTH CHEEKS: side wings over the throat gap (see params.CHUTE) —
        # a hot small roller with +x drift crossed the open gap sides before
        # touching any chute surface. Top capped by the plate-edge arc.
        ck_t = 0.015
        ck_y0 = y0 + d * cc["cheek_inset"]
        ck_y1 = ck_y0 + d * cc["cheek_len"]
        ck_cy = (ck_y0 + ck_y1) / 2
        for sgn, nm in ((1, "l"), (-1, "r")):
            off = sgn * (cc["width"] / 2 + ck_t)
            self.add_box(f"chute{zone}_cheek_{nm}",
                         (cx + off, ck_cy, z0 + cc["cheek_h"] - 0.034),
                         (ck_t, cc["cheek_len"] / 2, 0.034),
                         color=soft(P.ROUTE_RGBA.get(zone, (0.6, 0.6, 0.6))),
                         mat=mat_chute)
        # side rails to the destination wall plane. SWEEP-CORRIDOR RULE: the
        # tilting tray edge sweeps y 2.69..3.31 down to z 0.44 — the rails
        # start 0.12 m down-slope so their upper tips stay >= 8 cm below the
        # sweep envelope (smoke6: a rail tip at the chute mouth pinned a
        # discharging tray at 13 deg and flicked the item off the far side).
        # The first 0.12 m is the discharge AIRBORNE zone anyway.
        rail_t = 0.015
        # 0.09: rails reach y0-0.09 = 2.66 — 77 mm clear of the tilted lip
        # tip's southmost reach (y 2.737). The old 40 mm STUB rails over the
        # first 0.12 m are deleted: the clearance audit showed them grazing
        # the fully-tilted plate (-3.5 mm AABB at 38.8 deg); their lateral-
        # containment job at the throat belongs to the mouth CHEEKS now.
        setback = 0.09
        col = soft(P.ROUTE_RGBA.get(zone, (0.6, 0.6, 0.6)))
        r0 = y0 + d * setback
        rmid = (r0 + wall_at) / 2
        rlen = float(np.hypot(wall_at - r0, (abs(wall_at - r0)) *
                              math.tan(math.radians(32.0))))
        tanang = math.tan(math.radians(32.0))
        for sgn, nm in ((1, "l"), (-1, "r")):
            off = sgn * (cc["width"] / 2 + rail_t)
            rz = z0 - abs(rmid - y0) * tanang + P.GUIDE_H / 2
            self.add_box(f"chute{zone}_rail_{nm}", (cx + off, rmid, rz),
                         (rail_t, rlen / 2, P.GUIDE_H / 2), euler, col,
                         mat=mat_chute)
        # brake pad (inside the destination)
        pmid = (y1 + pad_end) / 2
        plen = abs(pad_end - y1)
        self.add_box(f"chute{zone}_pad", (cx, pmid, z1 - 0.020),
                     (cc["width"] / 2, plen / 2, 0.015),
                     color=(0.30, 0.31, 0.35), mat=mat_pad)
        # NO HOOD: with the deep-drop chute the aperture is crossed at
        # z ~0.22 (sill 0.20, header 0.83) — the old fly-out window the hood
        # guarded no longer exists, and an open chute keeps every jam point
        # vertically extractable by the exception arm.

    # ------------------------------------------------------- B incline connector
    def build_b_connector(self, mat_belt, mat_wall):
        """Powered incline belt from the B-station tray lip up to the FIXED
        main-sorter belt B infeed. Every B item is non-round by rule and
        holds on the 16.6-deg incline by friction."""
        bc = P.B_CONNECT
        ang = math.atan2(bc["z_top1"] - bc["z_top0"], bc["y1"] - bc["y0"])
        length = float(np.hypot(bc["y1"] - bc["y0"], bc["z_top1"] - bc["z_top0"]))
        cy = (bc["y0"] + bc["y1"]) / 2
        cz = (bc["z_top0"] + bc["z_top1"]) / 2 - 0.015
        path = self.make_conveyor(
            "bconnect", (bc["cx"], cy, cz),
            (bc["width"] / 2, length / 2, 0.015),
            velocity=(0, bc["speed"], 0), color=BELT_COL, mat=mat_belt,
            euler_rad=(ang, 0, 0))
        # side rails absorb the +x drift the item keeps from the moving tray.
        # SWEEP-CORRIDOR RULE: no rail south of y 3.45 — the tilting tray's
        # north edge sweeps to y 3.31/z 0.62 and a rail tip there pinned the
        # B-station tray at 6 deg (smoke6). The first 0.19 m of the incline
        # is the airborne landing zone.
        rail_t = 0.015
        r_y0 = 3.45
        r_cy = (r_y0 + bc["y1"]) / 2
        r_len = float(np.hypot(bc["y1"] - r_y0,
                               (bc["y1"] - r_y0) * math.tan(ang)))
        frac = (r_cy - bc["y0"]) / (bc["y1"] - bc["y0"])
        r_cz = bc["z_top0"] + frac * (bc["z_top1"] - bc["z_top0"])
        for sgn, nm in ((1, "l"), (-1, "r")):
            self.add_box(f"bconnect_rail_{nm}",
                         (bc["cx"] + sgn * (bc["width"] / 2 + rail_t), r_cy,
                          r_cz + 0.09), (rail_t, r_len / 2, 0.06),
                         (ang, 0, 0), color=RAIL_COL, mat=mat_wall,
                         roughness=RAIL_R, metallic=RAIL_M)
        return path

    # --------------------------------------------------------- tilt-tray train
    def build_carrier(self, i, x0, z_shuttle, mat_tray):
        """One sorter carrier: a chassis body on a world PRISMATIC joint with
        a velocity drive (the traction chain axis), carrying a dished tray on
        a REVOLUTE joint with an angular position drive (the tilt actuator).
        z_shuttle: the shuttle centre height for this carrier's starting leg
        (top run or under-deck return). Returns the paths."""
        S = P.SORTER
        y = S["y"]
        base = f"{ROOT}/sorter/car{i}"
        # ---- shuttle (chain follower) — KINEMATIC body, position-controlled
        # by the chain controller every physics step (a real sorter chain is
        # speed-servo'd; per-carrier position comes from the chain encoder).
        # Runtime joint-frame rewrites are not applied by PhysX, so the
        # top/return leg change must be a kinematic pose, never a joint edit.
        xform = UsdGeom.Xform.Define(self.stage, base)
        UsdGeom.Xformable(xform.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(float(x0), y, float(z_shuttle)))
        rb = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        rb.CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr(14.0)
        pxrb = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.GetPrim())
        pxrb.CreateSleepThresholdAttr(0.0)
        body = UsdGeom.Cube.Define(self.stage, f"{base}/body")
        body.CreateSizeAttr(2.0)
        UsdGeom.Xformable(body.GetPrim()).AddScaleOp().Set(
            Gf.Vec3f(0.27, 0.27, 0.025))
        body.CreateDisplayColorAttr([Gf.Vec3f(*SHUTTLE_COL)])
        self._bind_vis(body.GetPrim(), SHUTTLE_COL)
        # visual chain link block under the body
        link = UsdGeom.Cube.Define(self.stage, f"{base}/link")
        link.CreateSizeAttr(2.0)
        lxf = UsdGeom.Xformable(link.GetPrim())
        lxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
        lxf.AddScaleOp().Set(Gf.Vec3f(0.10, 0.06, 0.03))
        link.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.31, 0.34)])
        # ---- tray — dynamic body on revolute joint (tilt axis along travel)
        pivot_dz = S["pivot_z"] - (S["shuttle_top"] - 0.025)  # pivot above shuttle

        # ---- carrier readability (VISUAL ONLY, non-colliding children of
        # the kinematic shuttle — same discipline as the chain-link block):
        # dark inset end bands reveal the carrier-to-carrier gap, top-edge
        # bevel strips, the tilt PIVOT SHAFT with bearing blocks at the
        # pivot line, and the tilt-actuator housing on the south flank.
        def _chassis_box(nm, tr, half, col, euler=None, rough=0.70, met=0.10):
            c = UsdGeom.Cube.Define(self.stage, f"{base}/{nm}")
            c.CreateSizeAttr(2.0)
            cxf = UsdGeom.Xformable(c.GetPrim())
            cxf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in tr]))
            if euler is not None:
                cxf.AddRotateXYZOp().Set(Gf.Vec3f(*[float(e) for e in euler]))
            cxf.AddScaleOp().Set(Gf.Vec3f(*[float(h) for h in half]))
            c.CreateDisplayColorAttr([Gf.Vec3f(*col)])
            self._bind_vis(c.GetPrim(), col, roughness=rough, metallic=met)
            return c

        for sgn, enm in ((1, "e"), (-1, "w")):
            _chassis_box(f"band_{enm}", (sgn * 0.258, 0, 0),
                         (0.012, 0.272, 0.027), (0.030, 0.032, 0.036))
            _chassis_box(f"bevel_{enm}", (sgn * 0.27, 0, 0.025),
                         (0.006, 0.27, 0.006), POWDER_COL, euler=(0, 45, 0),
                         rough=POWDER_R, met=POWDER_M)
        # pivot shaft (brushed) just under the dished plates: r 14 mm keeps
        # the full ±38-deg plate sweep clear (plate bottom passes z -0.013
        # rel pivot at |y| 0.028; the shaft top stays at 0.000)
        shaft = UsdGeom.Cylinder.Define(self.stage, f"{base}/pivot_shaft")
        shaft.CreateRadiusAttr(0.014)
        shaft.CreateHeightAttr(float(S["tray_l"] - 0.06))
        shaft.CreateAxisAttr("X")
        shxf = UsdGeom.Xformable(shaft.GetPrim())
        shxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, float(pivot_dz) - 0.014))
        shaft.CreateDisplayColorAttr([Gf.Vec3f(*CHUTE_COL)])
        self._bind_vis(shaft.GetPrim(), CHUTE_COL, roughness=CHUTE_R,
                       metallic=CHUTE_M)
        for sgn, enm in ((1, "e"), (-1, "w")):   # bearing blocks, shaft ends
            _chassis_box(f"bearing_{enm}",
                         (sgn * 0.250, 0, pivot_dz - 0.030),
                         (0.020, 0.028, 0.012), POWDER_COL,
                         rough=POWDER_R, met=POWDER_M)
        # tilt-actuator housing + rod, SOUTH flank: the tilted lip trace
        # bottoms at y -0.244 / z 0.426 world — the housing (y -0.27..-0.31,
        # bottom z 0.532 on the top run) stays >= 0.10 m above/behind the
        # discharge fall line and outside the tilted-plate envelope.
        _chassis_box("actuator", (0, -0.29, 0.010), (0.055, 0.020, 0.035),
                     POWDER_COL, rough=POWDER_R, met=POWDER_M)
        _chassis_box("actuator_rod", (0, -0.270, 0.030),
                     (0.008, 0.008, 0.020), CHUTE_COL, euler=(30, 0, 0),
                     rough=CHUTE_R, met=CHUTE_M)

        tray = f"{ROOT}/sorter/tray{i}"
        txform = UsdGeom.Xform.Define(self.stage, tray)
        UsdGeom.Xformable(txform.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(float(x0), y, float(z_shuttle) + pivot_dz))
        UsdPhysics.RigidBodyAPI.Apply(txform.GetPrim())
        UsdPhysics.MassAPI.Apply(txform.GetPrim()).CreateMassAttr(6.0)
        tprb = PhysxSchema.PhysxRigidBodyAPI.Apply(txform.GetPrim())
        tprb.CreateSleepThresholdAttr(0.0)
        tprb.CreateSolverPositionIterationCountAttr(8)
        dish = math.radians(S["dish_deg"])
        hy = S["tray_w"] / 4 / math.cos(dish)      # half plate width
        hz = S["tray_t"] / 2
        z_in = S["tray_top"] - S["pivot_z"]        # surface at the centre line
        for sgn, nm in ((1, "n"), (-1, "s")):
            plate = UsdGeom.Cube.Define(self.stage, f"{tray}/plate_{nm}")
            plate.CreateSizeAttr(2.0)
            pxf = UsdGeom.Xformable(plate.GetPrim())
            cyy = sgn * (S["tray_w"] / 4)
            czz = z_in - hz + (S["tray_w"] / 4) * math.tan(dish)
            pxf.AddTranslateOp().Set(Gf.Vec3d(0, cyy, czz))
            pxf.AddRotateXYZOp().Set(Gf.Vec3f(sgn * S["dish_deg"], 0, 0))
            pxf.AddScaleOp().Set(Gf.Vec3f(S["tray_l"] / 2, hy, hz))
            plate.CreateDisplayColorAttr([Gf.Vec3f(*TRAY_COL)])
            # powder-coated steel plate (mid-grey, roughness ~0.5)
            self._bind_vis(plate.GetPrim(), TRAY_COL, roughness=0.50,
                           metallic=0.20)
            UsdPhysics.CollisionAPI.Apply(plate.GetPrim())
            pxc = PhysxSchema.PhysxCollisionAPI.Apply(plate.GetPrim())
            pxc.CreateContactOffsetAttr(0.004)
            pxc.CreateRestOffsetAttr(0.0)
            self._bind_phys(plate.GetPrim(), mat_tray)
        # low end fences on the +-x edges only (rolling containment during
        # carry; never in the +-y discharge path)
        for sgn, nm in ((1, "e"), (-1, "w")):
            lip = UsdGeom.Cube.Define(self.stage, f"{tray}/lip_{nm}")
            lip.CreateSizeAttr(2.0)
            lxf2 = UsdGeom.Xformable(lip.GetPrim())
            lxf2.AddTranslateOp().Set(Gf.Vec3d(
                sgn * (S["tray_l"] / 2 - S["lip_t"] / 2), 0,
                z_in + S["lip_h"] / 2))
            lxf2.AddScaleOp().Set(Gf.Vec3f(S["lip_t"] / 2,
                                           S.get("lip_w", S["tray_w"]) / 2,
                                           S["lip_h"] / 2))
            # darker steel lips — part of the tray, not gold trim
            lip.CreateDisplayColorAttr([Gf.Vec3f(*LIP_COL)])
            self._bind_vis(lip.GetPrim(), LIP_COL, roughness=0.50,
                           metallic=0.30)
            UsdPhysics.CollisionAPI.Apply(lip.GetPrim())
            pxc = PhysxSchema.PhysxCollisionAPI.Apply(lip.GetPrim())
            pxc.CreateContactOffsetAttr(0.004)
            pxc.CreateRestOffsetAttr(0.0)
            self._bind_phys(lip.GetPrim(), mat_tray)
            # thin visual 45-deg chamfer strip along the lip top (no
            # CollisionAPI — pure render geometry riding with the tray)
            chf = UsdGeom.Cube.Define(self.stage, f"{tray}/lipch_{nm}")
            chf.CreateSizeAttr(2.0)
            cxf3 = UsdGeom.Xformable(chf.GetPrim())
            cxf3.AddTranslateOp().Set(Gf.Vec3d(
                sgn * (S["tray_l"] / 2 - S["lip_t"] / 2), 0,
                z_in + S["lip_h"]))
            cxf3.AddRotateXYZOp().Set(Gf.Vec3f(0, 45, 0))
            cxf3.AddScaleOp().Set(Gf.Vec3f(
                0.005, S.get("lip_w", S["tray_w"]) / 2 - 0.004, 0.005))
            chf.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.31, 0.34)])
            self._bind_vis(chf.GetPrim(), (0.30, 0.31, 0.34), roughness=0.45,
                           metallic=0.40)
        # revolute tilt joint shuttle -> tray, axis X, angular position drive
        rj = UsdPhysics.RevoluteJoint.Define(self.stage, f"{tray}_tilt")
        rj.CreateAxisAttr("X")
        rj.CreateLowerLimitAttr(-(P.SORTER["tilt_deg"] + 8.0))
        rj.CreateUpperLimitAttr(P.SORTER["tilt_deg"] + 8.0)
        rj.CreateBody0Rel().SetTargets([Sdf.Path(base)])
        rj.CreateBody1Rel().SetTargets([Sdf.Path(tray)])
        rj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, float(pivot_dz)))
        rj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        # NOTE units gotcha: omni.physx angular-drive stiffness/damping act on
        # the DEGREE error when the target is in degrees. The values are
        # tuned so the drive is stiff either way and maxForce (N*m) is the
        # honest saturation; verified empirically with the --probe trace.
        rdrive = UsdPhysics.DriveAPI.Apply(rj.GetPrim(), "angular")
        rdrive.CreateTypeAttr("force")
        rdrive.CreateStiffnessAttr(float(S["drive_stiffness"]))
        rdrive.CreateDampingAttr(float(S["drive_damping"]))
        rdrive.CreateMaxForceAttr(float(S["drive_max_torque"]))
        rdrive.CreateTargetPositionAttr(0.0)
        return {
            "shuttle": base, "tray": tray,
            "tilt_joint": rj.GetPrim().GetPath().pathString,
        }

    def build_sorter_train(self, mat_tray):
        """The full linear sorter: carriers spread along the loop, chassis
        frame with station cutouts, and the two end modules that enclose the
        wrap (the return leg runs under the deck at true return time)."""
        S = P.SORTER
        y = S["y"]
        L_top = S["x_east"] - S["x_west"]          # visible top run
        loop_len = 2 * L_top                       # top + return
        n = S["n_carriers"]
        z_top = S["shuttle_top"] - 0.025
        z_ret = S["return_z"] - 0.05
        carriers = []
        for i in range(n):
            s = (i * S["pitch"]) % loop_len
            if s < L_top:                          # top run
                x0, z0, leg = S["x_west"] + s, z_top, "top"
            else:                                  # under-deck return run
                x0, z0, leg = S["x_east"] - (s - L_top), z_ret, "return"
            car = self.build_carrier(i, x0, z0, mat_tray)
            car["leg0"] = leg
            car["s0"] = s
            carriers.append(car)
        # chassis side beams with cutouts at the discharge stations
        cuts_s = [(P.STATIONS[k]["x"] - 0.36, P.STATIONS[k]["x"] + 0.36)
                  for k in ("C", "D")]
        cuts_n = [(P.STATIONS[k]["x"] - 0.36, P.STATIONS[k]["x"] + 0.36)
                  for k in ("B", "REVIEW")]

        def segments(x0, x1, cuts):
            segs, cur = [], x0
            for c0, c1 in sorted(cuts):
                if c0 > cur:
                    segs.append((cur, min(c0, x1)))
                cur = max(cur, c1)
            if cur < x1:
                segs.append((cur, x1))
            return [(a, b) for a, b in segs if b - a > 0.05]

        for sgn, nm, cuts in ((-1, "s", cuts_s), (1, "n", cuts_n)):
            by = y + sgn * 0.38
            for k, (a, b) in enumerate(segments(S["x_west"] + 0.02,
                                                S["x_east"] - 0.02, cuts)):
                self.add_box(f"train_beam_{nm}{k}", ((a + b) / 2, by, 0.55),
                             ((b - a) / 2, 0.03, 0.03), color=FRAME_COL,
                             collide=False)
                self.add_box(f"train_skirt_{nm}{k}", ((a + b) / 2, by, 0.47),
                             ((b - a) / 2, 0.012, 0.05), color=DARK,
                             collide=False)
                # legs at segment ends
                for lx in (a + 0.06, b - 0.06):
                    self.add_box(f"train_leg_{nm}{k}_{lx:.2f}",
                                 (lx, by, 0.26), (0.025, 0.025, 0.26),
                                 color=FRAME_COL, collide=False)
        # UNDER-DECK RETURN ENCLOSURE (visual only, collide=False): long side
        # skirt panels close the black void where the return leg runs, so
        # the under-sorter volume reads as machine enclosure. Panels sit
        # OUTSIDE y 2.62..3.38 (never inside the tray sweep or a discharge
        # fall corridor), span z 0.10..0.38, and are SEGMENTED around every
        # station discharge cutout (C/D south, B/REVIEW north) exactly like
        # the chassis beams above. Access-panel seams + vent grilles give
        # the long runs service detail.
        enc_x0, enc_x1 = S["x_west"] + 0.2, S["x_east"] - 0.2
        enc_zc, enc_zh = 0.24, 0.14                     # z 0.10..0.38
        for sgn, nm, cuts in ((-1, "s", cuts_s), (1, "n", cuts_n)):
            ey = y + sgn * 0.415
            for k, (a, b) in enumerate(segments(enc_x0, enc_x1, cuts)):
                self.add_box(f"train_enc_{nm}{k}", ((a + b) / 2, ey, enc_zc),
                             ((b - a) / 2, 0.012, enc_zh),
                             color=(0.14, 0.15, 0.18), collide=False,
                             roughness=POWDER_R, metallic=POWDER_M)
                if b - a > 0.35:        # recessed access-panel seams
                    for fx in (0.25, 0.50, 0.75):
                        self.add_box(f"train_encseam_{nm}{k}_{int(fx * 100)}",
                                     (a + fx * (b - a), ey + sgn * 0.0125,
                                      enc_zc),
                                     (0.0015, 0.0015, enc_zh - 0.015),
                                     color=(0.05, 0.055, 0.06), collide=False)
                if b - a > 0.5:         # small vent grille (3 louvre slots)
                    gx_ = (a + b) / 2 - 0.10
                    for gz in (0.185, 0.215, 0.245):
                        self.add_box(
                            f"train_encvent_{nm}{k}_{int(gz * 1000)}",
                            (gx_, ey + sgn * 0.012, gz),
                            (0.075, 0.002, 0.006),
                            color=(0.06, 0.065, 0.07), collide=False)
        # debris CATCH PAN under the top run: anything that slips through an
        # inter-tray gap (sub-3 mm freight arrives unmetered under the
        # escapement blade) lands here and the watchdog raises an operator
        # call-out — never a floor spill. Return leg passes underneath.
        self.add_box("train_pan", ((S["pan_x0"] + S["pan_x1"]) / 2, y,
                                   S["pan_z_top"] - 0.005),
                     ((S["pan_x1"] - S["pan_x0"]) / 2, S["pan_y_half"], 0.005),
                     color=(0.13, 0.14, 0.16), mat=mat_tray)
        # end modules (enclose the end wheels; the wrap teleports happen
        # inside them). Visual shells only.
        self.add_box("train_end_e", (9.57, y, 0.46), (0.15, 0.40, 0.28),
                     color=FRAME_COL, collide=False)
        self.add_box("train_end_e_cap", (9.57, y, 0.76), (0.15, 0.40, 0.02),
                     color=DARK, collide=False)
        self.add_box("train_end_w", ((6.44 + 6.74) / 2, y, 0.40),
                     (0.15, 0.40, 0.21), color=FRAME_COL, collide=False)
        return carriers

    # ------------------------------------------------------- station dressing
    def build_station(self, zone, mat_wall):
        """Discharge-station furniture: portal posts + route-colour beacon on
        the discharge side. Visual only (the mechanism is the tray)."""
        st = P.STATIONS[zone]
        x, side = st["x"], st["side"]
        col = P.ROUTE_RGBA.get(zone, (0.7, 0.7, 0.7))
        by = P.SORTER["y"] + side * 0.46
        for sgn in (-1, 1):
            self.add_box(f"st{zone}_post{sgn}", (x + sgn * 0.30, by, 0.50),
                         (0.02, 0.02, 0.50), color=FRAME_COL, collide=False)
        self.add_box(f"st{zone}_beam", (x, by, 1.02), (0.32, 0.02, 0.02),
                     color=FRAME_COL, collide=False)
        # route beacon HOUSED in a dark bezel channel (a bare colour block
        # sitting on the beam read as a floating fragment on video): bottom
        # tray + end caps + top visor, lamp face recessed 5 mm inside
        self.add_box(f"st{zone}_lampbez_b", (x, by, 1.048),
                     (0.115, 0.030, 0.008), color=(0.05, 0.055, 0.065),
                     collide=False)
        self.add_box(f"st{zone}_lampbez_t", (x, by, 1.112),
                     (0.115, 0.030, 0.006), color=(0.05, 0.055, 0.065),
                     collide=False)
        for sgn2 in (-1, 1):
            self.add_box(f"st{zone}_lampbez_{'ew'[sgn2 > 0]}",
                         (x + sgn2 * 0.108, by, 1.08),
                         (0.008, 0.030, 0.030), color=(0.05, 0.055, 0.065),
                         collide=False)
        self.add_box(f"st{zone}_lamp", (x, by, 1.08), (0.10, 0.025, 0.025),
                     color=col, collide=False)

    # -------------------------------------------------------------------- items
    def build_item(self, e, index, mat_item, damping=(0.0, 0.05)):
        slug = e["slug"]
        # pre-spawn staging: park BEHIND the east backdrop wall so waiting
        # items are occluded from every presentation camera.
        park = (11.5, 1.3 + index * 0.42, e["dims_m"][2] / 2 + 0.003)
        body_path = f"{ROOT}/items/item_{_sanitize(slug)}"
        xform = UsdGeom.Xform.Define(self.stage, body_path)
        UsdGeom.Xformable(xform.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*park))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(xform.GetPrim())
        mass.CreateMassAttr(float(e["mass_kg"]) * self.mass_mult)
        pxrb = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.GetPrim())
        pxrb.CreateLinearDampingAttr(float(damping[0]))
        pxrb.CreateAngularDampingAttr(float(damping[1]))
        pxrb.CreateSolverPositionIterationCountAttr(8)
        # never sleep: PhysX silently ignores contact velocity on sleeping
        # bodies — a briefly-stopped item would freeze forever
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
        pxrb.CreateEnableCCDAttr(True)           # thin/fast items tunnel-proof
        # PRIMITIVE COLLIDER (manifest "collider"): the synthetic edge items
        # ARE axis-aligned primitives — a native box collider avoids the
        # mesh-hull contact path, whose offset cushion anchored a 5 g rod on
        # the 32-deg slope (PhysX small-mesh stiction; the rod "floated"
        # 7 mm high on contact offsets, statically held against mu 0.28).
        if e.get("collider") == "box":
            coll = UsdGeom.Cube.Define(self.stage, f"{body_path}/coll")
            coll.CreateSizeAttr(1.0)
            UsdGeom.Xformable(coll.GetPrim()).AddScaleOp().Set(
                Gf.Vec3f(*[float(d) for d in e["dims_m"]]))
            UsdGeom.Imageable(coll.GetPrim()).MakeInvisible()
            coll_prim = coll.GetPrim()
        else:
            coll_prim = mesh.GetPrim()
            mcoll = UsdPhysics.MeshCollisionAPI.Apply(coll_prim)
            mcoll.CreateApproximationAttr("convexHull")
        UsdPhysics.CollisionAPI.Apply(coll_prim)
        pxc = PhysxSchema.PhysxCollisionAPI.Apply(coll_prim)
        small = min(e["dims_m"]) < 0.02
        pxc.CreateContactOffsetAttr(0.0015 if small else 0.005)
        pxc.CreateRestOffsetAttr(0.0)
        self._bind_phys(coll_prim, mat_item)
        return body_path, park

    # ------------------------------------------------------------------ cameras
    def build_camera_lookat(self, name, pos, target, fovy_deg=45.0):
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

        # physics materials. Tray + chute dictate via combine=min (guaranteed
        # discharge slide even for the mu=0.95 sack); the brake pad and cage
        # mat dictate via combine=max (guaranteed braking).
        m_belt = self.phys_material("belt", 0.80, 0.72, combine="average")
        m_item = self.phys_material("item", *P.MATERIALS["_default"][:2],
                                    restitution=P.MATERIALS["_default"][2],
                                    combine="average")
        m_tray = self.phys_material("tray", *P.SORTER["tray_mu"], combine="min")
        mu_chute = float(P.CHUTE["friction"].split()[0])
        m_chute = self.phys_material("chute", mu_chute, mu_chute,
                                     combine="min")
        m_pad = self.phys_material("pad", 0.45, 0.45, combine="max")
        m_mat = self.phys_material("cage_mat", 0.90, 0.90, combine="max")
        m_wall = self.phys_material("wall", 0.30, 0.30, combine="min")

        a, b = P.BELT_A, P.BELT_B

        # floor: dark industrial concrete
        self.add_box("floor", (5, 3, -0.01), (12, 8, 0.01),
                     color=(0.28, 0.30, 0.33), mat=m_belt)

        # ---- conveyor A: main body + knife-edge nose section the trays run
        # UNDER (15 mm cantilevered nose — real small-item transfers use
        # exactly this so the handoff gap is zero)
        conveyors = {
            "beltA": self.make_conveyor(
                "beltA", ((a["x0"] + a["knife_x0"]) / 2, a["y"], a["top"] / 2),
                ((a["knife_x0"] - a["x0"]) / 2, a["width"] / 2, a["top"] / 2),
                velocity=(a["speed"], 0, 0), color=BELT_COL, mat=m_belt),
            "knife": self.make_conveyor(
                "knife", ((a["knife_x0"] + a["nose_x"]) / 2, a["y"],
                          a["top"] - a["knife_t"] / 2),
                ((a["nose_x"] - a["knife_x0"]) / 2, a["width"] / 2,
                 a["knife_t"] / 2),
                velocity=(a["speed"], 0, 0), color=BELT_COL, mat=m_belt),
            "beltB": self.make_conveyor(
                "beltB", (b["cx"], (b["y0"] + b["y1"]) / 2, b["top"] / 2),
                (b["width"] / 2, (b["y1"] - b["y0"]) / 2, b["top"] / 2),
                velocity=(0, b["speed"], 0), color=BELT_COL, mat=m_belt),
        }
        conveyors["bconnect"] = self.build_b_connector(m_belt, m_wall)

        # flow blades: escapement (synchronized induction release) + pre-gate
        # hold (single item in the vision window). Both on the full belt body.
        beltA_p = conveyors["beltA"]
        blades = {
            "egate": self.build_blade("egate", a["gate_x"], a["y"],
                                      a["width"] - 0.02, filter_paths=[beltA_p]),
            "hold2": self.build_blade("hold2", a["hold2_x"], a["y"],
                                      a["width"] - 0.02, filter_paths=[beltA_p]),
        }

        # low side guides along belt A (up to the nose; above the knife the
        # guide bottom stays clear of the tray-lip sweep)
        for sgn, nm in ((1, "l"), (-1, "r")):
            self.add_box(f"beltA_guide_{nm}",
                         (a["knife_x0"] / 2, a["y"] + sgn * (a["width"] / 2 + 0.015),
                          a["top"] + 0.04),
                         (a["knife_x0"] / 2, 0.015, 0.05), color=STEEL, mat=m_wall)
            self.add_box(f"knife_guide_{nm}",
                         ((a["knife_x0"] + a["nose_x"]) / 2,
                          a["y"] + sgn * (a["width"] / 2 + 0.015),
                          a["top"] + 0.045),
                         ((a["nose_x"] - a["knife_x0"]) / 2, 0.015, 0.045),
                         color=STEEL, mat=m_wall)

        # ---- the tilt-tray sorter train
        carriers = self.build_sorter_train(m_tray)
        for zone in ("C", "B", "D", "REVIEW"):
            self.build_station(zone, m_wall)

        # ---- chutes + destinations
        cages = P.cages_for()
        wall_c = (cages["C"]["center"][1] + cages["C"]["inner"][1] / 2
                  + cages["C"]["wall_t"] / 2)
        wall_d = (cages["D"]["center"][1] + cages["D"]["inner"][1] / 2
                  + cages["D"]["wall_t"] / 2)
        rp = P.REVIEW_PEN
        wall_r = rp["center"][1] - rp["inner"][1] / 2 - rp["wall_t"] / 2
        self.build_chute("C", P.CHUTE_C, wall_c, m_chute, m_pad, m_wall)
        self.build_chute("D", P.CHUTE_D, wall_d, m_chute, m_pad, m_wall)
        self.build_chute("REVIEW", P.CHUTE_REVIEW, wall_r, m_chute, m_pad,
                         m_wall)
        for zone, cage in cages.items():
            self.build_cage(zone, cage, m_wall, m_mat)
        self.build_cage("REVIEW", rp, m_wall, m_mat, tag="reviewpen")

        # items: one physics material per slug from the material table
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

        # cameras
        vs = P.VIRTUAL_SENSOR
        ox = vs["overhead_pos"][0]
        cams = {
            "overview": self.build_camera("overview", (2.4, -2.4, 4.6),
                                          (0.7006, -0.7136, 0), (0.3288, 0.3229, 0.8880)),
            "top_view": self.build_camera("top_view", (7.7, 3.0, 6.5),
                                          (1, 0, 0), (0, 1, 0)),
            # south view onto the discharge stations + cages
            "routing": self.build_camera_lookat(
                "routing", (9.0, 0.55, 2.75), (8.15, 2.8, 0.62), fovy_deg=50.0),
            # overhead metrology head (also the perception still camera)
            "lookahead": self.build_camera(
                "lookahead", (ox, 3.0, vs["overhead_pos"][2]), (1, 0, 0), (0, 1, 0)),
            # close-range MACRO head (small-item certification; dual-range DWS)
            "macro": self.build_camera(
                "macro", (vs["macro_pos"][0], vs["macro_pos"][1],
                          vs["macro_pos"][2]), (1, 0, 0), (0, 1, 0),
                fovy_deg=vs["macro_fov_deg"]),
            "hero_sw": self.build_camera_lookat(
                "hero_sw", (0.6, -1.9, 2.3), (5.4, 3.0, 0.75), fovy_deg=52.0),
            "cell_iso": self.build_camera_lookat(
                "cell_iso", (-1.4, -1.4, 5.6), (5.0, 3.0, 0.55), fovy_deg=48.0),
            # elevated 3/4 from the open SOUTH-EAST corner onto the train +
            # discharge stations
            "deck_front": self.build_camera_lookat(
                "deck_front", (9.85, 1.15, 2.35), (8.15, 2.95, 0.68),
                fovy_deg=46.0),
            "deck_top": self.build_camera_lookat(
                "deck_top", (8.05, 3.0, 2.85), (8.05, 3.0, 0.64),
                fovy_deg=52.0),
            # presentation brief: LOW north-east 3/4 hero along the train —
            # cages on the FAR side, arm in profile, sorter line unblocked
            "hero_ne": self.build_camera_lookat(
                "hero_ne", (9.8, 5.3, 1.55), (7.0, 2.85, 0.62), fovy_deg=50.0),
            # mechanism close-up: C-station tilt + chute + cage aperture in
            # one technical side view (from over cage C's SE corner, above
            # the arm pedestal sightline)
            "mech_c_side": self.build_camera_lookat(
                "mech_c_side", (8.5, 1.45, 1.10), (7.42, 2.70, 0.50),
                fovy_deg=38.0),
            # B-transfer close-up: tray lip -> powered incline -> belt B
            "b_transfer": self.build_camera_lookat(
                "b_transfer", (9.35, 3.85, 1.05), (8.40, 3.55, 0.58),
                fovy_deg=40.0),
        }
        # DWS side profiler heads
        off, zh = vs["side_head_offset_m"], vs["side_head_z_m"]
        a_y, a_top = P.BELT_A["y"], P.BELT_A["top"]
        cams["side_a"] = self.build_camera_lookat(
            "side_a", (ox, a_y - off, zh), (ox, a_y, a_top + 0.10))
        cams["side_b"] = self.build_camera_lookat(
            "side_b", (ox, a_y + off, zh), (ox, a_y, a_top + 0.10))
        # jam-localization camera: straight-down over the chutes + cages +
        # arm exception zone (south side)
        cams["jamcam"] = self.build_camera("jamcam", (8.15, 2.3, 3.8),
                                           (1, 0, 0), (0, 1, 0))

        # industrial presentation layer (visuals only — physics/sensor-safe)
        viz = None
        try:
            from isaac.dressing import dress_scene
            viz = dress_scene(st)
        except Exception as exc:
            print(f"[dressing] SKIPPED ({exc})", flush=True)
        return {"items": item_info, "cams": cams, "conveyors": conveyors,
                "blades": blades, "carriers": carriers, "viz": viz}


def load_manifest(repo_root, extra=False):
    """Official 11-item ground truth; extra=True merges the synthetic
    borderline/edge sets (manifest_extra.json + manifest_edge.json)."""
    root = Path(repo_root) / "cell" / "assets"
    entries = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    if extra:
        for name in ("manifest_extra.json", "manifest_edge.json"):
            p = root / name
            if p.exists():
                entries += json.loads(p.read_text(encoding="utf-8"))
    return entries
