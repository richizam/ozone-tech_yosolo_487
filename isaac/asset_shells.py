# -*- coding: utf-8 -*-
"""Official Isaac Sim assets as VISUAL SHELLS over the validated physics.

Strategy (user directive): don't rebuild industrial hardware from primitives.
The invisible kinematic surface-velocity colliders remain the authoritative
physics; official conveyor/robot/sensor assets provide the credible look.
Every referenced subtree is sanitized: colliders and rigid bodies disabled so
the shells can never touch the validated contact physics.
"""
import math

from pxr import Gf, Usd, UsdGeom, UsdPhysics

ASSETS = {
    "belt": "/Isaac/Props/Conveyors/ConveyorBelt_A05.usd",     # 2.0 m straight belt
    "roller": "/Isaac/Props/Conveyors/ConveyorBelt_A08.usd",   # 2.72 m roller bed
    "ur10e": "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
    "ur_mount": "/Isaac/Props/Mounts/ur10_mount.usd",
}
# official suction end-effector (the UR10 palletizing demo gripper) — the
# exception arm is a vacuum picker, so the flange should carry a real EEF,
# not end at bare metal. Candidate paths vary across asset-pack versions.
GRIPPER_CANDIDATES = (
    # verified on the 6.0 asset S3 (bucket listing 2026-07-09)
    "/Isaac/Robots/UniversalRobots/ur10/grippers/short_gripper.usd",
    "/Isaac/Robots/UR10/Props/short_gripper.usd",       # classic 4.x path
)


def _stat_ok(url):
    try:
        import omni.client
        res, _ = omni.client.stat(url)
        return res == omni.client.Result.OK
    except Exception:
        return False
# measured in the gallery probe: bbox top of the conveyor family (rail top)
CONVEYOR_BBOX_TOP = 1.17
# the riding BELT surface sits below the rail top; calibrated via preview
SURFACE_FRACTION = 0.855


def assets_root():
    try:
        from isaacsim.storage.native import get_assets_root_path
    except ImportError:
        from isaacsim.core.utils.nucleus import get_assets_root_path
    return get_assets_root_path()


def _set_ops(prim, translate=None, rotate_deg=None, scale=None):
    """Set xform ops REUSING existing ops (referenced assets own theirs)."""
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpType(): op for op in xf.GetOrderedXformOps()}
    if translate is not None:
        op = ops.get(UsdGeom.XformOp.TypeTranslate) or xf.AddTranslateOp()
        op.Set(Gf.Vec3d(*[float(v) for v in translate]))
    if rotate_deg is not None:
        op = ops.get(UsdGeom.XformOp.TypeRotateXYZ) or xf.AddRotateXYZOp()
        op.Set(Gf.Vec3f(*[float(v) for v in rotate_deg]))
    if scale is not None:
        op = ops.get(UsdGeom.XformOp.TypeScale) or xf.AddScaleOp()
        op.Set(Gf.Vec3f(*[float(v) for v in scale]))


def sanitize(prim, keep_joints=False):
    """Disable every physics API in a referenced subtree (visual shell).
    Instanceable prototypes are opted out first — Usd.PrimRange does not
    descend into instances, which would leave their colliders live."""
    for _ in range(4):                              # nested instancing
        changed = False
        for p in Usd.PrimRange(prim):
            if p.IsInstanceable():
                p.SetInstanceable(False)
                changed = True
        if not changed:
            break
    for p in Usd.PrimRange(prim):
        # referenced assets may SHIP THEIR OWN LIGHTS (the UR10e/gripper
        # pack lit the whole C-bin corner and blew out the exposure — the
        # user's "spotlight on the arm"). A shell contributes geometry only.
        if p.GetTypeName().endswith("Light"):
            inten = p.GetAttribute("inputs:intensity")
            if inten:
                inten.Set(0.0)
            UsdGeom.Imageable(p).MakeInvisible()
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
        if not keep_joints:
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                UsdPhysics.ArticulationRootAPI(p).CreateArticulationEnabledAttr(
                    False)
            if p.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(p).CreateJointEnabledAttr(False)


def reference(stage, path, url, translate, rotate_deg=(0, 0, 0),
              scale=(1, 1, 1), keep_joints=False):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(url)
    _set_ops(prim, translate, rotate_deg, scale)
    sanitize(prim, keep_joints=keep_joints)
    return prim


def conveyor_shells(stage, top=0.70):
    """Official conveyor segments matched to the cell layout. Returns a dict
    {"arb_pills": {patch_id: [pill paths]}} on success (caller then hides the
    collider boxes and skips primitive decor), None on failure.

    EVERY item-contact surface is CONTINUOUS (A05 belt): the official test
    set includes a 9 mm pen, so open roller beds would contradict the physics
    on video ("why doesn't the pen fall between the rollers?"). The routing
    zone reads as an Intralox-class Activated Roller Belt — a continuous
    belt with small rollers embedded FLUSH in the surface (added by
    arb_overlay) — which is also exactly what the vectored surface-velocity
    physics simulates. Roller drive hardware stays visible at the drums and
    side frames, never as gaps under the freight.

    Segments (asset local: length along X, belt at z SURFACE_FRACTION*bbox_top):
      belt A   x 0.00-6.90  -> 3x A05 tiles
      entry    x 6.90-7.95  -> A05 belt (continuous)
      zone     x 7.95-8.55  -> A05 belt + flush ARB roller overlay
      connectB y 3.55-4.20  -> A05 (rot 90)
      belt B   y 4.20-6.00  -> A05 (rot 90)
    """
    root = assets_root()
    if not root:
        return None
    zs = top / (CONVEYOR_BBOX_TOP * SURFACE_FRACTION)
    parent = "/World/shells"
    UsdGeom.Xform.Define(stage, parent)

    RUBBER = Gf.Vec3f(0.075, 0.078, 0.085)

    m_band = _pbr(stage, f"{parent}/mat_band", (0.055, 0.058, 0.065),
                  metallic=0.0, roughness=0.88)        # worn belt rubber:
    m_skirt = _pbr(stage, f"{parent}/mat_skirt", (0.20, 0.22, 0.26),
                   metallic=0.35, roughness=0.6)       # steel skirting

    def belt(name, cx, cy, length, width, yaw=0.0, kind="belt",
             native_len=2.0, band=True):
        prim = reference(stage, f"{parent}/{name}", root + ASSETS[kind],
                         (cx, cy, 0.0), (0, 0, yaw),
                         (length / native_len, width / 0.62, zs))
        # BBOX-FIT along the flow axis: the asset's real content overhangs
        # its nominal 2 m (end drums, motor boxes) — the single stretched
        # belt-A tile pushed its tail drums 3.4 m past the belt end, straight
        # into the cage-C volume (audit: SM_..._A05 aabb to x=10.35). Measure
        # and rescale so ALL content stays inside the nominal span.
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                 [UsdGeom.Tokens.default_]
                                 ).ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = bbox.GetMin(), bbox.GetMax()
        ax = 0 if abs(yaw) < 1e-6 else 1
        ext = float(mx[ax] - mn[ax])
        if ext > length + 1e-3:
            s_along = (length / native_len) * (length / ext)
            _set_ops(prim, scale=((s_along, width / 0.62, zs) if ax == 0
                                  else (s_along, width / 0.62, zs)))
            bbox = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
            ).ComputeWorldBound(prim).ComputeAlignedRange()
            mn, mx = bbox.GetMin(), bbox.GetMax()
        off = [0.0, 0.0, 0.0]
        off[ax] = (cx if ax == 0 else cy) - float(mn[ax] + mx[ax]) / 2
        _set_ops(prim, ((cx + off[0]) if ax == 0 else cx,
                        (cy + off[1]) if ax == 1 else cy, 0.0))
        # §2 audit fix: the asset's interior Rollers/Rubberbands sub-meshes
        # sit UNDER the belt band (z 0.50-0.54) and read as a fake conveyor
        # ("item rides a flat belt while rollers sit underneath"). They
        # contribute nothing (sanitized visual shell) — hide them and close
        # the sides with steel skirting panels instead.
        for scope in ("Rollers", "Rubberbands"):
            for p in Usd.PrimRange(prim):
                if p.GetName() == scope:
                    UsdGeom.Imageable(p).MakeInvisible()
        if not band:                    # the ARB wheel deck IS the surface
            return
        from pxr import UsdShade
        band_c = UsdGeom.Cube.Define(stage, f"{parent}/{name}_band")
        band_c.CreateSizeAttr(2.0)
        xf = UsdGeom.Xformable(band_c.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(float(cx), float(cy), top - 0.007))
        if abs(yaw) > 1e-6:
            xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, float(yaw)))
        xf.AddScaleOp().Set(Gf.Vec3f(length / 2, width / 2 - 0.035, 0.007))
        UsdShade.MaterialBindingAPI.Apply(band_c.GetPrim()).Bind(m_band)
        # enclosed underside: skirting panels both sides (z 0.52 -> band)
        for sgn, nm in ((-1, "s"), (1, "n")):
            sk = UsdGeom.Cube.Define(stage, f"{parent}/{name}_skirt_{nm}")
            sk.CreateSizeAttr(2.0)
            sxf = UsdGeom.Xformable(sk.GetPrim())
            oy = sgn * (width / 2 - 0.02)
            if abs(yaw) < 1e-6:
                sxf.AddTranslateOp().Set(Gf.Vec3d(cx, cy + oy, top - 0.095))
            else:
                sxf.AddTranslateOp().Set(Gf.Vec3d(cx + oy, cy, top - 0.095))
                sxf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, float(yaw)))
            sxf.AddScaleOp().Set(Gf.Vec3f(length / 2, 0.008, 0.088))
            UsdShade.MaterialBindingAPI.Apply(sk.GetPrim()).Bind(m_skirt)

    # belt A: a SINGLE stretched tile — tile seams put the asset's end-cap
    # hardware mid-span ABOVE the belt surface and items visibly clipped
    # through it; one tile keeps head/tail structures at the true ends only
    belt("beltA_0", 6.9 / 2, 3.0, 6.9, 0.56)
    # table: continuous belts (see docstring), ARB caps overlaid on the zone
    belt("entry", (6.9 + 7.95) / 2, 3.0, 1.05, 1.12)
    arb_viz = arb_overlay(stage, parent, top, root)
    # powered connector + belt B (along Y)
    belt("connectB", 8.4, (3.55 + 4.20) / 2, 0.65, 0.56, yaw=90.0)
    belt("beltB", 8.4, (4.20 + 6.00) / 2, 1.80, 0.56, yaw=90.0)
    return {"arb_pills": arb_viz["leds"], "arb_rollers": arb_viz["rollers"]}


def _pbr(stage, path, color, metallic=0.0, roughness=0.6):
    """Shared UsdPreviewSurface PBR material (real metal/rubber response
    instead of flat displayColor)."""
    from pxr import Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/pbr")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def arb_overlay(stage, parent, top, root=None):
    """ARB actuator deck — a LATERAL SORTER TOP the freight visibly rides:
    a dense field of 45-degree steerable wheels (Intralox-ARB / wheel-sorter
    class) with their crowns exactly at the ride plane, mounted on visible
    steel shafts with galvanized brackets over a recessed deck plate, framed
    by powder-coated side channels with bolts, drive motors and a cable
    tray. No official asset ships this mechanism (the full A01-A49 conveyor
    set was swept: A49 is a longitudinal roller/transfer bed — items read as
    riding a flat band; the fork diverts A45/46 are 4x4.5 m branch modules),
    so the deck top is procedural by design; official conveyor assets are
    used for infeed/outfeed only.

    Module state shows on a 4x7 STATUS LED MATRIX on the deck's south face
    (machine-HMI idiom) — one LED per physics patch — and the wheels of a
    commanded patch SPIN toward the commanded direction at runtime.

    Honesty note (documented in isaac/README): the validated physics is
    PATCH-LEVEL surface actuation (PhysxSurfaceVelocityAPI per module with
    latency/ramp/saturation/noise); this wheel field is the mechanical /
    visual embodiment. Wheel-by-wheel bearing/contact simulation is
    intentionally not used — neither required by the evidence nor validated.

    Returns {"leds": {patch_id: [paths]},
             "rollers": {patch_id: [(spin_xform_path, sign)]}}."""
    from pxr import UsdShade
    from cell import params as _P
    tb = _P.TABLE
    x0, x1 = tb["route_x"], tb["x1"]
    y0, y1 = tb["y"] - tb["width"] / 2, tb["y"] + tb["width"] / 2
    zone_cx = (x0 + x1) / 2
    UsdGeom.Xform.Define(stage, f"{parent}/arb")
    m_wheel = _pbr(stage, f"{parent}/arb/mat_wheel", (0.055, 0.056, 0.062),
                   metallic=0.0, roughness=0.82)     # dark polyurethane
    m_steel = _pbr(stage, f"{parent}/arb/mat_steel", (0.62, 0.63, 0.65),
                   metallic=0.9, roughness=0.42)     # shafts, worn
    m_galv = _pbr(stage, f"{parent}/arb/mat_galv", (0.48, 0.50, 0.52),
                  metallic=0.55, roughness=0.6)      # galvanized brackets
    m_frame = _pbr(stage, f"{parent}/arb/mat_frame", (0.15, 0.17, 0.21),
                   metallic=0.25, roughness=0.62)    # powder-coated channel
    m_plate = _pbr(stage, f"{parent}/arb/mat_plate", (0.085, 0.09, 0.10),
                   metallic=0.1, roughness=0.8)      # recessed deck plate

    def bind(prim, mat):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    def box(path, c, h, mat, rot=None):
        b = UsdGeom.Cube.Define(stage, path)
        b.CreateSizeAttr(2.0)
        xf = UsdGeom.Xformable(b.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in c]))
        if rot is not None:
            xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
        xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in h]))
        bind(b.GetPrim(), mat)
        return b

    # recessed deck plate (wheels sit proud of it up to the ride plane)
    box(f"{parent}/arb/plate", (zone_cx, tb["y"], top - 0.020),
        ((x1 - x0) / 2, (y1 - y0) / 2, 0.006), m_plate)
    # wheel field: 8 x 14 steerable wheels at 45 deg, crowns AT the ride
    # plane — freight visibly rides the wheels, not a flat band
    R_W, L_W = 0.024, 0.048
    NX, NY = 8, 14
    px_, py_ = (x1 - x0) / NX, (y1 - y0) / NY
    rollers = {f"r{p['r']}c{p['c']}": [] for p in _P.arb_patches()}
    wi = 0
    for ic in range(NX):
        for ir in range(NY):
            wx = x0 + (ic + 0.5) * px_
            wy = y0 + (ir + 0.5) * py_
            base = f"{parent}/arb/w_{wi}"
            bx = UsdGeom.Xform.Define(stage, base)
            bxf = UsdGeom.Xformable(bx.GetPrim())
            bxf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, top - R_W))
            bxf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 45.0))
            # fixed shaft through the wheel + galvanized mounting bracket
            shaft = UsdGeom.Cylinder.Define(stage, f"{base}/shaft")
            shaft.CreateRadiusAttr(0.005)
            shaft.CreateHeightAttr(L_W + 0.026)
            shaft.CreateAxisAttr("X")
            bind(shaft.GetPrim(), m_steel)
            box(f"{base}/bracket", (0, 0, -R_W + 0.008),
                (0.008, 0.016, 0.010), m_galv)
            # spinning wheel body + tread groove (rotation reads on video)
            spin = UsdGeom.Xform.Define(stage, f"{base}/spin")
            UsdGeom.Xformable(spin.GetPrim()).AddRotateXOp().Set(
                float((wi * 47) % 360))
            body = UsdGeom.Cylinder.Define(stage, f"{base}/spin/wheel")
            body.CreateRadiusAttr(R_W)
            body.CreateHeightAttr(L_W)
            body.CreateAxisAttr("X")
            bind(body.GetPrim(), m_wheel)
            ring = UsdGeom.Cylinder.Define(stage, f"{base}/spin/ring")
            ring.CreateRadiusAttr(R_W + 0.0012)
            ring.CreateHeightAttr(0.007)
            ring.CreateAxisAttr("X")
            UsdGeom.Xformable(ring.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(0.012, 0, 0))
            bind(ring.GetPrim(), m_steel)
            # patch mapping: 2x2 wheels per 150x157 mm physics patch
            pid = f"r{min(ir // 2, 6)}c{min(ic // 2, 3)}"
            rollers[pid].append((f"{base}/spin", 1))
            wi += 1
    # powder-coated side channels + end plates + bolts (C-channel frame)
    for sgn, nm in ((-1, "s"), (1, "n")):
        cy = tb["y"] + sgn * ((y1 - y0) / 2 + 0.018)
        box(f"{parent}/arb/chan_{nm}", (zone_cx, cy, top - 0.055),
            ((x1 - x0) / 2 + 0.02, 0.012, 0.062), m_frame)
        box(f"{parent}/arb/flange_{nm}", (zone_cx, cy, top + 0.004),
            ((x1 - x0) / 2 + 0.02, 0.016, 0.004), m_frame)
        for i in range(5):
            bolt = UsdGeom.Cylinder.Define(stage,
                                           f"{parent}/arb/bolt_{nm}{i}")
            bolt.CreateRadiusAttr(0.006)
            bolt.CreateHeightAttr(0.006)
            bolt.CreateAxisAttr("Y")
            UsdGeom.Xformable(bolt.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(x0 + 0.06 + i * 0.12, cy + sgn * 0.013, top - 0.045))
            bind(bolt.GetPrim(), m_galv)
    for ex, nm in ((x0 - 0.012, "w"), (x1 + 0.012, "e")):
        box(f"{parent}/arb/end_{nm}", (ex, tb["y"], top - 0.055),
            (0.012, (y1 - y0) / 2 + 0.03, 0.062), m_frame)
    # drive motors + cable tray on the south face (under the status panel)
    for i, mx_ in enumerate((x0 + 0.14, x1 - 0.14)):
        box(f"{parent}/arb/motor_{i}", (mx_, y0 - 0.055, top - 0.10),
            (0.045, 0.038, 0.036), m_frame)
        mc = UsdGeom.Cylinder.Define(stage, f"{parent}/arb/motor_cap{i}")
        mc.CreateRadiusAttr(0.024)
        mc.CreateHeightAttr(0.05)
        mc.CreateAxisAttr("Y")
        UsdGeom.Xformable(mc.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(mx_, y0 - 0.11, top - 0.10))
        bind(mc.GetPrim(), m_galv)
    box(f"{parent}/arb/tray", (zone_cx, y0 - 0.04, top - 0.165),
        ((x1 - x0) / 2, 0.022, 0.008), m_galv)
    print("[dressing] ARB deck = procedural 45-deg steerable-wheel sorter "
          "top (8x14 wheels, crowns at ride plane)", flush=True)
    # ---- 4x7 module status LED matrix on the south face (routing-cam side)
    m_panel = _pbr(stage, f"{parent}/arb_panel_mat", (0.10, 0.11, 0.13),
                   metallic=0.2, roughness=0.5)
    from pxr import UsdShade
    panel = UsdGeom.Cube.Define(stage, f"{parent}/arb_panel")
    panel.CreateSizeAttr(2.0)
    py = tb["y"] - tb["width"] / 2 - 0.045
    pxf = UsdGeom.Xformable(panel.GetPrim())
    pxf.AddTranslateOp().Set(Gf.Vec3d(zone_cx, py, 0.575))
    pxf.AddScaleOp().Set(Gf.Vec3f(0.155, 0.008, 0.062))
    UsdShade.MaterialBindingAPI.Apply(panel.GetPrim()).Bind(m_panel)
    leds = {}
    for p in _P.arb_patches():
        pid = f"r{p['r']}c{p['c']}"
        # matrix layout: columns follow the patch x-column (flow), rows
        # follow the patch y-row — the panel mirrors the deck top-down
        lx = zone_cx - 0.155 + 0.048 + p["c"] * 0.072
        lz = 0.575 - 0.048 + p["r"] * 0.016
        led = UsdGeom.Cube.Define(stage, f"{parent}/arb_led_{pid}")
        led.CreateSizeAttr(2.0)
        lxf = UsdGeom.Xformable(led.GetPrim())
        lxf.AddTranslateOp().Set(Gf.Vec3d(lx, py - 0.006, lz))
        lxf.AddScaleOp().Set(Gf.Vec3f(0.026, 0.003, 0.0055))
        led.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.24, 0.26)])
        leds[pid] = [f"{parent}/arb_led_{pid}"]
    return {"leds": leds, "rollers": rollers}


def ur10e_arm(stage, base_xy, pedestal_h=0.65):
    """Official UR10e + mount as the exception arm's body. The articulation
    stays ENABLED but is posed by direct joint-STATE writes each control tick
    (SingleArticulation.set_joint_positions) — PD drives sag under gravity
    and lag the reference, which read as "the arm is stuck" on video.
    Collisions disabled + gravity disabled on every link (kinematic-visual
    parity with the MuJoCo twin). Returns paths dict or None."""
    from pxr import PhysxSchema
    root = assets_root()
    if not root:
        return None
    bx, by = base_xy
    reference(stage, "/World/ur_mount", root + ASSETS["ur_mount"],
              (bx, by, 0.0), (0, 0, 0), (1, 1, pedestal_h / 0.65))
    prim = reference(stage, "/World/ur10e", root + ASSETS["ur10e"],
                     (bx, by, pedestal_h), (0, 0, 0), (1, 1, 1),
                     keep_joints=True)
    # find the revolute joints by name + the articulation root + the flange;
    # kill gravity on every link so the pose cannot sag between state writes
    names = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
    joints, flange, art_root = {}, None, None
    for p in Usd.PrimRange(prim):
        nm = p.GetName()
        if nm in names:
            joints[nm] = p.GetPath().pathString
        if nm in ("tool0", "flange", "wrist_3_link"):
            flange = p.GetPath().pathString if nm != "wrist_3_link" or \
                flange is None else flange
        if p.HasAPI(UsdPhysics.ArticulationRootAPI) and art_root is None:
            art_root = p.GetPath().pathString
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(
                True)
    if len(joints) < 6:
        return None
    # official suction gripper on the flange (sanitized visual shell, child
    # of the flange link so it follows the real arm motion). The controller
    # TCP hangs TOOL=0.20 m below the flange — the gripper body + bellows
    # visually occupy that offset so a carried item reads as held by the
    # vacuum cups, not floating under bare metal.
    if flange is not None:
        for cand in GRIPPER_CANDIDATES:
            if not _stat_ok(root + cand):
                continue
            try:
                gp = reference(stage, f"{flange}/eef_gripper", root + cand,
                               (0.0, 0.0, 0.0), (0, 0, 0), (1, 1, 1))
                sanitize(gp)
                print(f"[arm] suction gripper referenced: {cand}", flush=True)
                break
            except Exception as exc:
                print(f"[arm] gripper reference failed ({exc})", flush=True)
    return {"prim": prim.GetPath().pathString, "joints": joints,
            "flange": flange, "art_root": art_root or prim.GetPath().pathString}
