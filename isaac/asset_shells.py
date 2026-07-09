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

    def belt(name, cx, cy, length, width, yaw=0.0, kind="belt",
             native_len=2.0, band=True):
        reference(stage, f"{parent}/{name}", root + ASSETS[kind],
                  (cx, cy, 0.0), (0, 0, yaw),
                  (length / native_len, width / 0.62, zs))
        if not band:                    # the ARB modules ARE the surface
            return
        # CONTINUOUS rubber belt band riding the asset's roller bed (a real
        # belted-roller conveyor): the official asset supplies frames/stands/
        # drive rollers, the band supplies the continuous contact surface —
        # the 9 mm pen can never read as "about to fall between rollers".
        band_c = UsdGeom.Cube.Define(stage, f"{parent}/{name}_band")
        band_c.CreateSizeAttr(2.0)
        xf = UsdGeom.Xformable(band_c.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(float(cx), float(cy), top - 0.007))
        if abs(yaw) > 1e-6:
            xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, float(yaw)))
        xf.AddScaleOp().Set(Gf.Vec3f(length / 2, width / 2 - 0.035, 0.007))
        band_c.CreateDisplayColorAttr([RUBBER])

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
    """ARB actuator deck — the OFFICIAL right-angle transfer module
    ConveyorBelt_A49 (found by sweeping the full A42-A49 tail of the 6.0
    conveyor set: A49 is a standalone 1.06 x 1.08 m transfer deck — silver
    carry rollers with interleaved pop-up transfer wheel packs, the exact
    industrial mechanism class of this cell's routing zone). Referenced as
    a sanitized visual shell fitted over the zone: deck top exactly at the
    ride plane, so freight visibly rides the real rollers.

    Module state is shown on a 4x7 STATUS LED MATRIX on the deck's south
    face (the machine-HMI idiom real sorters use) — one LED per physics
    patch, tinted by the actuator state at runtime. No dots, pills or
    painted arrows on the deck.

    Honesty note (documented in isaac/README): the validated physics is
    PATCH-LEVEL surface actuation (PhysxSurfaceVelocityAPI per module with
    latency/ramp/saturation/noise); the A49 shell is the mechanical/visual
    embodiment. Roller-by-roller bearing/contact simulation is intentionally
    not used — neither required by the evidence nor validated.

    Returns {"leds": {patch_id: [prim paths]}, "rollers": {}}."""
    from cell import params as _P
    tb = _P.TABLE
    zone_cx = (tb["route_x"] + tb["x1"]) / 2
    zone_len = tb["x1"] - tb["route_x"]
    if root:
        prim = stage.DefinePrim(f"{parent}/arb_a49", "Xform")
        prim.GetReferences().AddReference(
            root + "/Isaac/Props/Conveyors/ConveyorBelt_A49.usd")
        _set_ops(prim, (0.0, 0.0, 0.0), (0, 0, 0),
                 (zone_len / 1.06, (tb["width"] + 0.06) / 1.08, top / 0.80))
        sanitize(prim)
        # fit by measured bounds: center the deck on the zone, top at the
        # ride plane (the asset's origin is not its bbox center)
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                 [UsdGeom.Tokens.default_]
                                 ).ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = bbox.GetMin(), bbox.GetMax()
        _set_ops(prim, (zone_cx - (mn[0] + mx[0]) / 2,
                        tb["y"] - (mn[1] + mx[1]) / 2,
                        top - mx[2]))
        print("[dressing] ARB deck = official ConveyorBelt_A49 transfer "
              "module (sanitized shell)", flush=True)
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
    return {"leds": leds, "rollers": {}}


def _unused_procedural_arb_modules(stage, parent, top):
    """Procedural angled-roller module field — kept as the documented
    fallback if the official asset server is unreachable offline. The A49
    shell above won the side-by-side render comparison."""
    from cell import params as _P
    UsdGeom.Xform.Define(stage, f"{parent}/arb")
    m_roller = _pbr(stage, f"{parent}/arb/mat_roller", (0.045, 0.047, 0.052),
                    metallic=0.0, roughness=0.85)      # rubber-lagged
    m_steel = _pbr(stage, f"{parent}/arb/mat_steel", (0.72, 0.73, 0.75),
                   metallic=0.95, roughness=0.35)      # shafts/bearings
    m_frame = _pbr(stage, f"{parent}/arb/mat_frame", (0.16, 0.17, 0.20),
                   metallic=0.30, roughness=0.55)      # anthracite frame
    m_plate = _pbr(stage, f"{parent}/arb/mat_plate", (0.23, 0.25, 0.29),
                   metallic=0.15, roughness=0.70)      # module plate

    from pxr import UsdShade

    def bind(prim, mat):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    R_ROLL = 0.016            # roller radius (50 mm class hardware: 32 mm OD)
    L_ROLL = 0.058            # rubber sleeve length
    L_SHAFT = 0.082           # shaft protrudes into the bearing blocks
    leds, rollers = {}, {}
    for p in _P.arb_patches():
        pid = f"r{p['r']}c{p['c']}"
        leds[pid], rollers[pid] = [], []
        mod = f"{parent}/arb/mod_{pid}"
        UsdGeom.Xform.Define(stage, mod)
        # module plate: rollers sit proud of it up to the ride plane
        plate = UsdGeom.Cube.Define(stage, f"{mod}/plate")
        plate.CreateSizeAttr(2.0)
        pxf = UsdGeom.Xformable(plate.GetPrim())
        pxf.AddTranslateOp().Set(Gf.Vec3d(p["cx"], p["cy"], top - 0.012))
        pxf.AddScaleOp().Set(Gf.Vec3f(p["hx"] - 0.005, p["hy"] - 0.005, 0.004))
        bind(plate.GetPrim(), m_plate)
        # frame bars around the module perimeter
        for k, (dx, dy, sx, sy) in enumerate((
                (0.0, p["hy"] - 0.003, p["hx"], 0.003),
                (0.0, -p["hy"] + 0.003, p["hx"], 0.003),
                (p["hx"] - 0.003, 0.0, 0.003, p["hy"] - 0.006),
                (-p["hx"] + 0.003, 0.0, 0.003, p["hy"] - 0.006))):
            bar = UsdGeom.Cube.Define(stage, f"{mod}/frame_{k}")
            bar.CreateSizeAttr(2.0)
            bxf = UsdGeom.Xformable(bar.GetPrim())
            bxf.AddTranslateOp().Set(Gf.Vec3d(p["cx"] + dx, p["cy"] + dy,
                                              top - 0.007))
            bxf.AddScaleOp().Set(Gf.Vec3f(sx, sy, 0.005))
            bind(bar.GetPrim(), m_frame)
        # six angled rollers: 2 columns x 3 rows per module
        ri = 0
        for ox in (-p["hx"] / 2, p["hx"] / 2):
            for oy in (-0.052, 0.0, 0.052):
                base = f"{mod}/roller_{ri}"
                bx = UsdGeom.Xform.Define(stage, base)
                bxf = UsdGeom.Xformable(bx.GetPrim())
                bxf.AddTranslateOp().Set(Gf.Vec3d(p["cx"] + ox,
                                                  p["cy"] + oy,
                                                  top - R_ROLL))
                bxf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 45.0))
                # fixed steel shaft + bearing blocks (the roller spins
                # AROUND the shaft, as real hardware does)
                shaft = UsdGeom.Cylinder.Define(stage, f"{base}/shaft")
                shaft.CreateRadiusAttr(0.0045)
                shaft.CreateHeightAttr(L_SHAFT)
                shaft.CreateAxisAttr("X")
                bind(shaft.GetPrim(), m_steel)
                for sgn, nm in ((-1, "a"), (1, "b")):
                    brg = UsdGeom.Cube.Define(stage, f"{base}/bearing_{nm}")
                    brg.CreateSizeAttr(2.0)
                    gxf = UsdGeom.Xformable(brg.GetPrim())
                    gxf.AddTranslateOp().Set(
                        Gf.Vec3d(sgn * (L_SHAFT / 2 - 0.004), 0, -0.004))
                    gxf.AddScaleOp().Set(Gf.Vec3f(0.006, 0.009, 0.010))
                    bind(brg.GetPrim(), m_frame)
                # spinning rubber sleeve (runtime rotateX from ArbDeck)
                spin = UsdGeom.Xform.Define(stage, f"{base}/spin")
                spin_op = UsdGeom.Xformable(spin.GetPrim()).AddRotateXOp()
                spin_op.Set(float((ri * 53) % 360))    # varied start phase
                body = UsdGeom.Cylinder.Define(stage, f"{base}/spin/sleeve")
                body.CreateRadiusAttr(R_ROLL)
                body.CreateHeightAttr(L_ROLL)
                body.CreateAxisAttr("X")
                bind(body.GetPrim(), m_roller)
                # machined groove ring: makes rotation readable on video
                ring = UsdGeom.Cylinder.Define(stage, f"{base}/spin/ring")
                ring.CreateRadiusAttr(R_ROLL + 0.0012)
                ring.CreateHeightAttr(0.006)
                ring.CreateAxisAttr("X")
                rxf = UsdGeom.Xformable(ring.GetPrim())
                rxf.AddTranslateOp().Set(Gf.Vec3d(0.014, 0, 0))
                bind(ring.GetPrim(), m_steel)
                # lateral sign: +1 pushes north (B), -1 south (D)
                rollers[pid].append((f"{base}/spin", 1))
                ri += 1
        # module status LED bar on the south frame edge (subtle industrial
        # indicator — replaces the old painted deck arrows)
        led = UsdGeom.Cube.Define(stage, f"{mod}/led")
        led.CreateSizeAttr(2.0)
        lxf = UsdGeom.Xformable(led.GetPrim())
        lxf.AddTranslateOp().Set(Gf.Vec3d(p["cx"], p["cy"] - p["hy"] + 0.003,
                                          top - 0.0035))
        lxf.AddScaleOp().Set(Gf.Vec3f(0.030, 0.0022, 0.0022))
        led.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.24, 0.26)])
        leds[pid].append(f"{mod}/led")
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
