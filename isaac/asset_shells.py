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
    """Official conveyor segments matched to the cell layout. Returns True on
    success (caller then hides the collider boxes and skips primitive decor).

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
        return False
    zs = top / (CONVEYOR_BBOX_TOP * SURFACE_FRACTION)
    parent = "/World/shells"
    UsdGeom.Xform.Define(stage, parent)

    RUBBER = Gf.Vec3f(0.075, 0.078, 0.085)

    def belt(name, cx, cy, length, width, yaw=0.0, kind="belt",
             native_len=2.0):
        reference(stage, f"{parent}/{name}", root + ASSETS[kind],
                  (cx, cy, 0.0), (0, 0, yaw),
                  (length / native_len, width / 0.62, zs))
        # CONTINUOUS rubber belt band riding the asset's roller bed (a real
        # belted-roller conveyor): the official asset supplies frames/stands/
        # drive rollers, the band supplies the continuous contact surface —
        # the 9 mm pen can never read as "about to fall between rollers".
        band = UsdGeom.Cube.Define(stage, f"{parent}/{name}_band")
        band.CreateSizeAttr(2.0)
        xf = UsdGeom.Xformable(band.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(float(cx), float(cy), top - 0.007))
        if abs(yaw) > 1e-6:
            xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, float(yaw)))
        xf.AddScaleOp().Set(Gf.Vec3f(length / 2, width / 2 - 0.035, 0.007))
        band.CreateDisplayColorAttr([RUBBER])

    # belt A: a SINGLE stretched tile — tile seams put the asset's end-cap
    # hardware mid-span ABOVE the belt surface and items visibly clipped
    # through it; one tile keeps head/tail structures at the true ends only
    belt("beltA_0", 6.9 / 2, 3.0, 6.9, 0.56)
    # table: continuous belts (see docstring), ARB caps overlaid on the zone
    belt("entry", (6.9 + 7.95) / 2, 3.0, 1.05, 1.12)
    belt("zone", (7.95 + 8.55) / 2, 3.0, 0.60, 1.16)
    arb_overlay(stage, parent, top)
    # powered connector + belt B (along Y)
    belt("connectB", 8.4, (3.55 + 4.20) / 2, 0.65, 0.56, yaw=90.0)
    belt("beltB", 8.4, (4.20 + 6.00) / 2, 1.80, 0.56, yaw=90.0)
    return True


def arb_overlay(stage, parent, top):
    """Activated-Roller-Belt deck on the routing zone — the industrial
    mechanism for 3-way ORTHOGONAL sortation in a sub-metre footprint with
    small items in the mix (the official fork diverts A45/A46 are 45-degree
    branch modules with a 4.0 x 4.5 m footprint: they cannot serve the fixed
    N/E/S exits of this cell — evaluated and documented). The look: dense
    staggered rows of ANGLED pill rollers embedded flush in a lighter belt
    band (dark metal on light deck reads as machinery, not dots); the
    ACTIVE-ROUTE deck arrows brighten over it at runtime. Purely visual —
    the contact surface stays continuous, exactly like a real Intralox ARB."""
    import numpy as np
    UsdGeom.Xform.Define(stage, f"{parent}/arb")
    # lighter deck band over the rubber (the ARB belt itself)
    deck = UsdGeom.Cube.Define(stage, f"{parent}/arb/deck")
    deck.CreateSizeAttr(2.0)
    xf = UsdGeom.Xformable(deck.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(8.25, 3.0, top - 0.0055))
    xf.AddScaleOp().Set(Gf.Vec3f(0.29, 0.52, 0.006))
    deck.CreateDisplayColorAttr([Gf.Vec3f(0.34, 0.36, 0.40)])
    # angled pill rollers, flush in the deck (embedded-roller signature)
    i = 0
    for col, x in enumerate(np.arange(7.99, 8.53, 0.055)):
        row_off = 0.0275 * (col % 2)
        for y in np.arange(2.51 + row_off, 3.47, 0.055):
            cap = UsdGeom.Capsule.Define(stage, f"{parent}/arb/pill_{i}")
            cap.CreateRadiusAttr(0.0085)
            cap.CreateHeightAttr(0.024)
            cap.CreateAxisAttr("X")
            pxf = UsdGeom.Xformable(cap.GetPrim())
            pxf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y),
                                              top - 0.0075))
            pxf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 45.0))
            cap.CreateDisplayColorAttr([Gf.Vec3f(0.14, 0.145, 0.16)])
            i += 1


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
    return {"prim": prim.GetPath().pathString, "joints": joints,
            "flange": flange, "art_root": art_root or prim.GetPath().pathString}
