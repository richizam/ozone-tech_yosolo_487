# -*- coding: utf-8 -*-
"""A/B render: official A49 transfer module scaled to the routing zone vs
the procedural angled-roller ARB module field. Pick the deck look by eye."""
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

try:
    from isaacsim import SimulationApp
except ImportError:
    from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True, "width": 1920, "height": 1080})

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402

try:
    from isaacsim.storage.native import get_assets_root_path
except ImportError:
    from isaacsim.core.utils.nucleus import get_assets_root_path

root = get_assets_root_path()
world = World(physics_dt=1 / 240.0, rendering_dt=1 / 240.0,
              stage_units_in_meters=1.0, backend="numpy")
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(600.0)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(1800.0)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50, 20, 0))

# ---- variant A: official A49 scaled to the 0.60 x 1.10 zone, top 0.70
pa = stage.DefinePrim("/World/varA", "Xform")
pa.GetReferences().AddReference(
    root + "/Isaac/Props/Conveyors/ConveyorBelt_A49.usd")
xf = UsdGeom.Xformable(pa)
ops = {op.GetOpType(): op for op in xf.GetOrderedXformOps()}
(ops.get(UsdGeom.XformOp.TypeTranslate) or xf.AddTranslateOp()).Set(
    Gf.Vec3d(0.0, 0.0, 0.0))
(ops.get(UsdGeom.XformOp.TypeScale) or xf.AddScaleOp()).Set(
    Gf.Vec3f(0.60 / 1.06, 1.10 / 1.08, 0.70 / 0.80))
for p in Usd.PrimRange(pa):
    if p.IsInstanceable():
        p.SetInstanceable(False)
for p in Usd.PrimRange(pa):
    if p.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)

# ---- variant B: procedural ARB module field (same builder the cell uses),
# on a simple stand at x offset. arb_overlay builds at the real cell coords
# (7.95..8.55 x, 2.45..3.55 y), so place the camera to see both.
from isaac.asset_shells import arb_overlay  # noqa: E402
UsdGeom.Xform.Define(stage, "/World/shellsB")
arb_overlay(stage, "/World/shellsB", 0.70)
# plain support box under the module field
sup = UsdGeom.Cube.Define(stage, "/World/shellsB/support")
sup.CreateSizeAttr(2.0)
sxf = UsdGeom.Xformable(sup.GetPrim())
sxf.AddTranslateOp().Set(Gf.Vec3d(8.25, 3.0, 0.33))
sxf.AddScaleOp().Set(Gf.Vec3f(0.30, 0.55, 0.33))
sup.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.28, 0.45)])

world.reset()
from PIL import Image  # noqa: E402
for name, pos, tgt in (("ab_A49", (1.1, -1.6, 1.9), (0.0, 0.0, 0.68)),
                       ("ab_MOD", (9.35, 1.4, 1.9), (8.25, 3.0, 0.68))):
    cam_path = f"/World/cam_{name}"
    camg = UsdGeom.Camera.Define(stage, cam_path)
    cxf = UsdGeom.Xformable(camg.GetPrim())
    pos_v = np.array(pos, float)
    f = np.array(tgt, float) - pos_v
    f /= np.linalg.norm(f)
    z = -f
    x = np.cross((0.0, 0.0, 1.0), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    import math
    m = np.column_stack([x, y, z])
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    s = math.sqrt(max(tr + 1.0, 1e-9)) * 2
    q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
         (m[1, 0] - m[0, 1]) / s)
    cxf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    op = cxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    op.Set(Gf.Quatd(q[0], Gf.Vec3d(q[1], q[2], q[3])))
    camg.CreateFocalLengthAttr(20.0)
    camg.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    cam = Camera(prim_path=cam_path, resolution=(1920, 1080))
    cam.initialize()
    for _ in range(90):
        world.step(render=True)
    rgba = cam.get_rgba()
    Image.fromarray(np.asarray(rgba)[..., :3]).save(
        f"/workspace/sortmaster_out/{name}.png")
    print(f"SAVED {name}", flush=True)
app.close()
