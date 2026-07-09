# -*- coding: utf-8 -*-
"""Close-up gallery of conveyor assets A42-A49 (unknown tail of the official
set) — hunting for a real sorter/diverter/transfer module for the ARB deck."""
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
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1000.0)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(1500.0)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 25, 0))

ids = [42, 43, 44, 45, 46, 47, 48, 49]
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_])
for i, aid in enumerate(ids):
    r, c = divmod(i, 4)
    path = f"/World/g_{aid:02d}"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(
        f"{root}/Isaac/Props/Conveyors/ConveyorBelt_A{aid:02d}.usd")
    xf = UsdGeom.Xformable(prim)
    tr = next((op for op in xf.GetOrderedXformOps()
               if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
    (tr or xf.AddTranslateOp()).Set(Gf.Vec3d(c * 5.5, -r * 6.5, 0))
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)

cam_path = "/World/cam"
cam_prim = UsdGeom.Camera.Define(stage, cam_path)
xf = UsdGeom.Xformable(cam_prim.GetPrim())
xf.AddTranslateOp().Set(Gf.Vec3d(8.5, -13.5, 16.0))
xf.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 0.0))
cam_prim.CreateFocalLengthAttr(16.0)
cam_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))

world.reset()
cam = Camera(prim_path=cam_path, resolution=(1920, 1080))
cam.initialize()
for _ in range(120):
    world.step(render=True)
rgba = cam.get_rgba()
from PIL import Image  # noqa: E402
Image.fromarray(np.asarray(rgba)[..., :3]).save(
    "/workspace/sortmaster_out/gallery_tail.png")
print("GALLERY SAVED", flush=True)
for aid in ids:
    p = stage.GetPrimAtPath(f"/World/g_{aid:02d}")
    b = bbox_cache.ComputeWorldBound(p).ComputeAlignedRange()
    mn, mx = b.GetMin(), b.GetMax()
    print(f"A{aid:02d} dims=({mx[0]-mn[0]:.2f},{mx[1]-mn[1]:.2f},"
          f"{mx[2]-mn[2]:.2f}) top={mx[2]:.2f}", flush=True)
app.close()
