# -*- coding: utf-8 -*-
"""Render a gallery of official conveyor assets (pick shapes by eye) and
list sensor/camera prop paths."""
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

import omni.client  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402

try:
    from isaacsim.storage.native import get_assets_root_path
except ImportError:
    from isaacsim.core.utils.nucleus import get_assets_root_path

root = get_assets_root_path()
for rel in ("/Isaac/Sensors", "/Isaac/Props/Camera", "/Isaac/Props/Mounts",
            "/Isaac/Robots/UniversalRobots/ur10e"):
    res, entries = omni.client.list(root + rel)
    print(rel, "->", [e.relative_path for e in entries][:30], flush=True)

world = World(physics_dt=1 / 240.0, rendering_dt=1 / 240.0,
              stage_units_in_meters=1.0, backend="numpy")
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(900.0)

ids = list(range(1, 35)) + [37, 38, 39, 40, 41]
cols = 8
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_])
for i, aid in enumerate(ids):
    r, c = divmod(i, cols)
    path = f"/World/g_{aid:02d}"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(
        f"{root}/Isaac/Props/Conveyors/ConveyorBelt_A{aid:02d}.usd")
    xf = UsdGeom.Xformable(prim)
    tr = next((op for op in xf.GetOrderedXformOps()
               if op.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
    (tr or xf.AddTranslateOp()).Set(Gf.Vec3d(c * 4.5, -r * 4.5, 0))
    # disable physics inside
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)

cam_path = "/World/cam"
cam_prim = UsdGeom.Camera.Define(stage, cam_path)
xf = UsdGeom.Xformable(cam_prim.GetPrim())
xf.AddTranslateOp().Set(Gf.Vec3d(16.0, -9.0, 34.0))
cam_prim.CreateFocalLengthAttr(14.0)
cam_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))

world.reset()
cam = Camera(prim_path=cam_path, resolution=(1920, 1080))
cam.initialize()
for _ in range(90):
    world.step(render=True)
rgba = cam.get_rgba()
from PIL import Image  # noqa: E402
Image.fromarray(np.asarray(rgba)[..., :3]).save("/tmp/sortmaster_out/gallery.png")
print("GALLERY SAVED", flush=True)

# print bboxes for sizing
for aid in ids[:12]:
    p = stage.GetPrimAtPath(f"/World/g_{aid:02d}")
    b = bbox_cache.ComputeWorldBound(p).ComputeAlignedRange()
    mn, mx = b.GetMin(), b.GetMax()
    print(f"A{aid:02d} dims=({mx[0]-mn[0]:.2f},{mx[1]-mn[1]:.2f},"
          f"{mx[2]-mn[2]:.2f}) top={mx[2]:.2f}", flush=True)
app.close()
