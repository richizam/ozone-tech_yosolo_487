# -*- coding: utf-8 -*-
"""Render ConveyorBelt_A45/A46 fork modules alone: bbox + top view still."""
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
try:
    from isaacsim import SimulationApp
except ImportError:
    from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True, "width": 1280, "height": 960})

from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from isaac.asset_shells import assets_root, reference  # noqa: E402

world = World(physics_dt=1 / 240.0, rendering_dt=1 / 240.0,
              stage_units_in_meters=1.0, backend="numpy")
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(800.0)
root = assets_root()

for i, name in enumerate(("ConveyorBelt_A45", "ConveyorBelt_A46")):
    prim = reference(stage, f"/World/fork_{i}",
                     root + f"/Isaac/Props/Conveyors/{name}.usd",
                     (i * 8.0, 0.0, 0.0))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    print(f"{name}: min=({mn[0]:.2f},{mn[1]:.2f},{mn[2]:.2f}) "
          f"max=({mx[0]:.2f},{mx[1]:.2f},{mx[2]:.2f}) "
          f"size=({mx[0]-mn[0]:.2f},{mx[1]-mn[1]:.2f},{mx[2]-mn[2]:.2f})",
          flush=True)

cam_path = "/World/cam"
cam = UsdGeom.Camera.Define(stage, cam_path)
xf = UsdGeom.Xformable(cam.GetPrim())
xf.AddTranslateOp().Set(Gf.Vec3d(4.0, -1.5, 9.0))
xf.AddRotateXYZOp().Set(Gf.Vec3f(10.0, 0.0, 0.0))
world.reset()
c = Camera(prim_path=cam_path, resolution=(1280, 960))
c.initialize()
for _ in range(30):
    world.step(render=True)
rgba = c.get_rgba()
if rgba is not None and getattr(rgba, "size", 0):
    from PIL import Image
    Image.fromarray(np.asarray(rgba)[..., :3]).save("/tmp/fork_probe.png")
    print("SAVED /tmp/fork_probe.png", flush=True)
app.close()
