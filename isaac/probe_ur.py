# -*- coding: utf-8 -*-
"""Calibrate the UR10e joint-mapping: set known drive targets, print the real
flange world position, render each pose."""
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

try:
    from isaacsim import SimulationApp
except ImportError:
    from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import omni.client  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from isaac.asset_shells import assets_root, ur10e_arm  # noqa: E402

root = assets_root()
res, entries = omni.client.list(root + "/Isaac/Sensors/RealSense")
print("RealSense dir:", [e.relative_path for e in entries][:20], flush=True)

world = World(physics_dt=1 / 240.0, rendering_dt=1 / 240.0,
              stage_units_in_meters=1.0, backend="numpy")
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(700.0)
floor = UsdGeom.Cube.Define(stage, "/World/floor")
floor.CreateSizeAttr(2.0)
xf = UsdGeom.Xformable(floor.GetPrim())
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
xf.AddScaleOp().Set(Gf.Vec3f(3, 3, 0.05))

info = ur10e_arm(stage, (0.0, 0.0), pedestal_h=0.65)
print("UR info:", info, flush=True)

cam_path = "/World/cam"
camp = UsdGeom.Camera.Define(stage, cam_path)
cxf = UsdGeom.Xformable(camp.GetPrim())
cxf.AddTranslateOp().Set(Gf.Vec3d(2.6, -2.6, 1.6))
# look at origin: -Z toward target, +Y up-ish
import math  # noqa: E402
from isaac.scene_usd import mat_to_quat  # noqa: E402
fwd = np.array([-2.6, 2.6, -0.9]); fwd /= np.linalg.norm(fwd)
right = np.cross(fwd, np.array([0, 0, 1.0])); right /= np.linalg.norm(right)
up = np.cross(right, fwd)
R = np.column_stack([right, up, -fwd])
w, x, y, z = mat_to_quat(R)
op = cxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
camp.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))

world.reset()
cam = Camera(prim_path=cam_path, resolution=(1280, 720))
cam.initialize()

attrs = {}
for nm, jp in info["joints"].items():
    prim = stage.GetPrimAtPath(jp)
    a = prim.GetAttribute("drive:angular:physics:targetPosition")
    if not a or not a.IsValid():
        from pxr import UsdPhysics
        drv = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(1.0e5)
        drv.CreateDampingAttr(1.0e4)
        a = prim.GetAttribute("drive:angular:physics:targetPosition")
    attrs[nm] = a
print("drive attrs ok:", sorted(attrs), flush=True)

flange_prim = stage.GetPrimAtPath(info["flange"])
print("flange prim:", info["flange"], flush=True)


def set_pose(vals_deg):
    for nm, v in vals_deg.items():
        attrs[nm].Set(float(v))


def flange_pos():
    m = UsdGeom.Xformable(flange_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    return (round(m[3][0], 4), round(m[3][1], 4), round(m[3][2], 4))


from PIL import Image  # noqa: E402
POSES = {
    "zero": dict(shoulder_pan_joint=0, shoulder_lift_joint=0, elbow_joint=0,
                 wrist_1_joint=0, wrist_2_joint=0, wrist_3_joint=0),
    "elbow90": dict(shoulder_pan_joint=0, shoulder_lift_joint=-90,
                    elbow_joint=90, wrist_1_joint=-90, wrist_2_joint=-90,
                    wrist_3_joint=0),
    "reach": dict(shoulder_pan_joint=90, shoulder_lift_joint=-45,
                  elbow_joint=60, wrist_1_joint=-105, wrist_2_joint=-90,
                  wrist_3_joint=0),
}
for name, pose in POSES.items():
    set_pose(pose)
    for _ in range(240):
        world.step(render=False)
    for _ in range(6):
        world.step(render=True)
    print(f"POSE {name}: flange={flange_pos()}", flush=True)
    rgba = cam.get_rgba()
    Image.fromarray(np.asarray(rgba)[..., :3]).save(
        f"/tmp/sortmaster_out/ur_{name}.png")
print("UR PROBE DONE", flush=True)
app.close()
