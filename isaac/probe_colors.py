# -*- coding: utf-8 -*-
"""Color forensics: every visible gprim in the C-bin camera region with its
resolved displayColor and bound-material diffuseColor. Finds what renders
pale/blown-out."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from isaacsim import SimulationApp                       # noqa: E402
app = SimulationApp({"headless": True})

from isaacsim.core.api import World                      # noqa: E402
from pxr import Usd, UsdGeom, UsdShade                   # noqa: E402

from isaac.scene_usd import SceneBuilder, load_manifest  # noqa: E402

world = World(stage_units_in_meters=1.0)
stage = world.stage
SceneBuilder(stage, REPO).build(load_manifest(REPO))

x0, x1, y0, y1, z0, z1 = 8.60, 10.20, 2.25, 3.75, 0.25, 1.30
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_],
                          useExtentsHint=False)
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Gprim):
        continue
    if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
        continue
    p = prim.GetPath().pathString
    if "/World/items/" in p:
        continue
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        continue
    lo, hi = box.GetMin(), box.GetMax()
    if (hi[0] < x0 or lo[0] > x1 or hi[1] < y0 or lo[1] > y1
            or hi[2] < z0 or lo[2] > z1):
        continue
    if max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]) < 0.05:
        continue
    dc = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
    dc = tuple(round(float(v), 2) for v in dc[0]) if dc else None
    op = UsdGeom.Gprim(prim).GetDisplayOpacityAttr().Get()
    op = round(float(op[0]), 2) if op else 1.0
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    mdc = None
    if mat:
        sh = UsdShade.Shader(stage.GetPrimAtPath(
            str(mat.GetPath()) + "/pbr"))
        if sh:
            inp = sh.GetInput("diffuseColor")
            v = inp.Get() if inp else None
            if v is not None:
                mdc = tuple(round(float(c), 2) for c in v)
    flag = ""
    ref = mdc or dc
    if ref and min(ref) > 0.45:
        flag = "  <-- PALE"
    print(f"{p}  disp={dc} op={op} mat={mdc}{flag}", flush=True)

# ---- render the SAME stage from the routing camera pose: if this image
# shows a cream cage while the values above are dark, the wash is optical;
# if it shows a dark cage, the runtime build differs from this probe.
import numpy as np
from isaacsim.sensors.camera import Camera
from pxr import UsdLux
cam_path = "/World/cams/routing"
world.reset()
cam = Camera(prim_path=cam_path, resolution=(1280, 720))
cam.initialize()
for _ in range(90):
    world.step(render=True)
rgba = cam.get_rgba()
from PIL import Image
Image.fromarray(np.asarray(rgba)[..., :3]).save(
    "/workspace/sortmaster_out/probe_colors_view.png")
print("PROBE VIEW SAVED", flush=True)

print("COLOR PROBE DONE", flush=True)
app.close()
