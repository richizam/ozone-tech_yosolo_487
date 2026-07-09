# -*- coding: utf-8 -*-
"""Strict scene audit: exact prim paths for every VISIBLE gprim intersecting
the problem regions (silver rollers under the belts, drums in cage C, the
deck), plus every light with intensity/position. Facts before fixes."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from isaacsim import SimulationApp                       # noqa: E402
app = SimulationApp({"headless": True})

from isaacsim.core.api import World                      # noqa: E402
from pxr import Usd, UsdGeom, UsdLux                     # noqa: E402

from isaac.scene_usd import SceneBuilder, load_manifest  # noqa: E402

world = World(stage_units_in_meters=1.0)
stage = world.stage
manifest = load_manifest(REPO)
SceneBuilder(stage, REPO).build(manifest)

REGIONS = {
    "ZONE_DECK":   (7.90, 8.60, 2.35, 3.65, 0.50, 0.80),
    "CAGE_C_VOL":  (9.00, 10.30, 2.30, 3.70, 0.05, 1.00),
    "BELT_A_SIDE": (2.00, 6.00, 2.55, 3.45, 0.30, 0.72),
}
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_],
                          useExtentsHint=False)
for name, (x0, x1, y0, y1, z0, z1) in REGIONS.items():
    print(f"==== {name} x[{x0},{x1}] y[{y0},{y1}] z[{z0},{z1}]", flush=True)
    n = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Gprim):
            continue
        img = UsdGeom.Imageable(prim)
        if img.ComputeVisibility() == UsdGeom.Tokens.invisible:
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
        # skip tiny slivers to keep the report readable
        if max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]) < 0.02:
            continue
        n += 1
        print(f"  {p}  aabb=({lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f})..("
              f"{hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f}) type={prim.GetTypeName()}",
              flush=True)
        if n > 120:
            print("  ...truncated", flush=True)
            break
print("==== LIGHTS", flush=True)
for prim in stage.Traverse():
    if not (prim.IsA(UsdLux.SphereLight) or prim.IsA(UsdLux.DistantLight)
            or prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.RectLight)
            or prim.IsA(UsdLux.DiskLight)):
        continue
    api = UsdLux.LightAPI(prim)
    inten = prim.GetAttribute("inputs:intensity").Get()
    xf = UsdGeom.Xformable(prim)
    mat = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    print(f"  {prim.GetPath().pathString} type={prim.GetTypeName()} "
          f"intensity={inten} pos=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})",
          flush=True)
print("AUDIT DONE", flush=True)
app.close()
