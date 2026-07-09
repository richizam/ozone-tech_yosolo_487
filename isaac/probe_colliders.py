# -*- coding: utf-8 -*-
"""Collider census: build the full scene headlessly and list EVERY
collision-enabled prim whose world AABB intersects a query region.
Diagnoses hidden contact partners (box_l chute wedge).

  /isaac-sim/python.sh isaac/probe_colliders.py 8.60 9.25 2.60 3.40 0.35 1.05
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

x0, x1, y0, y1, z0, z1 = (float(v) for v in sys.argv[1:7]) if len(sys.argv) >= 7 \
    else (8.60, 9.25, 2.60, 3.40, 0.35, 1.05)

from isaacsim import SimulationApp                       # noqa: E402
app = SimulationApp({"headless": True})

from isaacsim.core.api import World                      # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics                 # noqa: E402

from isaac.scene_usd import SceneBuilder, load_manifest  # noqa: E402

world = World(stage_units_in_meters=1.0)
stage = world.stage
manifest = load_manifest(REPO)
builder = SceneBuilder(stage, REPO)
info = builder.build(manifest)

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_], useExtentsHint=False)
print(f"[census] region x[{x0},{x1}] y[{y0},{y1}] z[{z0},{z1}]", flush=True)
n = 0
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    api = UsdPhysics.CollisionAPI(prim)
    en = api.GetCollisionEnabledAttr().Get()
    if en is False:
        continue
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        continue
    lo, hi = box.GetMin(), box.GetMax()
    if (hi[0] < x0 or lo[0] > x1 or hi[1] < y0 or lo[1] > y1
            or hi[2] < z0 or lo[2] > z1):
        continue
    if "/World/items/" in prim.GetPath().pathString:
        continue
    n += 1
    print(f"[census] {prim.GetPath().pathString}  "
          f"aabb=({lo[0]:.3f},{lo[1]:.3f},{lo[2]:.3f})..("
          f"{hi[0]:.3f},{hi[1]:.3f},{hi[2]:.3f})", flush=True)
print(f"[census] {n} colliders intersect the region", flush=True)
app.close()
