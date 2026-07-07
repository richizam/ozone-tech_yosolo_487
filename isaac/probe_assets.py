# -*- coding: utf-8 -*-
"""List official Isaac Sim assets relevant to the cell (robots, conveyors,
warehouse props, sensors) so visual shells use real assets, not primitives."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

try:
    from isaacsim import SimulationApp
except ImportError:
    from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

import omni.client  # noqa: E402

try:
    from isaacsim.storage.native import get_assets_root_path
except ImportError:
    from isaacsim.core.utils.nucleus import get_assets_root_path

root = get_assets_root_path()
print("ASSETS_ROOT:", root, flush=True)


def ls(rel, depth=1, prefix=""):
    url = root + rel
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        print(f"{prefix}{rel}  -> {res}", flush=True)
        return
    names = [e.relative_path for e in entries]
    print(f"{prefix}{rel}: {names[:40]}", flush=True)
    if depth > 1:
        for n in names[:12]:
            ls(rel + "/" + n, depth - 1, prefix + "  ")


for path in ("/Isaac/Robots", "/Isaac/Robots/UniversalRobots",
             "/Isaac/Robots/UniversalRobots/ur10e",
             "/Isaac/Robots/Fanuc", "/Isaac/Robots/Kuka",
             "/Isaac/Props", "/Isaac/Props/Conveyors",
             "/Isaac/Environments", "/Isaac/Environments/Simple_Warehouse",
             "/Isaac/Sensors/Intel/RealSense"):
    ls(path)

app.close()
