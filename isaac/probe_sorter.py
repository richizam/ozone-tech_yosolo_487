# -*- coding: utf-8 -*-
"""Find official sorter/divert/transfer conveyor assets."""
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
print("ROOT:", root, flush=True)


def ls(rel):
    res, entries = omni.client.list(root + rel)
    if res == omni.client.Result.OK:
        print(rel, "->", [e.relative_path for e in entries], flush=True)
    else:
        print(rel, "->", res, flush=True)


for rel in ("/Isaac/Props/Conveyors", "/Isaac/Props",
            "/Isaac/Environments/Simple_Warehouse/Props",
            "/NVIDIA/Assets/DigitalTwin",
            "/Isaac/Samples/DigitalTwin"):
    ls(rel)
app.close()
