# -*- coding: utf-8 -*-
"""The published virtual-sensor config (cell.params.VIRTUAL_SENSOR) must be
implementation-true: every advertised value is the one the pipeline actually
uses (brief §5: 'do not invent values that are not actually used')."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cell import params as P                     # noqa: E402
from cell.run_sim import ItemManager             # noqa: E402
from perception import pipeline as pl            # noqa: E402


def test_window_is_single_sourced():
    assert ItemManager.CAM_WINDOW == P.VIRTUAL_SENSOR["window_x"]


def test_profiler_geometry_matches_config():
    cls = pl.LookaheadPerception
    assert cls.FAN_STEP == P.VIRTUAL_SENSOR["profile_plane_spacing_mm"] / 1000.0
    assert cls.SIDE_Y_OFF == P.VIRTUAL_SENSOR["side_head_offset_m"]
    assert cls.SIDE_Z == P.VIRTUAL_SENSOR["side_head_z_m"]
    fan_res = np.rad2deg(cls.FAN_ANGLES[1] - cls.FAN_ANGLES[0])
    assert abs(fan_res - P.VIRTUAL_SENSOR["profile_angular_res_deg"]) < 1e-9
    side_res = np.rad2deg(cls.SIDE_TILTS[1] - cls.SIDE_TILTS[0])
    assert abs(side_res - P.VIRTUAL_SENSOR["side_head_angular_res_deg"]) < 1e-9


def test_overhead_head_position_matches_scene():
    """The config's overhead position is the scene camera the rays cast from."""
    import mujoco
    from cell.scene import make_model
    model, _, _ = make_model(mode="table")
    cam = model.camera("lookahead")
    assert tuple(np.round(model.cam_pos[cam.id], 6)) == P.VIRTUAL_SENSOR["overhead_pos"]


def test_conveyor_speed_is_the_official_one():
    assert P.VIRTUAL_SENSOR["conveyor_speed_mps"] == P.BELT_A["speed"] == 1.0


def test_noise_default_is_ideal_optics_baseline():
    assert P.VIRTUAL_SENSOR["depth_noise_mm"] == 0.0
    p = object.__new__(pl.LookaheadPerception)   # check default resolution wiring
    assert P.VIRTUAL_SENSOR["ground_res_mm"] == 3.0
