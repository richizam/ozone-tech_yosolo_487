# -*- coding: utf-8 -*-
"""IK <-> FK consistency + reachability of all task points."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cell.arm_ik import fk, ik, unwrap_yaw  # noqa: E402
from cell import params  # noqa: E402


def test_ik_fk_roundtrip_random_targets():
    rng = np.random.default_rng(7)
    bx, by = params.ARM["base"]
    n_ok = 0
    for _ in range(500):
        # sample targets in an annulus around the base at plausible heights
        ang = rng.uniform(-np.pi, np.pi)
        r = rng.uniform(0.35, 1.15)
        z = rng.uniform(0.72, 1.30)
        target = np.array([bx + r * np.cos(ang), by + r * np.sin(ang), z])
        try:
            q = ik(target)
        except ValueError:
            continue
        tcp, _ = fk(q)
        assert np.allclose(tcp, target, atol=1e-9)
        n_ok += 1
    assert n_ok > 450  # nearly all sampled targets must be reachable


def test_all_task_points_reachable():
    """Pick, B place, C drop, D drop — including the worst-case item heights."""
    top = params.BELT_A["top"]
    cases = [
        (7.9, 3.0, top + 0.009),            # pen pick (thinnest)
        (7.85, 3.0, top + 0.264),           # pouf pick (tallest)
        (params.PLACE["B"]["xy"][0], params.PLACE["B"]["xy"][1], top + 0.202),  # box_s onto B
        (params.PLACE["C"]["xy"][0], params.PLACE["C"]["xy"][1], params.CAGE_WALL_TOP + 0.264 + 0.05),
        (params.PLACE["D"]["xy"][0], params.PLACE["D"]["xy"][1], params.CAGE_WALL_TOP + 0.282 + 0.05),
        (7.9, 3.0, params.LIFT_Z),          # lift over accumulator
    ]
    for x, y, z in cases:
        q = ik(np.array([x, y, z]))
        tcp, wrist = fk(q)
        assert np.allclose(tcp, [x, y, z], atol=1e-9)
        # elbow-up: wrist stays above the belt line
        assert wrist[2] > 0.85


def test_unwrap_yaw_takes_shortest_path():
    assert abs(unwrap_yaw(np.deg2rad(90), np.deg2rad(-146)) - np.deg2rad(-270)) < 1e-9
    assert abs(unwrap_yaw(np.deg2rad(-170), np.deg2rad(170)) - np.deg2rad(190)) < 1e-9


def test_out_of_reach_raises():
    bx, by = params.ARM["base"]
    with pytest.raises(ValueError):
        ik(np.array([bx + 2.0, by, 1.0]))
