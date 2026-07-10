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
    """Chute-mouth picks + route-correct cage places, worst-case heights.
    The arm is the exception handler: it recovers C/D chute snags only."""
    place = params.PLACE_BY_MODE["sorter"]
    cases = [
        # picks on the chute bodies (slope z 0.16..0.43 + item top)
        (params.STATIONS["C"]["x"], 2.60, 0.43 + 0.05),   # thin snag high on C chute
        (params.STATIONS["C"]["x"], 2.45, 0.30 + 0.30),   # box_l mid-chute
        (params.STATIONS["D"]["x"], 2.60, 0.43 + 0.09),   # bottle on D chute
        (params.STATIONS["D"]["x"], 2.35, 0.20 + 0.28),   # helmet low on D chute
        # route-correct placements over the cage walls
        (place["C"]["xy"][0], place["C"]["xy"][1], params.CAGE_WALL_TOP + 0.30 + 0.06),
        (place["D"]["xy"][0], place["D"]["xy"][1], params.CAGE_WALL_TOP + 0.30 + 0.06),
        (params.ARM["base"][0], params.ARM["base"][1] + 0.55, params.LIFT_Z),  # lift
    ]
    for x, y, z in cases:
        q = ik(np.array([x, y, z]))
        tcp, wrist = fk(q)
        assert np.allclose(tcp, [x, y, z], atol=1e-9)
        # elbow-up: the wrist rides the tool length above the TCP and always
        # clears the chute surface under it (z0=0.43 is the slope maximum)
        assert wrist[2] >= z + 0.20
        assert wrist[2] > 0.43 + 0.20


def test_unwrap_yaw_takes_shortest_path():
    assert abs(unwrap_yaw(np.deg2rad(90), np.deg2rad(-146)) - np.deg2rad(-270)) < 1e-9
    assert abs(unwrap_yaw(np.deg2rad(-170), np.deg2rad(170)) - np.deg2rad(190)) < 1e-9


def test_out_of_reach_raises():
    bx, by = params.ARM["base"]
    with pytest.raises(ValueError):
        ik(np.array([bx + 2.0, by, 1.0]))
