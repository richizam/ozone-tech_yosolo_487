# -*- coding: utf-8 -*-
"""Containment-by-design invariants: the geometry that keeps routed items
inside their cages must hold under any future re-tuning."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cell import params as P  # noqa: E402

MAX_ITEM_H = 0.30              # tallest official test item lying flat (box_l)
MAX_ITEM_LEN = 0.50            # official max item edge
# Singulated freight rides FLAT (the vision station measures one flat item
# per window). The official worst-case box is 450x320x320 mm -> resting
# HEIGHT 320 mm; the 500 mm edge is always horizontal, never on end. The
# exit gate lifts to clear the item's HEIGHT, so the invariant is
# height + margin (the old 500 mm-edge stroke of 0.54 m over-lifted and the
# panel "flew" above its guillotine frame). Every routed item passes under
# the 0.42 m gate (box_l 0.30, helmet 0.28 tall — verified).
MAX_ITEM_HEIGHT = 0.32        # official max resting height (320 mm middle dim)


def _slope_angle(chute, axis):
    if axis == "x":
        run = chute["x1"] - chute["x0"]
    else:
        run = chute["y0"] - chute["y1"]
    return float(np.arctan2(chute["z0"] - chute["z1"], run))


@pytest.mark.parametrize("chute,axis", [(P.CHUTE_C, "x"), (P.CHUTE_D, "y")])
def test_chute_cannot_hold_items_statically(chute, axis):
    """mu < tan(slope): nothing can rest on the slope — no stall points,
    and arm-recovery drops always slide into the cage."""
    ang = _slope_angle(chute, axis)
    mu = float(chute["friction"].split()[0])
    assert mu < np.tan(ang) * 0.85, "slope must never statically hold an item"


@pytest.mark.parametrize("chute,axis", [(P.CHUTE_C, "x"), (P.CHUTE_D, "y")])
def test_hood_clears_riding_items(chute, axis):
    """Normal clearance under the hood exceeds the tallest item's normal
    height on the slope (it can never wedge under the cover)."""
    ang = _slope_angle(chute, axis)
    need = MAX_ITEM_H / np.cos(ang) + 0.05
    assert P.HOOD["clearance"] >= need


def test_gate_travel_clears_max_item():
    """An open gate passes the tallest flat-riding item with margin."""
    assert P.GATES["travel"] >= MAX_ITEM_HEIGHT + 0.06


def test_cage_apertures_fit_chutes():
    for zone, chute in (("C", P.CHUTE_C), ("D", P.CHUTE_D)):
        cage = P.CAGES["table"][zone]
        assert cage["aperture_w"] >= chute["width"] + 0.10


def test_pads_end_inside_cages():
    cage_c = P.CAGES["table"]["C"]
    assert P.CHUTE_C["pad_x1"] < cage_c["center"][0] + cage_c["inner"][0] / 2
    cage_d = P.CAGES["table"]["D"]
    assert P.CHUTE_D["pad_y1"] > cage_d["center"][1] - cage_d["inner"][1] / 2


def test_models_build_with_gates_and_signage():
    mujoco = pytest.importorskip("mujoco")
    from cell.scene import make_model
    model, _, _ = make_model("table")
    for z in "BCD":
        assert model.actuator(f"ga{z}").id >= 0
        assert model.joint(f"gj{z}").id >= 0
        assert model.geom(f"gate{z}_panel").id >= 0
    for z in "CD":                                          # hoods on the chute exits
        assert model.geom(f"hood{z}").id >= 0
        assert model.geom(f"hood{z}_brow").id >= 0
        assert model.geom(f"cage{z}_skirt").id >= 0
        assert model.geom(f"chute{z}_pad").id >= 0
    assert model.actuator("ea").id >= 0                     # escapement gate flag
    assert model.geom("sign_a_infeed").id >= 0
    assert model.geom("tower_r").id >= 0
    # arm baseline still builds
    model_arm, _, _ = make_model("arm")
    assert model_arm.actuator("ea").id >= 0
