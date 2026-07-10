# -*- coding: utf-8 -*-
"""Tilt-tray sorter design invariants: the numbers that make the executive
physically correct must hold under any future re-tuning. Pure params — no
simulator needed."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cell import params as P  # noqa: E402

MAX_FOOTPRINT = 0.50           # largest official footprint edge (pouf 0.489)
MAX_MU_ITEM = 0.95             # stickiest item (soft sack)


def test_carrier_pitch_exceeds_largest_footprint():
    assert P.SORTER["pitch"] >= MAX_FOOTPRINT + 0.08


def test_tray_carries_largest_item():
    assert P.SORTER["tray_w"] >= 0.60          # pouf 0.489 + margin
    assert P.SORTER["tray_l"] >= 0.55


def test_loop_is_integer_carriers():
    loop = 2 * (P.SORTER["x_east"] - P.SORTER["x_west"])
    n = loop / P.SORTER["pitch"]
    assert abs(n - round(n)) < 1e-9
    assert P.SORTER["n_carriers"] == round(n)


def test_tilt_guarantees_discharge_for_every_item():
    """Gravity slide on the tilted tray must beat the tray-item friction
    pair (combine=min -> the tray's own mu) with wide margin, even for the
    mu=0.95 sack: tan(tilt) >> mu_tray."""
    mu = P.SORTER["tray_mu"][0]
    tilt = math.radians(P.SORTER["tilt_deg"])
    assert math.tan(tilt) > mu * 2.0
    # and the dish never cancels the discharge slope
    assert P.SORTER["tilt_deg"] - P.SORTER["dish_deg"] \
        > math.degrees(math.atan(mu)) + 10.0


def test_tray_friction_holds_items_during_carry():
    """Constant-speed carry needs no friction, but induction slip must damp
    within the tray half-length: slip = dv^2 / (2*mu*g) << tray_l/2."""
    dv = P.BELT_A["speed"] - P.SORTER["v_mps"]
    slip = dv ** 2 / (2 * P.SORTER["tray_mu"][1] * 9.81)
    assert slip < P.SORTER["tray_l"] / 2 - 0.10


def test_b_incline_holds_boxes():
    """Every B item is non-round by rule; the incline must stay well under
    the friction angle of the least grippy B pairing (~0.5 avg combine)."""
    assert P.B_INCLINE_DEG < math.degrees(math.atan(0.45))
    assert P.B_CONNECT["z_top1"] == P.BELT_B["top"]


def test_station_order_and_track_bounds():
    xs = {k: v["x"] for k, v in P.STATIONS.items()}
    assert P.SORTER["x_west"] < xs["C"] < xs["B"] < xs["D"] < xs["REVIEW"]
    assert xs["REVIEW"] + 0.25 <= P.SORTER["occupied_callout_x"]
    assert P.SORTER["occupied_callout_x"] < P.SORTER["x_east"]
    # B station must sit on the FIXED belt B axis
    assert xs["B"] == P.BELT_B["cx"]


def test_trigger_leads_positive_and_within_pitch():
    for st in P.STATIONS.values():
        assert 0.05 < st["trigger_lead_m"] < P.SORTER["pitch"]


def test_first_station_leaves_settle_distance():
    """The item lands at ~INDUCT.land_x and must have settled its induction
    slip before the earliest possible tilt command."""
    first_trig = P.STATIONS["C"]["x"] - P.STATIONS["C"]["trigger_lead_m"]
    dv = P.BELT_A["speed"] - P.SORTER["v_mps"]
    settle_t = dv / (P.SORTER["tray_mu"][1] * 9.81)
    settle_dist = P.SORTER["v_mps"] * settle_t
    assert first_trig > P.INDUCT["land_x"] + settle_dist


def test_knife_nose_clears_tray_lips():
    knife_bottom = P.BELT_A["top"] - P.BELT_A["knife_t"]
    lips_top = P.SORTER["tray_top"] + P.SORTER["lip_h"]
    assert knife_bottom - lips_top >= 0.010


def test_induction_drop_is_gentle():
    drop = P.BELT_A["top"] - P.SORTER["tray_top"]
    assert 0.03 <= drop <= 0.10
    assert abs(P.INDUCT["flight_s"] - math.sqrt(2 * drop / 9.81)) < 0.02


def test_verdict_commits_before_the_escapement():
    """The longest official item's centre must cross the window exit before
    its nose presses the escapement blade (look-ahead classification)."""
    longest_half = 0.489 / 2                     # pouf
    win1 = P.VIRTUAL_SENSOR["window_x"][1]
    assert P.BELT_A["gate_x"] - longest_half > win1 + 0.02


def test_inter_tray_gap_smaller_than_smallest_item():
    """An 11 mm cube can never fall between trays."""
    gap = P.SORTER["pitch"] - P.SORTER["tray_l"]
    assert gap < 0.011


def test_macro_certification_floor_passes_11mm():
    """Dual-range metrology: the macro head's guard band certifies an 11 mm
    cube as sortable (>10 mm) while 10 mm exact stays conservatively C."""
    floor = P.LIMIT_MIN_MM + P.VIRTUAL_SENSOR["guard_macro_mm"]
    assert floor < 11.0
    assert floor > 10.0


def test_return_run_clears_structures():
    """Under-deck return trays pass beneath the chute undersides."""
    tray_top_return = P.SORTER["return_z"] + 0.05 + P.SORTER["lip_h"]
    chute_underside = P.CHUTE["z0"] - 0.04       # at the train edge
    assert tray_top_return < chute_underside
