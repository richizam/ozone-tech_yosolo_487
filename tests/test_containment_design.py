# -*- coding: utf-8 -*-
"""Containment-by-design invariants: the geometry that keeps routed items
inside their destinations must hold under any future re-tuning. Pure params."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cell import params as P  # noqa: E402

MAX_ITEM_H = 0.30              # tallest routed item lying flat (box_l)
TAN32 = math.tan(math.radians(32.0))

DESTS = [("C", P.CHUTE_C, P.cages_for()["C"]),
         ("D", P.CHUTE_D, P.cages_for()["D"]),
         ("REVIEW", P.CHUTE_REVIEW, P.REVIEW_PEN)]


def wall_plane(cc, cage):
    """y of the destination wall the chute crosses (the aperture wall)."""
    cy = cage["center"][1]
    hy = cage["inner"][1] / 2 + cage["wall_t"] / 2
    return cy + hy if cc["dir"] < 0 else cy - hy


@pytest.mark.parametrize("zone,chute,cage", DESTS)
def test_chute_cannot_hold_items_statically(zone, chute, cage):
    """mu < tan(slope): nothing can rest on the slope — no stall points,
    and arm-recovery drops always slide into the destination."""
    mu = float(chute["friction"].split()[0])
    assert mu < TAN32 * 0.85


@pytest.mark.parametrize("zone,chute,cage", DESTS)
def test_chute_starts_under_the_tilted_tray_lip(zone, chute, cage):
    assert chute["z0"] < P.TRAY_LIP_Z - 0.005


@pytest.mark.parametrize("zone,chute,cage", DESTS)
def test_chute_crosses_wall_inside_the_aperture(zone, chute, cage):
    """The slope passes the destination wall ABOVE the sill and BELOW the
    aperture top — a routed item enters through steel, never over it."""
    w = wall_plane(chute, cage)
    z_cross = chute["z0"] - abs(w - chute["y0"]) * TAN32
    assert z_cross > cage["sill_top"] + 0.02
    assert z_cross + MAX_ITEM_H < cage["aperture_top"] + cage["wall_h"]


@pytest.mark.parametrize("zone,chute,cage", DESTS)
def test_pad_ends_inside_destination(zone, chute, cage):
    _, pad_end = P.chute_run(chute)
    cy = cage["center"][1]
    hy = cage["inner"][1] / 2
    assert cy - hy < pad_end < cy + hy


@pytest.mark.parametrize("zone,chute,cage", DESTS)
def test_aperture_fits_chute(zone, chute, cage):
    assert cage["aperture_w"] >= chute["width"] - 0.02


@pytest.mark.parametrize("zone,chute,cage", DESTS)
def test_no_structure_over_the_chute_extraction_path(zone, chute, cage):
    """The chutes are OPEN from above (no hood): with the deep-drop chute the
    aperture is crossed at z ~0.22 and no fly-out window exists, so every
    possible chute jam point stays vertically extractable by the arm."""
    # (kept as a design statement: the params no longer define a HOOD)
    assert not hasattr(P, "HOOD")


def test_cages_do_not_overlap_each_other_or_the_arm():
    cg = P.cages_for()
    spans = {}
    for z, c in cg.items():
        hx = c["inner"][0] / 2 + c["wall_t"]
        spans[z] = (c["center"][0] - hx, c["center"][0] + hx)
    assert spans["C"][1] < spans["D"][0] - 0.02
    ax, ay = P.ARM_BASE_ISAAC["sorter"]
    for z, c in cg.items():
        hx = c["inner"][0] / 2 + c["wall_t"]
        hy = c["inner"][1] / 2 + c["wall_t"]
        dx = max(abs(ax - c["center"][0]) - hx, 0.0)
        dy = max(abs(ay - c["center"][1]) - hy, 0.0)
        assert math.hypot(dx, dy) > 0.17          # pedestal r=0.15 + margin


def test_arm_reaches_chute_mouths_and_places():
    ax, ay = P.ARM_BASE_ISAAC["sorter"]
    for z in ("C", "D"):
        mouth = (P.STATIONS[z]["x"], 2.70)
        assert math.hypot(ax - mouth[0], ay - mouth[1]) < 1.0
        px, py = P.PLACE_BY_MODE["sorter"][z]["xy"]
        assert math.hypot(ax - px, ay - py) < 0.9


def test_arm_grasp_clamp_stays_clear_of_the_tray_sweep():
    """Tilted tray lips sweep to y ~2.756; the arm never grasps north of
    the clamp line."""
    tray_south_lip_y = P.SORTER["y"] - (P.SORTER["tray_w"] / 2) * math.cos(
        math.radians(P.SORTER["tilt_deg"]))
    assert P.ARM_GRASP_Y_MAX < tray_south_lip_y - 0.10


@pytest.mark.parametrize("zone,chute,cage", DESTS)
def test_mouth_chamfer_clears_the_tilting_tray(zone, chute, cage):
    """The 59-deg mouth chamfer must stay clear of BOTH swept arcs of the
    tilting tray on its side: the bare plate edge (radius tray_w/2) and the
    lip tip (edge + lip_h perpendicular). Checked at tilt*(1+2*noise)."""
    S = P.SORTER
    r_e = S["tray_w"] / 2
    lip = S["lip_h"]
    th_max = math.radians(S["tilt_deg"] * (1 + 2 * S["noise_frac"]))
    ch_top_dy = abs(chute["y0"] - S["y"]) - chute["chamfer_top_inset"]
    ch_top_z = chute["z0"] + chute["chamfer_rise"]
    # plate edge arc: (dy, z) = (r_e cos t, pivot - r_e sin t)
    # clearance at the chamfer-top ordinate, and at max tilt
    worst = 1e9
    for i in range(200):
        th = th_max * i / 199.0
        dy_e = r_e * math.cos(th)
        z_e = S["pivot_z"] - r_e * math.sin(th)
        dy_t = r_e * math.cos(th) + lip * math.sin(th)
        z_t = S["pivot_z"] - r_e * math.sin(th) + lip * math.cos(th)
        for dy, z in ((dy_e, z_e), (dy_t, z_t)):
            worst = min(worst, math.hypot(max(0.0, ch_top_dy - dy),
                                          max(0.0, z - ch_top_z)))
    assert worst > 0.007, f"{zone}: chamfer within {worst*1000:.1f} mm of sweep"
    # mouth cheeks: top face clears both swept arcs across the cheek span
    ck_top = chute["z0"] + chute["cheek_h"]
    ck_dy0 = abs(chute["y0"] - S["y"]) + chute["cheek_inset"]
    for i in range(200):
        th = th_max * i / 199.0
        for dy, z in ((r_e * math.cos(th),
                       S["pivot_z"] - r_e * math.sin(th)),
                      (r_e * math.cos(th) + lip * math.sin(th),
                       S["pivot_z"] - r_e * math.sin(th) + lip * math.cos(th))):
            if ck_dy0 <= dy <= ck_dy0 + chute["cheek_len"]:
                assert z > ck_top + 0.008, \
                    f"{zone}: cheek within {(z-ck_top)*1000:.1f} mm of sweep"
    # the chamfer face is too steep to rest on (steeper than repose for any mu)
    face = math.degrees(math.atan2(chute["chamfer_rise"] + 0.003,
                                   chute["chamfer_run"]))
    assert face > 50.0
    # bridging cure retained: >= 25 mm free fall from the tilted lip
    assert P.TRAY_LIP_Z - ch_top_z >= 0.025


def test_catch_pan_clears_the_tilt_sweep_and_fall_corridor():
    """The debris pan lives in the dead zone under the top run. It must stay
    (a) below the FULL-TILT tray plane z = pivot_z - tan(tilt)*|dy| with
    margin (tilt gain noise inflates the angle ~1%), and (b) north of the
    discharge fall corridor so a dribbling small item never deflects off a
    chute mouth (the 9 mm pen floor-drop of the 0.33 x 0.515 pan)."""
    S = P.SORTER
    tan_t = math.tan(math.radians(S["tilt_deg"] * 1.02))
    tray_plane_at_pan_edge = S["pivot_z"] - tan_t * S["pan_y_half"]
    assert S["pan_z_top"] < tray_plane_at_pan_edge - 0.015
    # fall corridor: the tilted lip hangs at |dy| ~ (tray_w/2)*cos(tilt);
    # freight leaves it moving outward — the pan edge stays inboard of it
    lip_dy = (S["tray_w"] / 2) * math.cos(math.radians(S["tilt_deg"]))
    assert S["pan_y_half"] < lip_dy - 0.02
    # still wide enough to catch the induction-gap fall zone under belt A's
    # centreline (a knifed card tumbles, but lands near |dy| < 0.15)
    assert S["pan_y_half"] >= 0.18
    # and below the shuttle undersides on the top run
    assert S["pan_z_top"] < 0.50


def test_everything_inside_the_work_zone():
    zone_x, zone_y = P.ZONE
    for z, c in {**P.cages_for(), "REVIEW": P.REVIEW_PEN}.items():
        hx = c["inner"][0] / 2 + c["wall_t"]
        hy = c["inner"][1] / 2 + c["wall_t"]
        assert 0 < c["center"][0] - hx and c["center"][0] + hx < zone_x
        assert 0 < c["center"][1] - hy and c["center"][1] + hy < zone_y
    assert P.SORTER["x_east"] + 0.35 < zone_x    # east end module fits


def test_review_pen_clears_fixed_belt_b():
    """The REVIEW pen's west wall must stand clear, east of belt B."""
    rp = P.REVIEW_PEN
    hx = rp["inner"][0] / 2 + rp["wall_t"]
    b_east = P.BELT_B["cx"] + P.BELT_B["width"] / 2
    assert rp["center"][0] - hx > b_east + 0.05
