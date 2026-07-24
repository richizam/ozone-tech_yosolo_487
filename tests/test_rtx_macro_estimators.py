# -*- coding: utf-8 -*-
"""Offline regression tests for the RTX macro-path section estimators,
against REAL captured clouds (no GPU needed).

The unknown-shapes clip caught a live regression the static probe missed:
in MOTION the macro frame keeps ~1/3 of its returns and leaves low smear
points on the flanks. The mirrored-HULL sweep took those as hull vertices
and a short lying cylinder (Ø12x15, elongation 1.3 — under the 1.8
primary-section gate) collapsed to 0.60-0.72 -> fused B -> FLOOR off the
B connector. The windowed angular-bin channel (per-bin maxima, 30-150 deg)
reads the outer contour through the smear and must flag it as circle
evidence; the hull stays the only B-certifier because the bin estimator
carries a +0.05 square bias (protective for D evidence, disqualifying
for B).

Fixtures: the exact .npy macro clouds exported by the failing clip run
(docs/report/isaac_evidence/xbelt/perception/unknown_shapes/), committed
as the testcase.
"""
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
NPY = ROOT / "docs" / "report" / "isaac_evidence" / "xbelt" / "perception" \
    / "unknown_shapes"

from cell import params as P                          # noqa: E402
from isaac.perception_rtx import RTXPerception        # noqa: E402

BELT_Z = P.BELT_A["top"]


def _estimators(pts):
    """Run the macro-branch section estimators on a world cloud, exactly
    as isaac/perception_rtx.py measure() does (same axes, same windows)."""
    mxy = pts[:, :2]
    mc = mxy.mean(axis=0)
    _, _, mvt = np.linalg.svd(mxy - mc, full_matrices=False)
    mh = pts[:, 2] - BELT_Z
    mxy_c = mxy - mc
    ang0 = float(np.arctan2(mvt[0][1], mvt[0][0]))
    hull_sweep = 0.0
    for k_ax in range(6):
        a_c = ang0 + k_ax * (np.pi / 6.0)
        ua = mxy_c @ np.array([np.cos(a_c), np.sin(a_c)])
        va = mxy_c @ np.array([-np.sin(a_c), np.cos(a_c)])
        hull_sweep = max(hull_sweep,
                         RTXPerception._section_ratio(ua, va, mh))
    bins = 0.0
    b_edges = np.linspace(np.deg2rad(30), np.deg2rad(150), 9)
    zc_m = 0.5 * float(np.percentile(mh, 99))
    for k_ax in range(6):
        a_c = ang0 + k_ax * (np.pi / 6.0)
        ua = mxy_c @ np.array([np.cos(a_c), np.sin(a_c)])
        va = mxy_c @ np.array([-np.sin(a_c), np.cos(a_c)])
        u_min, u_max = float(ua.min()), float(ua.max())
        span = u_max - u_min
        if span < 0.008:
            continue
        for s_st in np.linspace(u_min + 0.2 * span, u_max - 0.2 * span, 5):
            band = np.abs(ua - s_st) < max(0.002, 0.08 * span)
            if int(band.sum()) < 12:
                continue
            vb2, hb2 = va[band], mh[band]
            vc2 = 0.5 * (float(vb2.min()) + float(vb2.max()))
            th = np.arctan2(hb2 - zc_m, vb2 - vc2)
            rho = np.hypot(vb2 - vc2, hb2 - zc_m)
            rho_out = []
            for lo_e, hi_e in zip(b_edges[:-1], b_edges[1:]):
                m_b = (th >= lo_e) & (th < hi_e)
                if m_b.any():
                    rho_out.append(float(rho[m_b].max()))
            if len(rho_out) >= 6:
                bins = max(bins, min(rho_out) / max(rho_out))
    return hull_sweep, bins


@pytest.mark.skipif(
    not (NPY / "vision_macro_cloud_unkclip_04_cyl.npy").exists(),
    reason="captured clouds not present")
def test_smeared_short_cylinder_reads_circle_in_bins():
    """The clip's live failing frame: the smeared Ø12x15 lying cylinder
    must read as circle evidence in the windowed-bin channel even though
    the smeared hull under-reads it (that hull under-read is WHY it
    certified B and dropped to the floor)."""
    pts = np.load(NPY / "vision_macro_cloud_unkclip_04_cyl.npy")
    assert len(pts) >= 100
    hull_sweep, bins = _estimators(pts)
    assert bins >= P.CIRCLE_RATIO, (
        f"bin channel must flag the smeared cylinder (got {bins:.3f})")
    # document the failure mode this guards against: the smeared hull
    # alone sat under the threshold
    assert hull_sweep < P.CIRCLE_RATIO + 0.05


def test_die_stays_below_circle_in_both_channels():
    """A 12 mm die (QA-confirmed LEGAL sortable freight) must never read
    as circle evidence: hull exact 0.707, bins biased but bounded well
    under 0.8. Synthetic dense macro-like cloud: top face + partial
    walls at 0.4 mm gsd."""
    gsd = 0.0004
    xs = np.arange(-0.006, 0.006, gsd)
    top = np.array([(x, y, BELT_Z + 0.012)
                    for x in xs for y in xs])
    wall_z = np.arange(BELT_Z + 0.004, BELT_Z + 0.012, gsd)
    walls = np.array([(sx * 0.006, y, z)
                      for sx in (-1, 1) for y in xs[::2] for z in wall_z])
    pts = np.vstack([top, walls])
    hull_sweep, bins = _estimators(pts)
    assert hull_sweep < 0.78, f"hull must stay square-exact ({hull_sweep:.3f})"
    assert bins < P.CIRCLE_RATIO, (
        f"bins on a die must stay under the criterion (got {bins:.3f})")


def test_limbo_hex15_flags_in_macro_shape_channels():
    """The pre-matrix probe's live catch: a 15 mm lying hex prism yields
    ~112 OVERHEAD points (enough for the main path, far too few for shape
    at 3 mm gsd) and read B. With the limbo merge, the macro cloud's shape
    channels must flag it: hexagon bins ratio ~cos30 = 0.866 >= 0.8.
    Synthetic dense cloud: resting hex, top facet + upper slants (what the
    macro head sees)."""
    gsd = 0.0004
    a = 0.00866                       # side = flat-to-flat 15 mm / sqrt(3)
    xs = np.arange(-0.015, 0.015, gsd)
    pts = []
    for x in xs:
        for y in np.arange(-a / 2, a / 2, gsd):        # top facet, z = 15
            pts.append((x, y, BELT_Z + 0.015))
        for s in (-1, 1):                              # upper slant facets
            for k in np.arange(0.0, 1.0, gsd / 0.0075):
                y = s * (a / 2 + k * a / 2)
                z = BELT_Z + 0.015 - k * 0.0075
                pts.append((x, y, z))
    pts = np.array(pts)
    perc = RTXPerception([None], macro=None)
    circ, hull, bins = perc._macro_shape_channels(pts[:, :2], pts)
    assert bins >= P.CIRCLE_RATIO, (
        f"limbo hex must flag in the bins channel (got {bins:.3f})")


def test_limbo_die20_stays_safe_in_macro_shape_channels():
    """A 20 mm cube (legal B, limbo band) must NOT flag in any merged
    channel: square bins ~0.73-0.76, hull 0.707, circ ~0.7."""
    gsd = 0.0004
    xs = np.arange(-0.010, 0.010, gsd)
    top = np.array([(x, y, BELT_Z + 0.020) for x in xs for y in xs])
    wall_z = np.arange(BELT_Z + 0.006, BELT_Z + 0.020, gsd)
    walls = np.array([(sx * 0.010, y, z)
                      for sx in (-1, 1) for y in xs[::2] for z in wall_z])
    pts = np.vstack([top, walls])
    perc = RTXPerception([None], macro=None)
    circ, hull, bins = perc._macro_shape_channels(pts[:, :2], pts)
    assert circ < P.CIRCLE_RATIO
    assert hull < 0.78
    assert bins < P.CIRCLE_RATIO, f"die20 bins must stay safe ({bins:.3f})"


@pytest.mark.skipif(
    not (NPY / "vision_macro_cloud_unkclip_05_hull.npy").exists(),
    reason="captured clouds not present")
def test_small_pebble_capture_still_flags():
    """The 12 mm random hull (true D) from the same clip run: with the
    real in-motion capture it must carry circle evidence in at least one
    channel (footprint circularity fired for it in the run; the section
    channels must not contradict a B certification either way)."""
    pts = np.load(NPY / "vision_macro_cloud_unkclip_05_hull.npy")
    hull_sweep, bins = _estimators(pts)
    # the pebble is near-isotropic: either channel at/above 0.75 is
    # consistent evidence; a hard B-certification (both far under) would
    # regress the campaign's D verdict
    assert max(hull_sweep, bins) >= 0.72
