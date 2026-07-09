# -*- coding: utf-8 -*-
"""Config locks for the ARB actuator deck and the item material table
(SUPER_REALISTIC plan P1/P3). Pure params/manifest checks — no engine."""
import json
from pathlib import Path

import pytest

from cell import params as P

REPO = Path(__file__).resolve().parent.parent


def manifest_slugs():
    m = json.loads((REPO / "cell" / "assets" / "manifest.json")
                   .read_text(encoding="utf-8"))
    return [e["slug"] for e in m]


def test_arb_grid_tiles_the_routing_zone_exactly():
    pts = P.arb_patches()
    assert len(pts) == P.ARB_DECK["nx"] * P.ARB_DECK["ny"]
    tb = P.TABLE
    x_lo = min(p["cx"] - p["hx"] for p in pts)
    x_hi = max(p["cx"] + p["hx"] for p in pts)
    y_lo = min(p["cy"] - p["hy"] for p in pts)
    y_hi = max(p["cy"] + p["hy"] for p in pts)
    assert abs(x_lo - tb["route_x"]) < 1e-9 and abs(x_hi - tb["x1"]) < 1e-9
    assert abs(y_lo - (tb["y"] - tb["width"] / 2)) < 1e-9
    assert abs(y_hi - (tb["y"] + tb["width"] / 2)) < 1e-9
    # no overlaps / no gaps: total patch area == zone area
    area = sum(4 * p["hx"] * p["hy"] for p in pts)
    assert abs(area - (tb["x1"] - tb["route_x"]) * tb["width"]) < 1e-9


def test_arb_patch_size_is_industrial():
    p = P.arb_patches()[0]
    for d in (2 * p["hx"], 2 * p["hy"]):
        assert 0.10 <= d <= 0.20, "ARB cells are 100-200 mm class modules"


def test_arb_actuation_dynamics_are_realistic():
    d = P.ARB_DECK
    assert 0.01 <= d["latency_s"] <= 0.20        # drives/valves, not magic
    assert 1.0 <= d["ramp_mps2"] <= 20.0         # no step velocity changes
    assert d["v_max"] >= P.TABLE["speed"]        # saturation above setpoint
    assert 0.0 < d["noise_frac"] <= 0.05
    assert d["activation_pad_m"] > d["latency_s"] * P.TABLE["speed"], \
        "the pre-spin halo must give a patch more warning than its latency"


def test_every_manifest_item_has_an_explicit_material():
    missing = [s for s in manifest_slugs() if s not in P.MATERIALS]
    assert not missing, f"add MATERIALS entries for {missing}"


@pytest.mark.parametrize("slug", manifest_slugs())
def test_material_values_sane(slug):
    sf, df, rest, ld, ad = P.MATERIALS[slug]
    assert 0.05 <= df <= sf <= 1.2               # dynamic <= static
    assert 0.0 <= rest <= 0.5
    assert 0.0 <= ld <= 1.0 and 0.0 <= ad <= 1.0
