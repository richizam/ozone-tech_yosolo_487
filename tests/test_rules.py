# -*- coding: utf-8 -*-
"""Unit tests for the rule engine on synthetic primitives with known answers.

These encode the official rules' edge cases:
  - square section  cos(45deg)=0.707 < 0.8  -> not circular
  - pentagon        cos(36deg)=0.809 >= 0.8 -> circular (barely!)
  - hexagon         cos(30deg)=0.866 >= 0.8 -> circular (the «Цилиндр» trap)
  - priority: dimension violations always beat shape (oversized cylinder -> C)
"""
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.classify_mesh import classify_mesh  # noqa: E402


def prism(circumradius, height, sections):
    """Regular prism, axis along Z, dimensions in mm."""
    return trimesh.creation.cylinder(radius=circumradius, height=height, sections=sections)


def box(a, b, c):
    return trimesh.creation.box(extents=[a, b, c])


def test_plain_box_is_sortable():
    assert classify_mesh(box(200, 150, 100))["zone"] == "B"


def test_max_size_box_just_inside_limits():
    assert classify_mesh(box(449, 319, 319))["zone"] == "B"


def test_oversize_box():
    r = classify_mesh(box(460, 300, 300))
    assert r["zone"] == "C" and r["oversize"]


def test_second_dimension_over_320_is_oversize():
    # 400x400x300 fits 450 in length but 400 > 320 in width -> C (the Box L case)
    r = classify_mesh(box(400, 400, 300))
    assert r["zone"] == "C" and r["oversize"]


def test_thin_sheet_is_undersize():
    r = classify_mesh(box(200, 150, 9))
    assert r["zone"] == "C" and r["undersize"]


def test_cylinder_is_circular():
    r = classify_mesh(prism(50, 200, sections=64))
    assert r["zone"] == "D"
    assert r["max_ratio"] > 0.95


def test_sphere_is_circular():
    r = classify_mesh(trimesh.creation.icosphere(subdivisions=3, radius=100))
    assert r["zone"] == "D"


def test_square_prism_is_not_circular():
    r = classify_mesh(prism(100, 300, sections=4))
    assert r["zone"] == "B"
    assert r["max_ratio"] < 0.75  # cos(45deg) ~ 0.707


def test_pentagon_prism_is_circular_barely():
    r = classify_mesh(prism(100, 300, sections=5))
    assert r["zone"] == "D"
    assert 0.8 <= r["max_ratio"] < 0.83  # cos(36deg) ~ 0.809


def test_hexagon_prism_is_circular_the_cylinder_trap():
    r = classify_mesh(prism(25, 435, sections=6))
    assert r["zone"] == "D"
    assert 0.85 <= r["max_ratio"] < 0.89  # cos(30deg) ~ 0.866


def test_priority_oversized_cylinder_goes_to_C_not_D():
    # circle in section AND too long: dimension rule wins per the task statement
    r = classify_mesh(prism(50, 500, sections=64))
    assert r["zone"] == "C" and r["oversize"]


def test_priority_undersize_thin_rod_goes_to_C_not_D():
    # 9 mm diameter rod: circular but undersize (the «Ручка» case)
    r = classify_mesh(prism(4.5, 148, sections=64))
    assert r["zone"] == "C" and r["undersize"]


def test_official_ground_truth_is_reproduced():
    """The committed ground truth for the 11 official items must always hold."""
    import json
    gt_path = Path(__file__).resolve().parents[1] / "docs" / "ground_truth" / "item_ground_truth.json"
    stl_dir = Path(__file__).resolve().parents[1] / "extracted" / "doc-1782987733" / "Stl"
    if not stl_dir.exists():
        import pytest
        pytest.skip("official STL set not present")
    expected = {e["file"]: e["zone"] for e in json.loads(gt_path.read_text(encoding="utf-8"))}
    from tools.classify_mesh import classify
    for fname, zone in expected.items():
        got = classify(stl_dir / fname)
        assert got["zone"] == zone, f"{fname}: expected {zone}, got {got['zone']}"
