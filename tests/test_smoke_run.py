# -*- coding: utf-8 -*-
"""End-to-end smoke checks (brief §18): one small camera/table run and one
fault drill prove the critical engineering claims without opening the viewer.

Run time budget: two short sims (~30 s wall total). The full evidence base is
`python -m cell.validate` (multi-scenario, multi-seed).
"""
import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cell import run_sim  # noqa: E402


@pytest.fixture(scope="module")
def smoke_run(tmp_path_factory):
    """One three-item camera/table run: B, C and D routes all exercised."""
    out = tmp_path_factory.mktemp("smoke_run")
    scenario = {
        "seed": 42,
        "perception": "camera",
        "executive": "table",
        "items": {"box_s": 1, "box_l": 1, "bottle": 1},   # B, C, D
        "spawn_gap_s": [6.0, 8.0],
        "max_sim_s": 180,
    }
    sc_file = out / "smoke.yaml"
    sc_file.write_text(yaml.safe_dump(scenario), encoding="utf-8")
    rc = run_sim.main(["--scenario", str(sc_file), "--out", str(out / "run")])
    summary = json.loads((out / "run" / "summary.json").read_text(encoding="utf-8"))
    return rc, summary, out / "run"


def test_base_run_completes_and_routes(smoke_run):
    rc, s, _ = smoke_run
    assert rc == 0
    assert s["items"] == 3
    assert s["routing_accuracy"] == 1.0
    assert s["unsafe_errors"] == 0
    assert s["deadlocks"] == 0


def test_cycle_metrics_not_null(smoke_run):
    _, s, _ = smoke_run
    for key in ("cycle_mean_s", "cycle_p95_s", "cycle_max_s", "cycle_min_s"):
        assert s[key] is not None, f"{key} must not be null"
        assert s[key] > 0


def test_containment_reported_and_clean(smoke_run):
    _, s, _ = smoke_run
    assert s["containment_rate"] == 1.0
    assert s["containment_violations"] == 0
    assert s["cage_entry_speed_max_mps"] is not None


def test_modes_and_sensor_are_logged(smoke_run):
    _, s, _ = smoke_run
    assert s["perception_mode"] == "camera"
    assert s["executive_mode"] == "sorter"
    assert s["sensor_model"] != "oracle_ground_truth"
    assert s["oracle_used_for_classification"] is False


def test_timing_instrumentation(smoke_run):
    _, s, _ = smoke_run
    assert s["perception_latency_ms_mean"] is not None
    assert s["command_margin_s_min"] is not None
    assert s["command_margin_s_min"] > 0, "route command must precede table entry"
    assert s["items_with_negative_command_margin"] == 0
    assert s["conveyor_a_stop_count"] == 0


def test_events_csv_schema(smoke_run):
    """The event log carries the full executive story (same vocabulary as
    the Isaac build): release -> landing -> route -> tilt -> discharge ->
    delivery, so a reviewer can replay any item's life."""
    import csv
    _, _, run_dir = smoke_run
    with open(run_dir / "events.csv", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert rows and set(rows[0].keys()) >= {"t", "event", "slug"}
    events = {r["event"] for r in rows}
    for needed in ("item_classified", "escapement_release",
                   "induction_landed", "routing_cmd", "tilt_cmd",
                   "discharge_confirmed", "item_delivered"):
        assert needed in events, needed


def test_fault_jam_produces_intervention(tmp_path):
    """Injected snag -> watchdog -> arm recovery -> correct cage, no deadlock."""
    scenario = {
        "seed": 42,
        "perception": "camera",
        "executive": "sorter",
        "items": {"helmet": 1, "box_s": 1},
        "spawn_gap_s": [6.0, 8.0],
        "inject_jam": {"slug": "helmet", "at_y": 2.6},
        "max_sim_s": 200,
    }
    sc_file = tmp_path / "jam.yaml"
    sc_file.write_text(yaml.safe_dump(scenario), encoding="utf-8")
    rc = run_sim.main(["--scenario", str(sc_file), "--out", str(tmp_path / "run")])
    s = json.loads((tmp_path / "run" / "summary.json").read_text(encoding="utf-8"))
    assert rc == 0
    assert s["arm_interventions"] >= 1
    assert s["recovery_success"] == 1.0
    assert s["deadlocks"] == 0
    assert s["routing_accuracy"] == 1.0          # recovered to its TRUE zone


def test_camera_mode_uses_no_ground_truth_labels():
    """Static guard: the perception pipeline must not import or read the
    manifest ground truth (zone/category labels)."""
    src = (ROOT / "perception" / "pipeline.py").read_text(encoding="utf-8")
    for token in ("manifest", '"zone"]', "'zone']", "ground_truth",
                  "item_ground_truth"):
        assert token not in src, f"pipeline.py must not touch {token!r}"
