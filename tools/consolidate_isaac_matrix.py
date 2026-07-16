# -*- coding: utf-8 -*-
"""Consolidate the Isaac tilt-tray validation matrix into one report.

Usage:  python tools/consolidate_isaac_matrix.py <matrix_dir> [out.json]
<matrix_dir> holds one subdirectory per run, each with summary.json
(produced by isaac/run_matrix.sh). Emits consolidated JSON + a markdown
table on stdout, and gates the plan's target metrics (§8 of the rebuild
brief): classification >= 0.98, routing >= 0.97 (nominal), unsafe = 0,
containment = 1.0, direct velocity writes = 0, command margin > 0, and the
11 mm cube delivered to B in the edge run.
"""
import json
import sys
from pathlib import Path

TARGETS = {"classification_accuracy": 0.98, "routing_accuracy": 0.97,
           "unsafe_errors": 0, "containment_rate": 1.0,
           "direct_velocity_writes_nominal": 0, "command_margin_s_min": 0.0}


def main():
    root = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else root / "matrix_summary.json"
    runs = {}
    cube11 = None
    for d in sorted(root.iterdir()):
        sj = d / "summary.json"
        if not sj.is_file():
            continue
        s = json.loads(sj.read_text(encoding="utf-8"))
        cls = s.get("classification") or {}
        srt = s.get("sorter") or {}
        margin = s.get("command_margin_s") or {}
        items = s.get("items", [])
        manual = sum(1 for r in items if r.get("delivered") == "MANUAL")
        review = sum(1 for r in items if r.get("delivered") == "REVIEW")
        for r in items:
            if r.get("slug") == "edge_cube11":
                cube11 = {"run": d.name, "delivered": r.get("delivered"),
                          "ok": r.get("ok"), "dims_mm": r.get("dims_mm"),
                          "cls_reason": r.get("cls_reason")}
        runs[d.name] = {
            "seed": s.get("seed"),
            "sweep": s.get("sweep"),
            "n_items": s.get("n_items"),
            "n_delivered": s.get("n_delivered"),
            "n_routed_ok": s.get("n_routed_ok"),
            "manual_callouts": manual,
            "review_deliveries": review,
            "routing_accuracy": s.get("routing_accuracy"),
            "classification_accuracy": cls.get("accuracy"),
            "unsafe_errors": s.get("unsafe_errors"),
            "floor_drops": s.get("floor_drops"),
            "containment_rate": (s.get("containment") or {}).get("containment_rate"),
            "cage_entry_speed_max_mps": (s.get("containment") or {}).get("cage_entry_speed_max_mps"),
            "cycle_s": s.get("cycle_s"),
            "command_margin_s_min": margin.get("min"),
            "throughput_items_per_h": s.get("throughput_items_per_h"),
            "realtime_factor": s.get("realtime_factor"),
            "nominal_motion_model": s.get("nominal_motion_model"),
            "direct_velocity_writes_nominal": s.get("direct_velocity_writes_nominal"),
            "carrier_commands_count": srt.get("carrier_commands_count"),
            "tilt_time_ms": srt.get("tilt_time_ms"),
            "discharge_latency_ms": srt.get("discharge_latency_ms"),
            "landing_offset_max_mm": srt.get("landing_offset_max_mm"),
            "double_occupancy_events": srt.get("double_occupancy_events"),
            "discharge_misses_to_review": srt.get("discharge_misses_to_review"),
            "end_line_callouts": srt.get("end_line_callouts"),
            "recovery": s.get("recovery"),
        }
    nominal = {k: v for k, v in runs.items() if "nominal" in k}
    n_items = sum(r["n_items"] for r in nominal.values())
    n_cls_ok = sum(round((r["classification_accuracy"] or 0) * r["n_items"])
                   for r in nominal.values())
    margins = [r["command_margin_s_min"] for r in runs.values()
               if r["command_margin_s_min"] is not None]
    agg = {
        "executive": "tilt_tray_linear_sorter",
        "runs": len(runs),
        "nominal_seeds": len(nominal),
        "nominal_item_trials": n_items,
        "nominal_classification_accuracy": round(n_cls_ok / max(1, n_items), 4),
        "nominal_routing_ok": sum(r["n_routed_ok"] for r in nominal.values()),
        "nominal_routing_accuracy": round(
            sum(r["n_routed_ok"] for r in nominal.values()) / max(1, n_items), 4),
        "total_unsafe_errors": sum(r["unsafe_errors"] or 0 for r in runs.values()),
        "total_floor_drops": sum(r["floor_drops"] or 0 for r in runs.values()),
        "min_containment_rate": min((r["containment_rate"] for r in runs.values()
                                     if r["containment_rate"] is not None),
                                    default=None),
        "min_command_margin_s": min(margins, default=None),
        "total_carrier_commands": sum(r["carrier_commands_count"] or 0
                                      for r in runs.values()),
        "total_direct_velocity_writes_nominal": sum(
            r["direct_velocity_writes_nominal"] or 0 for r in runs.values()),
        "cube11_proof": cube11,
        "targets": TARGETS,
    }
    doc = {"aggregate": agg, "runs": runs}
    out_path.write_text(json.dumps(doc, indent=2), encoding="utf-8")

    cols = ["run", "n", "ok", "manual", "review", "cls", "unsafe", "contain",
            "margin_min", "cmds", "set_v"]
    print("| " + " | ".join(cols) + " |")
    print("|" + "---|" * len(cols))
    for name, r in runs.items():
        print(f"| {name} | {r['n_delivered']}/{r['n_items']} | {r['n_routed_ok']} "
              f"| {r['manual_callouts']} | {r['review_deliveries']} "
              f"| {r['classification_accuracy']} "
              f"| {r['unsafe_errors']} | {r['containment_rate']} "
              f"| {r['command_margin_s_min']} "
              f"| {r['carrier_commands_count']} "
              f"| {r['direct_velocity_writes_nominal']} |")
    print(json.dumps(agg, indent=2))
    # A missing measurement must FAIL the gate, never pass it silently: an
    # absent margin or an absent cube11 proof means the matrix did not run
    # what it claims to prove. (The cube11 proof lives in the edge runs, so
    # it is only required when the matrix includes them.)
    has_edge = any("edge" in name for name in runs)
    ok = (agg["total_unsafe_errors"] == 0
          and agg["total_floor_drops"] == 0
          and (agg["min_containment_rate"] or 0) >= 1.0
          and agg["total_direct_velocity_writes_nominal"] == 0
          and agg["nominal_classification_accuracy"] >= TARGETS["classification_accuracy"]
          and agg["nominal_routing_accuracy"] >= TARGETS["routing_accuracy"]
          and agg["min_command_margin_s"] is not None
          and agg["min_command_margin_s"] > 0
          and (not has_edge
               or (agg["cube11_proof"] is not None
                   and agg["cube11_proof"]["delivered"] == "B")))
    if agg["min_command_margin_s"] is None:
        print("GATE FAIL: no command margin logged in any run")
    if has_edge and agg["cube11_proof"] is None:
        print("GATE FAIL: edge runs present but no 11 mm cube proof found")
    print(f"GATES: {'PASS' if ok else 'CHECK FAILURES ABOVE'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
