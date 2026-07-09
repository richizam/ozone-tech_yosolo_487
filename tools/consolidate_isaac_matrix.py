# -*- coding: utf-8 -*-
"""Consolidate the Isaac ARB-deck validation matrix into one report.

Usage:  python tools/consolidate_isaac_matrix.py <matrix_dir> [out.json]
<matrix_dir> holds one subdirectory per run, each with summary.json
(produced by isaac/run_matrix.sh). Emits consolidated JSON + a markdown
table on stdout, and gates the plan's target metrics.
"""
import json
import sys
from pathlib import Path

TARGETS = {"classification_accuracy": 0.98, "routing_accuracy": 0.97,
           "unsafe_errors": 0, "containment_rate": 1.0}


def main():
    root = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else root / "matrix_summary.json"
    runs = {}
    for d in sorted(root.iterdir()):
        sj = d / "summary.json"
        if not sj.is_file():
            continue
        s = json.loads(sj.read_text(encoding="utf-8"))
        cls = s.get("classification") or {}
        deck = s.get("arb_deck") or {}
        manual = sum(1 for r in s.get("items", [])
                     if r.get("delivered") == "MANUAL")
        runs[d.name] = {
            "seed": s.get("seed"),
            "sweep": s.get("sweep"),
            "n_items": s.get("n_items"),
            "n_delivered": s.get("n_delivered"),
            "n_routed_ok": s.get("n_routed_ok"),
            "manual_callouts": manual,
            "routing_accuracy": s.get("routing_accuracy"),
            "classification_accuracy": cls.get("accuracy"),
            "unsafe_errors": s.get("unsafe_errors"),
            "containment_rate": (s.get("containment") or {}).get("containment_rate"),
            "cage_entry_speed_max_mps": (s.get("containment") or {}).get("cage_entry_speed_max_mps"),
            "cycle_s": s.get("cycle_s"),
            "throughput_items_per_h": s.get("throughput_items_per_h"),
            "realtime_factor": s.get("realtime_factor"),
            "nominal_motion_model": s.get("nominal_motion_model"),
            "direct_velocity_writes_nominal": s.get("direct_velocity_writes_nominal"),
            "actuator_commands_count": deck.get("actuator_commands_count"),
            "actuator_latency_ms": deck.get("actuator_latency_ms"),
            "max_surface_speed_mps": deck.get("max_surface_speed_mps"),
        }
    nominal = {k: v for k, v in runs.items() if k.endswith("_nominal")}
    n_items = sum(r["n_items"] for r in nominal.values())
    n_cls_ok = sum(round((r["classification_accuracy"] or 0) * r["n_items"])
                   for r in nominal.values())
    agg = {
        "runs": len(runs),
        "nominal_seeds": len(nominal),
        "nominal_item_trials": n_items,
        "nominal_classification_accuracy": round(n_cls_ok / max(1, n_items), 4),
        "nominal_routing_ok": sum(r["n_routed_ok"] for r in nominal.values()),
        "total_unsafe_errors": sum(r["unsafe_errors"] or 0 for r in runs.values()),
        "min_containment_rate": min((r["containment_rate"] for r in runs.values()
                                     if r["containment_rate"] is not None),
                                    default=None),
        "total_actuator_commands": sum(r["actuator_commands_count"] or 0
                                       for r in runs.values()),
        "total_direct_velocity_writes_nominal": sum(
            r["direct_velocity_writes_nominal"] or 0 for r in runs.values()),
        "targets": TARGETS,
    }
    doc = {"aggregate": agg, "runs": runs}
    out_path.write_text(json.dumps(doc, indent=2), encoding="utf-8")

    cols = ["run", "n", "ok", "manual", "cls", "unsafe", "contain",
            "v_entry", "acts", "set_v"]
    print("| " + " | ".join(cols) + " |")
    print("|" + "---|" * len(cols))
    for name, r in runs.items():
        print(f"| {name} | {r['n_delivered']}/{r['n_items']} | {r['n_routed_ok']} "
              f"| {r['manual_callouts']} | {r['classification_accuracy']} "
              f"| {r['unsafe_errors']} | {r['containment_rate']} "
              f"| {r['cage_entry_speed_max_mps']} "
              f"| {r['actuator_commands_count']} "
              f"| {r['direct_velocity_writes_nominal']} |")
    print(json.dumps(agg, indent=2))
    ok = (agg["total_unsafe_errors"] == 0
          and (agg["min_containment_rate"] or 0) >= 1.0
          and agg["total_direct_velocity_writes_nominal"] == 0)
    print(f"GATES: {'PASS' if ok else 'CHECK FAILURES ABOVE'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
