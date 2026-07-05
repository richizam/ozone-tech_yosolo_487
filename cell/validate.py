# -*- coding: utf-8 -*-
"""Batch validation runner — one command builds the full evidence base.

    python -m cell.validate --matrix configs/validation_matrix.yaml
    python -m cell.validate --quick          (first seed of every entry)

Runs every (scenario x seed) of the matrix through cell.run_sim in-process,
collects each run's summary.json, checks it against the entry's expectations
(plain key = exact, min_/max_ prefixes = bounds; every run must also exit 0),
and writes into runs/validation_<stamp>/:

    validation_matrix.csv    one row per run: key metrics + PASS/FAIL + reasons
    validation_report.md     scenario-level table + verdict (the review doc)
    <run dirs>/              full events.csv + summary.json per run

Exits non-zero if ANY run fails — wire it straight into CI.
"""
import argparse
import csv
import datetime
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cell import run_sim  # noqa: E402

MATRIX_COLS = [
    "scenario", "seed", "perception", "executive", "exit_code", "items",
    "classification_accuracy", "routing_accuracy", "containment_rate",
    "unsafe_errors", "conservative_errors", "cycle_p95_s",
    "throughput_items_per_h", "perception_latency_ms_mean",
    "command_margin_s_min", "arm_interventions", "recovery_success",
    "operator_callouts", "deadlocks", "low_confidence_fallback_count",
    "spacing_gate_activations", "verdict", "failed_checks",
]

REPORT_COLS = [
    "scenario", "seeds", "items", "classification_accuracy",
    "routing_accuracy", "containment_rate", "unsafe_errors",
    "conservative_errors", "cycle_p95_s", "throughput_items_per_h",
    "arm_interventions", "recovery_success_rate", "deadlocks", "verdict",
]


def check_expectations(expect, summary, exit_code):
    """-> list of failed-check descriptions (empty = pass)."""
    fails = []
    if exit_code != 0:
        fails.append(f"run_exit={exit_code}")
    for key, want in (expect or {}).items():
        if key.startswith("min_"):
            metric, op = key[4:], ">="
        elif key.startswith("max_"):
            metric, op = key[4:], "<="
        else:
            metric, op = key, "=="
        got = summary.get(metric)
        if got is None:
            fails.append(f"{metric}=None (want {op} {want})")
            continue
        ok = (got >= want if op == ">=" else
              got <= want if op == "<=" else got == want)
        if not ok:
            fails.append(f"{metric}={got} (want {op} {want})")
    return fails


def aggregate(scenario, rows):
    """Scenario-level line for the report table."""
    def vals(key):
        return [r[key] for r in rows if r.get(key) is not None]

    def fmin(key):
        v = vals(key)
        return round(min(v), 4) if v else None

    def fmean(key):
        v = vals(key)
        return round(sum(v) / len(v), 3) if v else None

    interventions = sum(vals("arm_interventions"))
    rec = vals("recovery_success")
    return {
        "scenario": scenario,
        "seeds": len(rows),
        "items": sum(vals("items")),
        "classification_accuracy": fmin("classification_accuracy"),
        "routing_accuracy": fmin("routing_accuracy"),
        "containment_rate": fmin("containment_rate"),
        "unsafe_errors": sum(vals("unsafe_errors")),
        "conservative_errors": sum(vals("conservative_errors")),
        "cycle_p95_s": max(vals("cycle_p95_s") or [None]),
        "throughput_items_per_h": fmean("throughput_items_per_h"),
        "arm_interventions": interventions,
        "recovery_success_rate": (round(sum(rec) / len(rec), 3) if rec else None),
        "deadlocks": sum(vals("deadlocks")),
        "verdict": "PASS" if all(r["verdict"] == "PASS" for r in rows) else "FAIL",
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--matrix", default=str(ROOT / "configs" / "validation_matrix.yaml"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--quick", action="store_true",
                    help="first seed of every matrix entry only")
    args = ap.parse_args(argv)

    matrix = yaml.safe_load(Path(args.matrix).read_text(encoding="utf-8"))
    defaults = matrix.get("defaults") or {}
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_root = Path(args.out) if args.out else ROOT / "runs" / f"validation_{stamp}"
    out_root.mkdir(parents=True, exist_ok=True)

    matrix_rows, report_rows, n_fail = [], [], 0
    for entry in matrix["runs"]:
        scenario = entry["scenario"]
        stem = Path(scenario).stem
        perception = entry.get("perception", "camera")
        executive = entry.get("executive", "table")
        expect = {**defaults, **(entry.get("expect") or {})}
        seeds = entry["seeds"][:1] if args.quick else entry["seeds"]
        scenario_rows = []
        for seed in seeds:
            run_dir = out_root / f"{stem}_{perception}_{executive}_s{seed}"
            label = f"{stem} seed={seed} {perception}/{executive}"
            print(f"\n=== validate: {label} ===")
            try:
                rc = run_sim.main(["--scenario", str(ROOT / scenario),
                                   "--seed", str(seed),
                                   "--perception", perception,
                                   "--executive", executive,
                                   "--out", str(run_dir)])
            except Exception as exc:            # a crash is a failed run, not a
                print(f"CRASH: {exc}")          # failed validation harness
                rc = 99
            summary = {}
            sfile = run_dir / "summary.json"
            if sfile.exists():
                summary = json.loads(sfile.read_text(encoding="utf-8"))
            fails = check_expectations(expect, summary, rc)
            row = {c: summary.get(c) for c in MATRIX_COLS
                   if c not in ("scenario", "seed", "perception", "executive",
                                "exit_code", "verdict", "failed_checks")}
            row.update(scenario=stem, seed=seed, perception=perception,
                       executive=executive, exit_code=rc,
                       verdict="PASS" if not fails else "FAIL",
                       failed_checks="; ".join(fails))
            matrix_rows.append(row)
            scenario_rows.append(row)
            n_fail += bool(fails)
            print(f"--- {label}: {'PASS' if not fails else 'FAIL: ' + '; '.join(fails)}")
        report_rows.append(aggregate(f"{stem} ({perception}/{executive})",
                                     scenario_rows))

    with open(out_root / "validation_matrix.csv", "w", newline="",
              encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=MATRIX_COLS, extrasaction="ignore")
        w.writeheader()
        w.writerows(matrix_rows)

    lines = [
        "# Validation report",
        "",
        f"Generated: {stamp} — `python -m cell.validate --matrix {Path(args.matrix).name}`"
        + (" `--quick`" if args.quick else ""),
        "",
        f"**{len(matrix_rows)} runs, {sum(r['items'] or 0 for r in matrix_rows)} items "
        f"end to end; {n_fail} failed.** Accuracy/containment columns show the "
        "scenario's WORST seed; unsafe/deadlock columns are totals.",
        "",
        "| " + " | ".join(REPORT_COLS) + " |",
        "|" + "|".join("---" for _ in REPORT_COLS) + "|",
    ]
    for r in report_rows:
        lines.append("| " + " | ".join(
            "—" if r.get(c) is None else str(r[c]) for c in REPORT_COLS) + " |")
    failed = [r for r in matrix_rows if r["verdict"] == "FAIL"]
    if failed:
        lines += ["", "## Failed runs", ""]
        lines += [f"- **{r['scenario']} seed {r['seed']}**: {r['failed_checks']}"
                  for r in failed]
    lines += ["", f"Per-run artifacts (events.csv, summary.json, events_raw.jsonl): "
                  f"subfolders of `{out_root.name}/`.", ""]
    (out_root / "validation_report.md").write_text("\n".join(lines), encoding="utf-8")

    print(f"\n{'=' * 60}")
    print(f"validation: {len(matrix_rows)} runs, {n_fail} failed -> {out_root}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
