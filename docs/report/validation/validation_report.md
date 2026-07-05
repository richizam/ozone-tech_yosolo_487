# Validation report

Generated: 20260706_022513 — `python -m cell.validate --matrix validation_matrix.yaml`

**22 runs, 235 items end to end; 0 failed.** Accuracy/containment columns show the scenario's WORST seed; unsafe/deadlock columns are totals.

| scenario | seeds | items | classification_accuracy | routing_accuracy | containment_rate | unsafe_errors | conservative_errors | cycle_p95_s | throughput_items_per_h | arm_interventions | recovery_success_rate | deadlocks | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| base (camera/table) | 6 | 66 | 0.9091 | 0.8182 | 1.0 | 0 | 2 | 43.69 | 470.217 | 1 | 1.0 | 0 | PASS |
| base (oracle/table) | 2 | 22 | 1.0 | 1.0 | 1.0 | 0 | 0 | 6.29 | 507.55 | 0 | — | 0 | PASS |
| borderline (camera/table) | 3 | 33 | 0.6364 | 0.5455 | 1.0 | 0 | 9 | 68.88 | 317.833 | 4 | 0.0 | 0 | PASS |
| close_spacing (camera/table) | 3 | 18 | 0.8333 | 0.8333 | 1.0 | 0 | 1 | 8.21 | 1115.433 | 0 | — | 0 | PASS |
| low_confidence (camera/table) | 1 | 8 | 0.625 | 0.625 | 1.0 | 0 | 3 | 5.08 | 510.1 | 0 | — | 0 | PASS |
| fault_jam (camera/table) | 1 | 5 | 1.0 | 1.0 | 1.0 | 0 | 0 | 29.15 | 397.8 | 1 | 1.0 | 0 | PASS |
| failed_transfer (camera/table) | 1 | 3 | 1.0 | 0.6667 | 1.0 | 0 | 1 | 22.76 | 214.5 | 1 | 1.0 | 0 | PASS |
| stress_mix (camera/table) | 5 | 80 | 0.8125 | 0.625 | 1.0 | 0 | 12 | 73.6 | 356.98 | 11 | 0.067 | 0 | PASS |

Per-run artifacts (events.csv, summary.json, events_raw.jsonl): subfolders of `validation_20260706_022513/`.
