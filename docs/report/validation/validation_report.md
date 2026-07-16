# Validation report

Generated: 20260715_194621 — `python -m cell.validate --matrix validation_matrix.yaml`

**22 runs, 235 items end to end; 0 failed.** Accuracy/containment columns show the scenario's WORST seed; unsafe/deadlock columns are totals.

| scenario | seeds | items | classification_accuracy | routing_accuracy | containment_rate | unsafe_errors | conservative_errors | cycle_p95_s | throughput_items_per_h | arm_interventions | recovery_success_rate | deadlocks | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| base (camera/sorter) | 6 | 66 | 1.0 | 0.9091 | 1.0 | 0 | 0 | 14.88 | 489.783 | 0 | — | 0 | PASS |
| base (oracle/sorter) | 2 | 22 | 1.0 | 1.0 | 1.0 | 0 | 0 | 8.18 | 495.7 | 0 | — | 0 | PASS |
| borderline (camera/sorter) | 3 | 33 | 0.8182 | 0.8182 | 1.0 | 0 | 6 | 8.91 | 512.033 | 0 | — | 0 | PASS |
| close_spacing (camera/sorter) | 3 | 18 | 0.6667 | 0.6667 | 1.0 | 0 | 2 | 7.66 | 1506.633 | 0 | — | 0 | PASS |
| low_confidence (camera/sorter) | 1 | 8 | 0.625 | 0.625 | 1.0 | 0 | 3 | 8.14 | 524.6 | 0 | — | 0 | PASS |
| fault_jam (camera/sorter) | 1 | 5 | 1.0 | 1.0 | 1.0 | 0 | 0 | 27.33 | 351.9 | 1 | 1.0 | 0 | PASS |
| failed_transfer (camera/sorter) | 1 | 3 | 1.0 | 0.6667 | 1.0 | 0 | 1 | 20.87 | 239.0 | 0 | — | 0 | PASS |
| stress_mix (camera/sorter) | 5 | 80 | 0.875 | 0.8125 | 1.0 | 0 | 10 | 23.02 | 622.38 | 1 | 1.0 | 0 | PASS |

Per-run artifacts (events.csv, summary.json, events_raw.jsonl): subfolders of `validation_20260715_194621/`.
