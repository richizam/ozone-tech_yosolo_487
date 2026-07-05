# Validation evidence (committed snapshot)

This folder holds the committed snapshot of the batch-validation evidence:

- [validation_report.md](validation_report.md) — scenario-level table (worst-seed
  accuracy/containment, unsafe/deadlock totals, cycle p95, throughput,
  interventions, recovery) with per-run verdicts.
- [validation_matrix.csv](validation_matrix.csv) — one row per (scenario × seed)
  run with the full metric set and any failed checks.

Regenerate from scratch (the run folders with full per-item `events.csv`,
`summary.json` and `events_raw.jsonl` land in `runs/validation_<stamp>/`,
which is intentionally not committed):

```powershell
.\.venv\Scripts\python.exe -m cell.validate --matrix configs\validation_matrix.yaml
```

The command exits non-zero if any run violates its expectations
(`configs/validation_matrix.yaml`): unsafe routes, containment violations,
deadlocks and negative command margins are hard gates for every scenario.
