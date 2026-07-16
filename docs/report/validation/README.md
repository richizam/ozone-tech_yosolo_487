# Validation evidence — MuJoCo twin (committed snapshot)

**Build:** tilt-tray sorter (`executive = sorter`) — the mechanism described
in [the report](../final_report_ru.md). Matrix generated **2026-07-15
19:46:21** on the final geometry (spill pans, arm safe terminals,
`small_tilt_frac` 0.85), the same build as the Isaac evidence under
[`isaac_evidence/`](../isaac_evidence/).

- [validation_report.md](validation_report.md) — scenario-level table
  (worst-seed accuracy/containment, unsafe/deadlock totals, cycle p95,
  throughput, interventions, recovery) with per-run verdicts.
- [validation_matrix.csv](validation_matrix.csv) — one row per
  (scenario × seed) with the full metric set and any failed checks.
- [`runs/`](runs/) — per-run `summary.json` + `events.csv` for all 22 runs,
  so every number above can be recomputed without running anything.

**Headline: 22 runs, 235 items, 0 failed, 0 unsafe errors, 0 deadlocks,
containment 1.0 in every run.**

Read the accuracy columns as designed: they show the scenario's **worst
seed**, and the stress/borderline scenarios deliberately allow conservative
outcomes (`allow_conservative: true` in
[`configs/validation_matrix.yaml`](../../../configs/validation_matrix.yaml)).
A "conservative error" is freight sent to C/D/REVIEW/operator instead of its
ideal zone — never the reverse. **The hard gates here are safety gates**
(unsafe routes, containment violations, deadlocks, negative command margin);
the 0.7–0.8 accuracy floors apply to attack scenarios only, by design.

Nominal detail (base, camera perception, 6 seeds): classification 1.0 on
every seed; routing 1.0 on five seeds and 0.9091 on seed 99, whose single
non-ideal item is a **safe operator call-out** (`operator_callouts: 1`,
`unsafe_errors: 0`) — a designed terminal, not a mis-sorted parcel.

Regenerate from scratch (full run folders land in `runs/validation_<stamp>/`,
intentionally not committed — this folder is the snapshot):

```bash
# any machine, CPU only — exits non-zero if any gate fails
python -m cell.validate --matrix configs/validation_matrix.yaml
# or, in the jury container:
docker compose up
```
