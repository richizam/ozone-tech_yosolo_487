# Simulation evidence — SortMaster sort cell

Everything here was produced by the **tilt-tray build at HEAD** — the
mechanism described in [the report](../final_report_ru.md) and
[the architecture note](../architecture.md). Two engines, one cell, one
parameter source ([`cell/params.py`](../../../cell/params.py)):

| Folder | Engine | What it is |
|---|---|---|
| [`xbelt/`](xbelt/) | **NVIDIA Isaac Sim 6.0.1** (PhysX 5 + RTX) | primary digital twin: classification from **rendered depth cameras**, freight moved by **contact physics only** |
| `twin_videos/` (in cloud storage) | **MuJoCo 3** | validation-twin recordings (CPU, no GPU needed) |
| [`../validation/`](../validation/) | MuJoCo 3 | the validation twin's committed matrix (22 runs, 235 items) |

> Earlier build generations (arm/table executive, ARB actuator deck, v23/v25
> tilt-tray revisions) have been removed from the working tree to keep the
> evidence unambiguous. They remain in git history; nothing in the current
> report cites them.

## Headline results — Isaac matrix (14 runs, GATES: PASS)

Consolidated by [`tools/consolidate_isaac_matrix.py`](../../../tools/consolidate_isaac_matrix.py),
which exits non-zero if any gate fails. Raw per-run artifacts for all 14 runs
are committed under [`xbelt/matrix/`](xbelt/matrix/) — every number below can
be recomputed without a GPU.

| Gate | Target | Actual |
|---|---|---|
| Classification, nominal (6 seeds × 11 items) | ≥ 0.98 | **1.0** (66/66) |
| Physical routing, nominal | ≥ 0.97 | **1.0** (66/66) |
| Unsafe errors, all 14 runs | 0 | **0** |
| Floor drops, all 14 runs | 0 | **0** |
| Containment in receptacles | 1.0 | **1.0** |
| Direct velocity writes on freight (nominal) | 0 | **0** |
| Min command margin | > 0 | **1.05 s** |
| 11 mm cube | delivered to B | **B** (binding min dim measured 11.0 mm, floor 10.8 mm) |

**The sweep runs degrade — honestly and safely.** The accuracy gates above
are nominal-only by design; stress and edge runs trade ideal-zone accuracy
for safety, and every deviation lands in a conservative terminal (C / D /
REVIEW / operator call-out) — never a permissive wrong-way route. Zero
unsafe errors and zero floor drops across all 14 runs. Per-run routing:

| Run | Routing | Note |
|---|---|---|
| `seed{42,1,2,3,7,99}_nominal` | 1.0 | 66/66 |
| `close_spacing`, `high_mass`, `fault_jam` | 1.0 | stress with no accuracy cost |
| `off_center` | 0.909 | 10/11, one conservative diversion |
| `fault_tray` | 0.818 | dead tilt actuator → end-line operator call-out (the designed reaction) |
| `edge_items_all` | 0.70 | 14/20 borderline-attack set |
| `edge_small` | 0.667 | sub-floor freight → C by the certification rule |
| `low_friction` (μ ×0.7) | 0.636 | 7/11 — worst case; still 0 unsafe, 0 floor |

## Contents of `xbelt/`

- [`matrix/`](xbelt/matrix/) — the 14-run matrix: per-run `summary.json`,
  `events.csv`, `actuator_log.csv`, `reads_log.json`, plus the consolidated
  [`matrix_summary.json`](xbelt/matrix/matrix_summary.json).
- [`audits/pans/`](xbelt/audits/pans/) — machine audits at HEAD: visual swept
  volume (**0 intersections / 0 corridor hits**) and collider clearance
  (PASS), both against the final geometry.
- [`perception/`](xbelt/perception/) — what the sensors actually saw: RGB +
  depth `.npy` + the perception pipeline's **own exported masks and point
  clouds**, the fused `reads_log.json` behind every verdict, and the jury
  panels ([`panels/`](xbelt/perception/panels/),
  [`panels_edge/`](xbelt/perception/panels_edge/),
  `perception_demo.mp4` in cloud storage at `xbelt/perception/perception_demo.mp4`).
- [`videos_final/`](xbelt/videos_final/) — RTX recordings of real runs (see
  the vintage note below) and [`endcard.png`](xbelt/videos_final/endcard.png),
  generated from `matrix_summary.json`.
- [`proof_pack/`](xbelt/proof_pack/) — before/after mechanical evidence from
  the carrier rebuild (64 visual intersections → 0).

## Video vintage — read this before comparing frames with the report

RTX clip rendering is expensive (~10–40 min per clip), so the showcase set
was **not** re-rendered after every geometry change. Three vintages exist:

- most of `videos_final/` — rendered before the discharge-flank spill pans;
- `edge_small_items.mp4` and `twin_videos/` — rendered on the spill-pans
  build (pans visible);
- the final build additionally carries the swept-axis section classifier
  (no visual change at all) and two containment guards found by the last
  matrix pass: a 6 mm fin inside the 30 mm REVIEW/B-connector seam and a
  stepped south apron under the deck. Both are internal containment
  furniture, invisible or near-invisible at every showcase camera angle;
  `unknown_shapes_demo.mp4`, when present, is rendered on this final build.

All guards are additive statics: they touch neither the sorter, the trays,
the chutes, the perception station nor any physics parameter, so every clip
remains a faithful recording of the mechanism it shows. Every physics claim
in the table above comes from the matrix at HEAD, not from the clips.

## Reproduce

```bash
# Isaac (GPU, official container) — 14 runs + hard gates
bash isaac/run_matrix.sh /out/m && python3 tools/consolidate_isaac_matrix.py /out/m
# MuJoCo twin (any machine, CPU) — 22 runs + hard gates
docker compose up
```

Exact commands, mounts and VRAM guidance: [`isaac/README.md`](../../../isaac/README.md).
