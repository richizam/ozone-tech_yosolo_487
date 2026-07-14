# AUTONOMOUS RUN (2026-07-14) — RTX 4090 box, owner away, OK granted

**Server:** `ssh -p 35206 root@85.218.235.6` (RTX 4090, 24 GB). Docker +
nvidia runtime present, isaac-sim:6.0.1 pulled, v25 repo deployed, shader
cache warm. HTTP served on 8080; local tunnel open.

**VRAM FIX = LEAN MODE (validated):** the 24 GB card OOMs on the full-
dressing 20-item scene (fit the 5090's 32 GB). `SM_LEAN=1` skips the
`dress_scene()` cosmetic layer (all collide=False) for VALIDATION runs —
physics/perception/containment identical. PROVEN: lean seed42 == full
seed42 (11/11, unsafe 0, floor 0, cls 1.0, contain 1.0, setv 0), byte-for-
byte on the gate metrics. Showcase renders WITH dressing (short, few items,
fits). `SM_LEAN` threaded through run_matrix.sh / rerun_edges.sh /
master_pipeline.sh (exports it); scene_usd.py gates on it (needs `import os`).

**Owner authorization (this session):** run to completion autonomously.
Take stills, verify visuals myself vs the 5 defect screenshots, and if I
judge them good, launch the showcase, pull evidence, stamp numbers, COMMIT
LOCAL. Guardrails: **no git push**, no destructive ops, only my own
containers, stop-and-flag on any severe visual defect.

**Live state:** master_pipeline.sh running lean matrix → consolidate →
audit → (lean) stills → touch MATRIX_PHASE_DONE. Nominals 6/6 clean,
edge_items_all 20/20 floor=0 (box_l fixed by v25 mouth interlock + no OOM).
Next: on MATRIX_PHASE_DONE → run `isaac/verify_stills.sh` (FULL dressing,
curated 5-item set) → Read the PNGs → judge vs defects → if good launch
`isaac/showcase5.sh <out> 7` (NO SM_LEAN) → fetch_evidence → panels/endcard
→ stamp README/report_ru/deck/calculations → local commit.

Restart deploy set if session breaks: scp isaac/{scene_usd,run_matrix.sh,
rerun_edges.sh,master_pipeline.sh,verify_stills.sh,sorter.py,run_isaac.py,
audit_clearance.py} + cell/{params,sorter,run_sim}.py to /root/sortmaster/.

---

# PROJECT STATE — pause point (2026-07-11, server shut down by owner)

Single source for resuming. Everything below is committed; the original GPU
server (RTX 5090, `ssh -p 40576 root@90.224.159.6`) was stopped for cost.

**NEW SERVER (2026-07-11): RTX 5070 Ti, 16 GB VRAM, IP 120.238.149.205**
(SSH port/user TBD — confirm on resume). Repo mirror `/root/sortmaster`,
outputs `/root/sortmaster_out`, same layout. Two changes vs the 5090 box:
- **16 GB VRAM (was 32)**: a single headless run at 720p (~8–12 GB) fits,
  but there is NO 16 GB of headroom for anything resident. Keep ONE
  container at a time (we always did). If a run OOMs, matrix runs need only
  RTX depth (not RGB frames) — drop `width/height` in the SimulationApp
  boot for matrix, keep 1280×720 only for the showcase.
- **~1.5× slower + cold caches**: first run pays a one-time docker image
  re-pull (~20 GB) + RTX shader compile (~10–20 min). Matrix ~4 h,
  showcase ~3 h. Warm the cache with one throwaway `--seed 42 --max-sim-s
  30` run before launching the matrix so the long jobs run warm.

## 1. What we built (done, validated, committed)

**The product**: Ozon Tech hackathon Track 3 — программно-аппаратный
комплекс: RTX-vision classification (B/C/D per the official rules,
dimension-priority, r/R>0.8, guard-banded) + a **linear tilt-tray sorter**
executive in Isaac Sim 6.0.1 (PhysX, real contact physics, zero scripted
freight velocity writes) with a MuJoCo digital twin. Size-independent
divert: the same mechanism carries an 11 mm cube and a 489 mm pouf.

**Верified headline numbers** (evidence in `docs/report/isaac_evidence/`):
- Isaac uniform 14-run matrix (v23 build, `matrix_v23/ev_matrix_v23.tgz`):
  nominal classification **66/66 = 1.0**, nominal routing **66/66 = 1.0**,
  unsafe **0**, containment **1.0**, min command margin 0.95 s,
  velocity writes 0. `GATES: PASS` (floor gate see §3).
- **11 mm cube**: measured 11.0 mm by the macro head (floor 10.8 mm),
  certified B, physically delivered to B (`cube11_proof` in the matrix
  summary).
- MuJoCo twin formal records: **22/22 runs, 235 items, zero failures** on
  v13/v16/v22 builds (`runs/validation_*`); v25 formal run is completing
  locally right now (`runs/validation_20260711_072155`).
- Clearance audit (v25, oriented-box): **arm 0 violations**; all remaining
  pairs are POSITIVE 12.6–13.6 mm gaps in two documented families (throat
  labyrinth cheeks; the designed B-handoff interface) — audit exceptions
  extended after the pull, so the next audit run reports PASS.
- Gentle handling budget: worst touch point ~2.5–3.4 g (documented in
  `docs/report/calculations_vs_simulation.md` §8.1).

**Presentation build** (user-driven "perfect pass", verified on stills in
`docs/report/isaac_evidence/stills_v23b/`): industrial roll cages, end
wheels + slotted return guards (mechanism storytelling), no decal litter,
cushioned blade faces, correct signage, relocated arm (base 8.10/2.42 in
the inter-chute aisle; places through the aperture; fault_jam revalidated
11/11), raised link corridor (1.22 m).

## 2. The final controller (v25) — committed, deployed, twin-green

`hold-at-escapement`: chute-mouth photo-eye occupancy holds the RELEASE at
the escapement (накопитель = buffer; this linear executive cannot
recirculate occupied carriers through the wrap). fault_jam: routing 1.0
with 2 escapement holds. Twin battery green on all prior marginal seeds
(base 123, stress 3/11/99, close 42, fault_jam 42).

## 3. IN FLIGHT — exactly what was interrupted

1. **v25 Isaac edge-pair proof NEVER RAN** (the rerun chain fired with the
   pre-fix script and died on a mkdir PermissionError; the fixed
   `isaac/rerun_edges.sh` — chmod 777 line — is committed). The v23 matrix
   carries `floor=1` in `edge_items_all` (box_l discharged into a mouth
   occupied by the slow-crawling 9 mm rod → dragged past the chute edge).
   The v25 mouth interlock is the fix; it must be PROVEN on the server.
2. **Final uniform matrix on v25** (14 runs) not run — the evidence set to
   stamp into the report should be one build.
3. **Definitive showcase** not rendered (only the v17/v20 preview set with
   old visuals exists locally under
   `docs/report/isaac_evidence/showcase_preview/`).
4. Twin v25 formal matrix finishing locally (check
   `runs/validation_20260711_072155/validation_report.md`; expect 22/22).
5. 4th verification still (`cage_closeup.png`) pulled but was rejected once
   by the reader — re-verify visually on resume.

## 4. RESTART CHECKLIST (in order, ~6 h GPU total)

```bash
# 0. server up, then from the repo root:
git log --oneline -3        # expect ec0189a hold-at-escapement + audit fix
bash tools/deploy.sh        # or: scp isaac/*.py cell/*.py tools/consolidate_isaac_matrix.py isaac/*.sh -> /root/sortmaster/...
                            # (params.py, sorter.py x2, run_isaac.py, run_sim.py,
                            #  audit_clearance.py, rerun_edges.sh are the criticals)

# 1. prove the v25 interlock on the edge pair (~35 min)
ssh -p 40576 root@90.224.159.6 "setsid nohup bash /root/sortmaster/isaac/rerun_edges.sh /root/sortmaster_out/xbelt_matrix_v25 > /root/sortmaster_out/v25_edges3.log 2>&1 &"
# expect: edge_items_all floor=0 (esc_holds >= 1), edge_small 6/6 delivered

# 2. audit (expect PASS now: labyrinth + b_handoff exceptions documented)
#    (same docker run as in isaac/present_pipeline.sh audit block, --out audit_v26)

# 3. full uniform matrix v25 (~2.5 h)
ssh ... "setsid nohup bash /root/sortmaster/isaac/run_matrix.sh /root/sortmaster_out/xbelt_matrix_final > /root/sortmaster_out/final.log 2>&1 &"
python3 tools/consolidate_isaac_matrix.py /root/sortmaster_out/xbelt_matrix_final   # GATES: PASS expected

# 4. definitive showcase on the SAME build (~2 h)
ssh ... "bash /root/sortmaster/isaac/showcase5.sh /root/sortmaster_out/showcase_final 7"
# then capture_stills.sh + verify EVERY camera against the user's defect list

# 5. package
bash tools/fetch_evidence.sh xbelt_matrix_final showcase_final
python tools/make_perception_panels.py … && python tools/make_endcard.py …
# stamp final numbers into: README.md (§banner, §7), isaac/README.md,
# docs/report/final_report_ru.md, docs/report/defense_deck_ru.md,
# docs/report/calculations_vs_simulation.md cross-check columns
# final commit (NO push without explicit user authorization)
```

## 5. Deliverables status (submission per doc-1783095831)

| Требование | Status |
|---|---|
| Классификация: код + правила + guard bands | done (`isaac/perception_rtx.py`, twin `perception/`) |
| Исполнительная часть: цифровая модель + симуляция | done, v25 controller |
| Связка частей + тайминги | done (events chain, command margins, §docs) |
| Валидационная матрица + гейты | v23 evidence local; v25 rerun pending (§4.1–3) |
| Видеодемонстрация | preview local; FINAL render pending (§4.4) |
| Отчет RU + deck | drafted: `docs/report/final_report_ru.md`, `defense_deck_ru.md` — need final number stamps |
| README-хаб + инструкции эксперта | done (§8 README, RU quick-start) |
| Облачные ссылки на большие файлы | pending upload by team |

## 6. Key engineering decisions log (for the report's "почему" sections)

tilt-tray over ARB (11 mm legality); dual-range DWS metrology (macro floor
10.8 mm); guard-banded safe-side classification; deep-drop chutes + mouth
chamfer + cheeks; catch pan + pan watchdog (carrier-guarded); roll-aware
release model; suspect-carrier purge; seat-time gate; physical-flatness
release check; length-aware confirm + stuck window; small-freight reduced
tilt + aim lead; guard bridges; mouth photo-eye + hold-at-escapement;
actuator sizing 150 N·m from the mass sweep; arm through-the-aperture
recovery; stiff thin shells (soft only on thick backed pads); oriented-box
clearance audit with documented labyrinth/handoff interfaces.
