# ROADMAP — how we win Track 3

Simulation-only entry, engineered to score. Total rubric: **130 pts**. Realistic winning target: **≥ 110** (most teams lose 30+ points on integration, timing evidence and reproducibility — we bank those).

The plan is phased in relative weeks (**W0 = today** → **W6 = submission/defence**; re-pin once the official calendar is known). Each phase has an exit gate — do not advance while a gate fails. The organizers' own advice (task statement, p.14) is our spine: *build the thinnest end-to-end loop first, then deepen each part, polish last.*

---

## Phase 0 — Bootstrap (W0, days 1–2) ✅ largely done

**Goal:** understand the battlefield, lock the strategy.

- [x] Extract and read all four organizer documents (task, rubric, software list, layout).
- [x] Decode the classification rules incl. priority order and the 0.8 circle criterion.
- [x] Compute ground truth for the 11 official STL models → [docs/ground_truth](docs/ground_truth/item_ground_truth.json); traps identified (hex-prism "cylinder" 0.867, 9-mm pen, oversize round pouf).
- [x] Choose stack: PyBullet + SimPy core, OpenCV/Open3D/YOLO perception, FreeCAD layout, Docker delivery.
- [ ] Create GitHub repo (submission hub), push docs + `tools/classify_mesh.py`, invite team.
- [x] ~~Confirm simulation-only participation is viable~~ — **answered by organizers' experts:** prototype not required; max level = strong validated simulation. Strategy locked.
- [x] First CAD artifact pulled forward: parametric cell layout `cad/layout_v0.py` (top view + 3D GLB + arm-reach validation; the validator already caught and fixed an out-of-reach B placement).
- [ ] Remaining questions for organizers: undersize = any-dim < 10 mm? sack expected category? target throughput for the sorter infeed?

**Exit gate:** repo online; every teammate can run the reference classifier and reproduce the ground-truth table.

## Phase 1 — Thin end-to-end loop (W1)

**Goal:** the smallest thing that is already a ПАК: item spawns → moves on belt → is classified → arm routes it to B/C/D. Ugly is fine; *connected* is mandatory.

- [ ] PyBullet scene v0: work zone 6000×10000, conveyor A (1 m/s, kinematic belt), accumulator stop, zone B conveyor, C/D cages as boxes, official STL items spawnable.
- [ ] Oracle classifier v0: feed mesh ground truth directly to the controller (perception comes later — the loop must close first).
- [ ] Arm v0: UR10 URDF + single suction gripper (fixed-constraint grasp), scripted pick from accumulator, place to B / drop to C / D.
- [ ] Controller v0: state machine (IDLE → PICK → PLACE → HOME), simple queue, event log to CSV.
- [ ] First cycle-time measurement; SimPy flow model v0 fed with that number.

**Exit gate:** one command runs a 10-item mixed scenario, all 11 item types route to their ground-truth zone, a metrics CSV comes out. **This alone is already УГТ CV-1 × Exec-3 territory.**

## Phase 2 — Real perception (W2)

**Goal:** replace the oracle with sensing, without breaking the loop.

- [ ] Overhead RGB-D camera in sim upstream of the accumulator (look-ahead station).
- [ ] Belt-plane calibration; background subtraction / depth segmentation → object mask.
- [ ] Point cloud → oriented bounding box → dimensions (mm) with error stats vs ground truth.
- [ ] Cross-section circularity from depth silhouette + multi-height slices → `r_in/R` estimator; validate against mesh-computed truth on all 11 items.
- [ ] YOLO (Ultralytics) belt detector trained on synthetic renders (Blender/PyBullet domain-randomized: pose, lighting, texture) — used for detection/tracking, *not* for category.
- [ ] Confidence model: geometric margins (distance of dims to limits, ratio to 0.8) → per-item confidence; low-confidence policy (route to D + flag) documented.
- [ ] Perception → decision → controller over one message bus (in-process pub/sub; ROS 2-shaped interfaces but zero-dependency).

**Exit gate:** ≥ 95% correct categories on randomized-pose test runs of the 11 items *from camera data alone*; measured dims within ±5 mm; full trace item → category → command → placement in the log.

## Phase 3 — Executive part depth (W3) — *the biggest rubric block (30 pts)*

**Goal:** manipulation quality, geometry coverage, safety — engineered, not hand-waved.

- [ ] Hybrid gripper: vacuum for boxes/flat/plate; adaptive two-finger for sack, bottle, pen-class items; grasp-point selection from perceived OBB + surface normal.
- [ ] Per-shape grasp library + placement strategies (gentle place on B, controlled drop into cages; no tossing — "отсутствие избыточного брака").
- [ ] Failed-grasp detection (gripper feedback / pose check) → retry ×2 → divert-to-D fallback.
- [ ] Layout optimization (v0 exists in `cad/`): refine arm base & cage positions from sim data; rebuild as a CAD assembly in FreeCAD (or KOMPAS-3D if licensed) and export **STEP + dimensioned PDF drawing** — the jury-facing formats; add gripper concept model, kinematic scheme, node specification (BOM).
- [ ] Safety concept: fenced cell + light curtain + e-stop chain + service mode; one diagram + one paragraph, referenced from the report.
- [ ] Nominal cycle-time budget per phase (perceive ≪ travel time, pick ~1.5 s, place 1.5–2.5 s by zone) cross-checked: analytic calc vs simulation — *this cross-check is what buys Exec level 4 ("validated model vs calculations")*.
- [ ] Decision gate: if sustained throughput < target (define ~600 items/h), evaluate plan-B executive (tri-directional roller diverter for B-flow, arm handles C/D only) — keep whichever wins, document the trade-off either way.

**Exit gate:** 50-item randomized run: ≥ 95% routed correctly on first attempt, 100% after retries/fallback, zero dropped items, cycle time distribution recorded.

## Phase 4 — Timing, faults, evidence (W4) — *20 pts nobody else bothers to prove*

**Goal:** turn "it works" into measured, attackable-proof evidence.

- [ ] Synchronization analysis: inference latency + command latency vs belt travel distance; show the look-ahead margin quantitatively (item classified X mm / Y ms before accumulator).
- [ ] Fault scenario suite (`scenarios/faults/`): items nose-to-tail, failed grasp, item jam at accumulator, conveyor stop signal, sensor dropout, mid-cycle e-stop. Each: detection mechanism → handling → recovery, all logged.
- [ ] SimPy campaign: 1000-item runs across item-mix distributions; utilization, queue lengths, accumulation limits, throughput histograms → CSV + plots.
- [ ] Borderline campaign: synthetic variants sweeping dims across 10 mm / 450×320×320 and ratio across 0.8 (scaled hexagons ↔ octagons, 449 mm vs 451 mm boxes) → correctness-vs-margin curve.
- [ ] Metrics pack frozen: category accuracy, routing accuracy, first-attempt grasp rate, cycle time (mean/p95), throughput, fault recovery rate.

**Exit gate:** every performance claim in the future report links to a script + CSV an expert can regenerate with one command.

## Phase 5 — Packaging: report, video, reproducibility (W5) — *15 pts + УГТ evidence*

**Goal:** make the jury's job effortless.

- [ ] Final report (PDF, in Russian), mirroring the required structure: approach & data; classification logic; executive part design; the *link* between them (category → control signal, timing constraints, ambiguous cases); checks performed; limitations; development directions.
- [ ] README finalized in Russian as the single navigation hub: quickstart, pinned versions, expert-tunable parameters, scenario catalogue, cloud links with descriptions.
- [ ] Docker image: `docker compose up` → headless sim run + metrics out; tested on a clean machine (this is how the organizers' server env will run it).
- [ ] Video (MP4): full-cycle demo — nominal mix, then borderline items, then a fault recovery; overlay live category/decision/latency readouts. Cut for ≤ 3 min inside the 7-min defence.
- [ ] Upload large binaries (weights, full video, CAD sources) to the organizers' cloud storage; link + describe each in README.

**Exit gate:** a teammate (or friend) with zero context reproduces the demo from README alone, no questions asked.

## Phase 6 — Defence (W6)

**Goal:** convert the work into the 10 presentation points and survive Q&A.

- [ ] Deck per the recommended structure: team → task & approach → classification logic → A/B/C/D routing scheme → executive part → **proof** (metrics, sim, calcs) → tools & why → limitations & next steps.
- [ ] 7-minute discipline: 2 full rehearsals with timer; demo pre-rendered (never live-debug on stage), live sim as backup flourish.
- [ ] Q&A drill from the rubric's own probes: priority of rules, the hex-prism trap, undersize interpretation, sync latency numbers, what happens on failed grasp, why PyBullet is representative, safety zones.
- [ ] One-slide answer ready for "why no physical prototype": the rules' own words + validated-sim = Exec L4 + calculation cross-check.

---

## Risk register

| Risk | Blast radius | Mitigation |
|---|---|---|
| Cycle time too slow with a single arm | 10 pts (perf) + realism | Phase-3 decision gate → roller-diverter plan-B already scoped |
| Sim doesn't run in the jury's server env | reproducibility + УГТ demotion | Headless-first design, Docker from W3, pinned versions, no GPU-mandatory path |
| Ambiguous rule readings (pen, sack) | up to 10 pts (test-set correctness) | Ask organizers early (Phase 0); implement both readings behind a config flag; document choice |
| Soft-body items (sack) unstable in physics | manipulation credibility | Rigid proxy + documented assumption; optional soft-body demo only if time allows |
| Team time runs out | everything | The phase order *is* the mitigation: a submittable end-to-end solution exists from W1 onward, every later week only adds points |
| 7-min overrun at defence | presentation pts | Rehearsals with hard timer; pre-rendered demo |

## Scoring forecast (be honest, then beat it)

| Rubric block | Max | Conservative | Target |
|---|---|---:|---:|
| Presentation | 10 | 7 | 9 |
| УГТ matrix (CV × Exec) | 20 | 15 (L3×L3) | 20 (L4×L4) |
| Classification correctness | 20 | 16 | 19 |
| Executive & manipulation | 30 | 22 | 27 |
| Performance & timing | 20 | 14 | 18 |
| Integration & realism | 15 | 11 | 14 |
| Report & reproducibility | 15 | 12 | 15 |
| **Total** | **130** | **97** | **122** |

The conservative line is what Phase 1–3 alone deliver. Phases 4–5 are cheap relative to the points they add — that asymmetry is the strategy.
