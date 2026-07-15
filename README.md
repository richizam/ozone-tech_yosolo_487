# SortMaster — Intelligent Robotic Product Sorting System

**Ozon Tech Hackathon — Track 3: «Интеллектуальная роботизированная система сортировки товаров»**

A software–hardware complex (ПАК), delivered **entirely in simulation**, that detects a product on the infeed conveyor, classifies it into one of three categories, and physically routes it to the correct processing zone — perception, decision and actuation working as one closed loop.

> **Status: FINAL — both validation matrices green, all gates PASS ✅** — the cell runs closed-loop on sensor data in BOTH engines. **Isaac Sim + RTX (14-run matrix): GATES PASS** — nominal classification **1.0** (66/66 over 6 seeds), nominal routing **1.0**, **zero unsafe misroutes, zero floor drops across all 14 runs** (incl. friction/mass/spacing/offset sweeps and both fault drills), containment **1.0**, min command margin **0.95 s**, zero scripted freight motion (`direct_velocity_writes_nominal = 0` is a hard gate). **MuJoCo twin: 22/22 scenario runs, 235 items, zero failures** (72/72 unit/design tests). Items travel conveyor A at 1 m/s, classify **in motion** on the dual-range RTX depth station (verdict committed **before the escapement**, belt never stops), release synchronized onto their own tilt tray, and discharge by gravity — C/D into aperture-walled roll-cages via 32° brake chutes, B over a powered incline onto the fixed sorter infeed, uncertain freight into a dedicated REVIEW pen. The divert is **size-independent**: the smallest certified parcel is an **11 mm cube**, measured at 11.0 mm by the close-range macro head (certification floor 10.8 mm) and physically delivered to B; a 2 mm card that slips under the escapement's 3 mm skim lands on the debris pan and raises an operator call-out — every failure mode has a designed safe terminal, and the run's exit code proves the outcome. Reproduce: `python -m cell.validate` (any machine) / `bash isaac/run_matrix.sh` (GPU server). This README is the root navigation document required by the submission rules. See [isaac/README.md](isaac/README.md), [docs/report/final_report_ru.md](docs/report/final_report_ru.md) (итоговый отчет) and [docs/report/calculations_vs_simulation.md](docs/report/calculations_vs_simulation.md).

## 1. The task in one page

Items travel on a feed conveyor (**A**, fixed, belt speed **1 m/s**, belt never stops in the base scenario, max item size 500×500×500 mm) and accumulate in a buffer (накопитель) at its end. The complex must take each item from the accumulator and route it to:

| Zone | Category (official) | Meaning |
|---|---|---|
| **B** — main sorter infeed (fixed conveyor, 500 mm wide, h=700) | «Подходит для сортировки» | Fits the sorter: dims > 10×10×10 mm, fits 450×320×320 mm, **no circular cross-section** |
| **C** — oversize cage (roll-cage 1200×800×800, free placement) | «Не подходит для сортировки по габаритам» | Under- or over-size |
| **D** — repack cage (roll-cage 1200×800×800, free placement) | «Не подходит для сортировки без доупаковки» | Has a **circle in some cross-section** |

**Classification rules (priority order matters and is scored):**
1. **Dimensions gate first:** any dimension ≤ 10 mm (undersize) or exceeding 450×320×320 mm (oversize) → **C**. *An oversized cylinder goes to C, not D.*
2. **Then shape:** circle-in-cross-section, formally `r_inscribed / R_circumscribed ≥ 0.8` for a cross-section → **D**.
3. Otherwise → **B**.

Work zone: 6000×10000 mm; A and B positions fixed, C/D cages and everything else placed by us (see the official layout, [doc-1783009942.pdf](doc-1783009942.pdf)).

**A physical prototype is NOT required.** The rules explicitly accept an engineering solution proven by digital models, simulation and calculations — and the readiness matrix (УГТ) awards its maximum (Executive axis level 4) to a *validated simulation cross-checked against calculations*. That is exactly what we build.

> ✅ **Confirmed by the organizers' experts (Q&A, July 2026):** a physical prototype is not required for the final; the maximum level is "either a strong validated simulation / digital model, or a physical prototype". What matters is depth of engineering, realism, and how convincingly workability is proven. Our roadmap climbs exactly that ladder: working sim (min) → detailed engineering: layout, calculations, 3D models, fault handling (mid) → validated sim cross-checked against calculations (max).

## 2. Architecture (one closed loop, not two components)

```mermaid
flowchart LR
    A[Conveyor A<br/>1 m/s] -->|items| VS[Vision station<br/>multi-head depth sensing<br/>+ escapement gates]
    VS -->|measured geometry| PC[Perception core<br/>dims + section-circularity<br/>multi-read fusion]
    PC -->|category + confidence| DE[Decision engine<br/>official rule order<br/>+ safe-side policies]
    DE -->|route command| CTRL[Cell controller<br/>watchdogs, exceptions]
    A --> TT[Tilt-tray sorter train<br/>synchronized induction<br/>one item per tray]
    CTRL --> TT
    TT -->|B tilt + powered incline| B[Zone B: sorter infeed]
    TT -->|C tilt + guided brake chute| C[Zone C: oversize cage<br/>aperture-walled, contained]
    TT -->|D tilt + guided brake chute| D[Zone D: repack cage<br/>aperture-walled, contained]
    TT -->|fallback tilt| R[REVIEW pen<br/>uncertain / double occupancy]
    CTRL -.zero-displacement jam watchdog.-> ARM[Exception arm<br/>chute snags only<br/>route-correct into the cage]
```

Key design decisions (each is defended in the report):

- **Look-ahead classification.** The camera sits upstream: an item is classified *while still travelling* toward the accumulator, so inference latency (~tens of ms) is hidden and the arm receives its command *before* the item arrives — this addresses the scored "Синхронизация по времени" criterion directly.
- **Geometry-first perception (built, validated).** The official rules are purely geometric, so the pipeline measures geometry and implements the *formal* 0.8 criterion — not a black-box class label. The virtual sensor suite mirrors a standard DWS dimensioning tunnel: an overhead depth grid plus a light-section profile scanner with side heads (ray-cast — deterministic, zero OpenGL/GPU dependency, identical headless and in the jury's server). Analysis: min-area-rect dims + pose-robust minor axis from section radii; transverse slices along the item's main axis, densified at both ends; per-slice radial circularity + surface-of-revolution test; end-cap circles must be confirmed by several nearby slices. Multi-read fusion during belt transit follows the official decision order, and uncertain shapes divert to D — never to the sorter. **Validated: 100/99.1/99.1% categories over 330 randomized poses (3 seeds); closed-loop 8-seed campaign: 97.7% end-to-end, 100% executive, zero unsafe errors.** A learned detector (YOLO on synthetic renders) is a Phase-3+ add-on for tracking only; the category never comes from a network. This makes borderline behaviour explainable — worth points in three rubric lines.
- **A tilt-tray sorter routes, the arm recovers.** The executive is a **linear tilt-tray sorter** («поворотные лотки» — one of the mechanism classes named in the official brief): after classification the escapement releases each item synchronized to an inbound EMPTY tray (landing offset measured per item), and the tray's revolute-joint tilt drive discharges it by gravity at its station — C/D south into 32° brake chutes and roll-cages, B north over a powered incline onto the fixed sorter infeed, REVIEW north into a dedicated manual-review pen for uncertain freight. The divert is **size-independent** (an 11 mm cube and a 489 mm pouf ride and discharge identically) — the engineering reason this executive replaced the roller/ARB deck, which cannot divert products under ~50–75 mm. The exception arm handles exactly one thing: a chute snag (watchdog + jam camera) is lifted route-correct into its own cage while that station is locked out; a stuck tray needs no arm at all — its freight rides to the REVIEW fallback / end-line operator call-out by design.
- **Containment by design (перекладка без брака).** The item is guided and bounded at every hop, never thrown and never in free fall: side guides on belt A → dished trays with central end fences (rounds self-centre and cannot roll off; a long box's corners clear the fence ends at discharge) → **deep-drop discharge** (the chute starts 50 mm under the tilted tray lip, so freight can never bridge tray→chute and wedge) → side-railed **single-slope chutes** (32°, μ < tan 32° — physically no stall points, even for recovery drops) with **brake pads** just above the cage floor → roll cages whose receiving wall carries a chute-sized **aperture with flanks and header** plus a full-floor landing mat; the slope crosses the aperture at z ≈ 0.22 m, low and slow. Measured cage-entry speeds ≈ 2 m/s; every delivery is then tracked to the end of the run — the containment metric (below) proves items **stay** in the correct container.
- **The cell shows what it is doing (jury-readable executive).** Every station carries bilingual signage (EN/RU billboards + floor decals): A-infeed, vision station, tilt-tray sorter, B-sorter, C-oversize, D-repack, manual review, exception arm. Route state is live: the item is tinted with its perceived category the moment classification commits; zone-coloured lane markings, chevrons and chute flow-arrows pulse along the active route; destination beacons and station lamps track the active route; an andon tower reads green/amber/red (idle / routing / jam-recovery); the escapement is a visible pop-up stop. All presentation geoms are non-colliding, live in a sensor-invisible geom group (a depth camera does not image painted lines either), and are animated by [cell/visuals.py](cell/visuals.py) — physics and perception results are bit-identical with the layer on or off.
- **Safety by design.** Fenced cell, light curtain across the human access side, e-stop chain, reduced-speed service mode — modelled in the layout and described per the "Безопасность эксплуатации" criterion.

### 2.1 Containment validation — routed ≠ done

Reaching the right zone is necessary, not sufficient: the item must **stay inside the correct container**. Every C/D delivery is therefore tracked from the moment it crosses the cage aperture until the end of the run:

| Metric (per item → aggregated in `summary.json`) | Meaning | Gate |
|---|---|---|
| `contained` / `containment_rate` | never left the cage envelope (walls + aperture zone) after delivery | **must be 1.0 — the run exits non-zero otherwise, same as a misroute** |
| `containment_violations` | count of escape events (`cell_event: containment_violation` with position) | 0 |
| `v_entry` / `cage_entry_speed_max_mps` | speed crossing into the cage — evidence of guided, non-thrown transfer | ≤ 2.2 m/s measured (≈ a 25 cm drop equivalent) |
| `cage_settle_s` | time from cage entry to rest (< 0.1 m/s for 0.5 s) | ~1–2 s measured |
| `cage_max_z` | highest point reached inside the cage — bounce headroom under the 0.8 m wall | ≤ 0.55 m measured |

Latest campaign (`scenarios/base.yaml`, all 11 official items per run): **6/6 oracle seeds and camera-perception runs at 100% routing + 100% containment, zero violations**; the fault drill (`fault_jam.yaml`) recovers an injected snag and still books 100% containment. Full numbers: [docs/report/containment_validation.md](docs/report/containment_validation.md).

### 2.2 Virtual sensor model (what `--perception camera` actually simulates)

The `camera` mode is a **virtual multi-head depth/dimensioning station** (DWS-tunnel class), not a hidden ground-truth feed. One visible config — `VIRTUAL_SENSOR` in [cell/params.py](cell/params.py) — holds every parameter, and each value is consumed by the implementation ([perception/pipeline.py](perception/pipeline.py)); a test suite ([tests/test_sensor_config.py](tests/test_sensor_config.py)) locks config↔implementation equality:

| Parameter | Value | Meaning |
|---|---|---|
| Sensing heads | **4 viewpoints** | overhead depth grid + light-section profilers: top fan + two side heads |
| Overhead head | (5.55, 3.0, 2.2) m | ray-cast depth grid, **3 mm** ground sampling |
| Profiler fans | 0.1° top / 0.2° side, planes every **4 mm** | swept along the belt (physically: one scanner + belt motion at 1 m/s) |
| Measurement window | x ∈ 5.65–6.05 m | items measured **in motion**; every verdict commits before the escapement |
| Capture cadence | 0.12 s (~8 Hz) | multi-read evidence per item, fused per the official rule order |
| Depth noise | σ = 0 mm default, **scenario-tunable** | `sensor: {depth_noise_mm: 2.0}`; 0 = ideal-optics baseline |
| Processing latency | 80 ms | fusion verdict → route command (logged per item as `perception_latency_ms`) |

What the pipeline does with the returns: calibrated **background subtraction** (3σ empty-belt depth map) → morphological denoise → instance isolation with an **identity gate** (a measurement must cover the tracked item's position, else it is a sensor miss — a neighbor's cloud can never be committed under another item's identity) → min-area-rect dims (robust percentile extents under noise) → per-slice radial circularity + surface-of-revolution test → multi-read fusion with **guard bands**: a measurement within the sensor's uncertainty of the 10 mm / 450×320×320 mm limits or the 0.8 circle threshold **cannot take the permissive branch** — it diverts to the safe side (legal-metrology practice). A dimension below the certification floor (10 mm + 2× ground sampling) always diverts to C. Result: at σ = 2 mm the official set still routes **11/11 with zero unsafe errors**.

`camera` vs `oracle` is explicit everywhere: CLI banner, run folder name, `summary.json` (`perception_mode`, `sensor_model`, `oracle_used_for_classification: false` in camera mode) and `events.csv` per item. Oracle mode injects ground truth after a configured latency and exists only as a debugging/regression baseline.

**Why not more cameras? (multi-view trade study.)** The station already fuses 4 viewpoints, and every failure the scenario suite ever produced was a *flow/dynamics* problem, not a coverage problem: tailgaters merging into a cloud (fixed by slug-spaced accumulation + the identity gate), a thin item shoved *onto* a neighbor in a contact queue (fixed by no-contact queue lines), rocking items smearing their multi-read dims (temporal — any number of heads samples the same rocking pose). Extra heads would add ~33% ray cost each and force re-validation of the whole tuned stack while attacking none of the observed failure modes; the one genuinely resolution-limited case (9 mm pen barrel at 3 mm sampling) is neutralized by the certification floor, which routes "cannot certify > 10 mm" to C — exactly what the rules' priority demands. The documented upgrade path, if future evidence demands it: denser ground sampling (3→2 mm) first, a second along-belt overhead head second, 5-sided end-view heads last.

## 3. Ground truth for the official test set (computed, reproducible)

We analysed the 11 official STL models with the reference classifier ([tools/classify_mesh.py](tools/classify_mesh.py)). Dimensions are oriented-bounding-box extents; ratio is max over cross-sections along principal axes of `r_in/R_circ`:

| Item | OBB dims, mm | max r_in/R | Verdict | Why |
|---|---|---|---|---|
| Короб 300×200×200 (box) | 301×200×200 | 0.72 | **B** | fits, square section 0.72 < 0.8 |
| ЛанчБокс (lunchbox) | 201×152×62 | 0.66 | **B** | fits, rectangular |
| Моющее средство (detergent) | 280×260×179 | 0.73 | **B** | oval but below 0.8 — near-borderline |
| Короб 400×400×300 (box) | 401×400×300 | 0.72 | **C** | 400 > 320 → oversize |
| Пуфик (pouf) | 489×489×264 | ≈1.0 | **C** | circular **but oversize — priority rule** |
| Ручка (pen) | 148×13×**9** | ≈1.0 | **C** | 9 mm < 10 mm → undersize — priority rule |
| Бутылка (bottle) | 305×91×91 | 0.997 | **D** | circular section |
| Тарелка (plate) | 209×209×27 | 0.999 | **D** | circular |
| Шлем (helmet) | 352×298×282 | 0.89 | **D** | dome sections circular |
| Мешок (sack) | 202×176×170 | 0.886 | **D** | rounded blob (borderline; see note) |
| Цилиндр («cylinder») | 435×50×43 | **0.867** | **D** | actually a **hexagonal prism**: cos 30° = 0.866 ≥ 0.8, and 435 mm is just under the 450 limit — a designed double-borderline trap |

Machine-readable: [docs/ground_truth/item_ground_truth.json](docs/ground_truth/item_ground_truth.json).

**Documented interpretation choices** (flagged for organizer Q&A, analysed in the report's borderline-cases section):
- *Undersize* is triggered by **any** dimension < 10 mm (a 9 mm-thin pen falls through sorter gaps). Alternative reading (all dims < 10 mm) would flip the pen to D.
- The sack's ratio (0.886) is computed on the convex outer contour; physically a soft sack belongs in repack regardless — both readings agree on D.
- Sections are taken perpendicular to the item's principal axes (an arbitrary oblique cut through a cube can look hexagonal — clearly not the rule's intent).

## 4. Repository structure

```
.
├── README.md                    ← you are here (root navigation document)
├── ROADMAP.md                   ← phased winning plan mapped to the scoring rubric
├── STEP_BY_STEP.md              ← concrete execution guide (commands, order, gates)
├── docs/
│   ├── ground_truth/            ← computed expected categories for the official test set
│   ├── report/                  ← 🔜 final report (PDF)
│   └── presentation/            ← 🔜 defence deck (≤ 7 min)
├── tools/
│   ├── classify_mesh.py         ← reference classifier: STL/STEP → category (CLI)
│   ├── make_borderline_items.py ← designed threshold attacks (GT computed, not asserted)
│   ├── make_edge_items.py       ← §edge set: 11/10 mm cubes, 9 mm rod, 2 mm card
│   ├── consolidate_isaac_matrix.py ← matrix → matrix_summary.json + §8 gates
│   ├── make_perception_panels.py / make_endcard.py ← jury-facing visuals
│   └── fetch_evidence.sh        ← pull videos/logs from the render server
├── cad/
│   ├── layout_v0.py             ← parametric cell layout (single source of truth for all dims)
│   └── out/                     ← generated: top-view PNG, 3D GLB scene, reach_check.json
├── cell/                        ← MuJoCo twin: scene gen, belts + tilt-tray sorter, arm IK,
│   │                              controller, metrics (cell/sorter.py = the executive)
│   ├── run_sim.py               ← entrypoint (--perception camera|oracle, --viewer, --record MP4)
│   ├── validate.py              ← batch validation runner → validation_report.md (one command)
│   ├── visuals.py               ← live presentation state: route lights, lane pulse, andon tower
│   ├── signs.py                 ← bilingual EN/RU signage textures (rendered on demand)
│   └── assets/                  ← true-surface meshes (official + synthetic borderline) + manifests
├── configs/
│   └── validation_matrix.yaml   ← scenario × seed matrix with pass/fail expectations
├── isaac/                       ← Isaac Sim digital twin: USD scene from cell/params.py,
│                                  PhysX closed loop, RTX video + depth captures (§5.1)
├── flow/                        ← SimPy flow model: capacity, queues (physics-measured times)
├── scenarios/                   ← base, borderline, close_spacing, low_confidence,
│                                  fault_jam, failed_transfer, stress_mix
├── tests/                       ← rules + kinematics + containment invariants + sensor-config
│                                  truth + end-to-end smoke (pytest, 68 tests)
├── perception/                  ← ray-cast multi-head sensing + geometric classification
│   ├── pipeline.py              ← DWS sensor suite → dims, sections, category, confidence
│   ├── geometry.py              ← min-area rect, circle fit, envelope primitives
│   └── validate.py              ← randomized-pose campaign → docs/metrics/
├── extracted/                   ← official STL/STEP test-set models (from organizer archives)
├── requirements.txt             ← pinned dependencies
└── Dockerfile                   ← 🔜 one-command reproduction for the expert jury
```

Original organizer documents kept at repo root: task statement ([doc-1783095831.pdf](doc-1783095831.pdf)), scoring rubric ([doc-1783011400.pdf](doc-1783011400.pdf)), allowed software ([doc-1783009063.pdf](doc-1783009063.pdf)), layout scheme ([doc-1783009942.pdf](doc-1783009942.pdf)), STL/STEP archives.

## 5. Quickstart

```bash
# Python 3.12 venv (PyBullet has no Windows wheels; we use MuJoCo — wheels everywhere)
uv venv --python 3.12 .venv          # or: py -3.12 -m venv .venv
uv pip install --python .venv -r requirements.txt

# Classify any mesh — the reference implementation of the official rules
.venv/Scripts/python tools/classify_mesh.py "extracted/doc-1782987733/Stl/Бутылка.stl"
# → category D («Не подходит для сортировки без доупаковки»), r_in/R = 0.997

# Recompute the full ground-truth table
.venv/Scripts/python tools/classify_mesh.py --all "extracted/doc-1782987733/Stl" --json docs/ground_truth/item_ground_truth.json

# Prepare sim assets (official STLs + synthetic borderline items)
.venv/Scripts/python cell/prep_assets.py
.venv/Scripts/python tools/make_borderline_items.py

# Run the FULL CELL end to end (headless), sensing included
.venv/Scripts/python -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception camera --executive table
# → runs/<stamp>_seed42_camera_table/events.csv + summary.json: classification /
#   executive / end-to-end accuracy, unsafe vs conservative errors, containment,
#   cycle_mean/p95/max, perception_latency_ms, command_margin_s, throughput,
#   arm interventions & recovery success
# --executive arm = the preserved arm-primary baseline;
# --perception oracle = ground-truth debug baseline; --viewer to watch live
.venv/Scripts/python -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception camera --executive table --viewer

# ONE COMMAND, ALL EVIDENCE: every scenario x seed with pass/fail gates
.venv/Scripts/python -m cell.validate --matrix configs/validation_matrix.yaml
# → runs/validation_<stamp>/validation_report.md + validation_matrix.csv
#   (+ full events.csv/summary.json per run); exits non-zero on any failure
.venv/Scripts/python -m cell.validate --quick     # first seed of each entry

# Scenario suite (each also runs standalone):
#   base | borderline | close_spacing | low_confidence | fault_jam |
#   failed_transfer | stress_mix
.venv/Scripts/python -m cell.run_sim --scenario scenarios/fault_jam.yaml

# Record a demo MP4 of any run (cameras: overview | top_view | routing | lookahead)
.venv/Scripts/python -m cell.run_sim --scenario scenarios/base.yaml --record demo.mp4 --camera overview --fps 30

# Perception validation campaign: 11 items x N randomized poses, camera data only
.venv/Scripts/python -m perception.validate --poses 10 --seed 5
# → docs/metrics/perception_validation.{csv,json}; gate: >=95% categories

# Flow model: capacity & queueing from measured cycle times
.venv/Scripts/python -m flow.model    # → flow/out/sweep.csv + flow_sweep.png

# Test suite: rules + kinematics + containment invariants + sensor-config
# truth + end-to-end smoke (cycle metrics non-null, margins positive, fault
# drill produces a recovered intervention)
.venv/Scripts/python -m pytest tests -q
```

🔜 `docker compose up` — the exact environment the jury can run.

### 5.1 Isaac Sim digital twin (`isaac/`) — the same cell, real sensors, real drives

The full closed loop also runs in **NVIDIA Isaac Sim 6.0.1** (PhysX 5 physics,
RTX rendering) — a port from the same single source of truth
(`cell/params.py`) that goes a step *beyond* the MuJoCo twin on sensor and
actuator realism:

- **Classification comes from a real rendered sensor — dual-range.** The RTX
  depth station (overhead metrology head + two side profiler heads + a
  close-range MACRO head for small freight) measures each item **in motion**;
  multi-read fusion with frame-completeness gating and legal-metrology guard
  bands applies the official rule order. The guard band scales with the
  measuring head (undersize certification floor = 10 mm + 2×GSD → 16 mm
  overhead, **10.8 mm macro**): the **11 mm cube certifies honestly as
  sortable and is physically delivered to B**, while the 10 mm cube and the
  9 mm pen stay conservatively C.
- **The executive is contact physics with real actuators.** Conveyors carry
  items via PhysX surface velocity (the Isaac Conveyor-Belt-utility
  mechanism) at the designed speeds (belt A at the official 1.0 m/s —
  probe-verified). The **tilt-tray train** carries each item on its own
  dynamic tray (revolute joint + angular position drive, 40 ms command
  pipeline, 160°/s ramp, gain noise, torque saturation); the discharge is
  position-triggered off the chain encoder and confirmed against the item's
  actual departure; every command is logged and `summary.json` certifies
  `direct_velocity_writes_nominal: 0`. Flow discipline is physical pop-up
  stop blades; induction is a synchronized release over a knife-edge nose
  with the landing offset measured per item. Items carry material-class
  physics (cardboard/PET/HDPE/ABS/soft-sack friction, restitution, damping)
  swept ×0.7/×1.3 in validation.
- **Faults are handled on camera data.** A chute-zone depth camera localizes
  a stuck item by background subtraction; the exception arm picks at the
  camera fix (odometry-refined) and places it **route-correct into its own
  cage** while the station is locked out; a dead tilt actuator needs no arm —
  its freight rides to the REVIEW fallback / end-line operator call-out by
  design (`--inject-tray-fault` proves it).

```bash
# on the GPU server, inside the official isaac-sim:6.0.1 container
/isaac-sim/python.sh isaac/run_isaac.py --seed 42 --record --camera overview \
    --out /tmp/sortmaster_out/final_seed42
# → summary.json + events.csv (same metric vocabulary as the MuJoCo runs),
#   frames/*.png -> MP4, vision_rgb/depth stills, jam_located events
```

Cross-engine agreement on the physics envelope (zero unsafe errors,
containment 1.0, cage-entry speed and bounce height inside the MuJoCo-measured
bounds, command margin ≥ 0.5 s) is exactly the «validated simulation
cross-checked» УГТ bar — and the Isaac twin adds the sensor-in-the-loop proof.
Evidence: [docs/report/isaac_evidence/](docs/report/isaac_evidence/); package
details: [isaac/README.md](isaac/README.md).

## 6. Toolchain (all from the organizers' allowed list)

| Purpose | Tool |
|---|---|
| Physics simulation of the cell | **Two engines, one cell** (both on the organizers' allowed list). **MuJoCo 3** — the deterministic validation engine (22/22 scenario matrix, headless, identical on any jury box). **NVIDIA Isaac Sim 6.0.1 (PhysX 5 + RTX)** — the high-fidelity digital twin of the SAME cell, built from the same `cell/params.py` single source of truth and run on the team GPU server (RTX 5090); cross-engine agreement of the physics envelope is itself validation evidence (see §5.1). PyBullet was the original pick but publishes **no Windows wheels** — verified empirically; decision documented in the report |
| Discrete-event flow & cycle time | **SimPy** (+ pandas, matplotlib for metrics) |
| Perception | **OpenCV**, **Open3D**, **Trimesh**, **Shapely**; **Ultralytics YOLO** for belt detection (synthetic training data rendered in **Blender**) |
| CAD / layout | **FreeCAD** (reads the official STEP models), exports STEP/STL |
| Packaging & reproducibility | **Python 3.11+**, **Docker**, **Git**; models exported to ONNX; scenes as URDF |
| Deliverable formats | PDF (report), MP4 (video), CSV/JSON (metrics), URDF/STL/STEP (models) |

## 7. Scoring map — where the 130 points live

| Rubric section | Max | Our vehicle |
|---|---|---|
| 1. Presentation | 10 | 7-min deck + rehearsed demo narrative |
| 2. Readiness matrix (УГТ 4×4) | 20 | CV L4 × Executive L4 = validated sim vs calculations |
| 3. Category correctness | 20 | Formal-rule classifier + borderline analysis + test-set demo |
| 4. Executive part & manipulation | 30 | Tilt-tray sorter in real physics: full cycle (signal → tilt → discharge → tray re-flattens), size-independent divert incl. 11 mm cube, per-shape behaviour swept, safety concept |
| 5. Performance & timing | 20 | Measured cycle_mean/p95/max + perception_latency_ms + command_margin_s per item; look-ahead sync (verdict committed before the escapement; min command margin > 1.4 s; belt never stops); fault/overload scenario suite |
| 6. Integration & realism | 15 | One message bus, category → command trace, industrially plausible cell |
| 7. Report, reproducibility, README | 15 | This README, Docker one-command run, full report |

## 8. For the expert jury (проверка решения)

> **Быстрый старт (RU).** Всё решение проверяется двумя командами.
> MuJoCo-твин (любая машина, CPU): `python -m cell.validate` — полная матрица
> 22 прогонов с гейтами, отчет в `runs/validation_*/validation_report.md`.
> Isaac-твин (GPU, официальный контейнер `isaac-sim:6.0.1`):
> `bash isaac/run_matrix.sh /out/m && python3 tools/consolidate_isaac_matrix.py /out/m`
> — 14 прогонов + консолидация с жесткими гейтами (ненулевой выход при любом
> провале). Одиночный прогон с видео и кадрами сенсоров:
> `/isaac-sim/python.sh isaac/run_isaac.py --seed 42 --record --depth-stills 3`.
> Итоговый отчет: [docs/report/final_report_ru.md](docs/report/final_report_ru.md).

- **Run instructions with pinned versions** — `requirements.txt`; Isaac runs inside the official `nvcr.io/nvidia/isaac-sim:6.0.1` container (exact mount set in [isaac/README.md](isaac/README.md)).
- **One-command evidence:** `python -m cell.validate` (MuJoCo, 22 runs × gates) and `bash isaac/run_matrix.sh <out>` + `tools/consolidate_isaac_matrix.py <out>` (Isaac, 14 runs + hard gates). Both exit non-zero on any failure.
- **Tunable input parameters — MuJoCo** (per scenario YAML or CLI): item mix and spawn order (`items`, `--seed`), arrival intensity (`spawn_gap_s`), sensor noise (`sensor: {depth_noise_mm}`), classification policy (`classification: {...}`), fault injection (`inject_jam: {slug, at_x}`), perception mode (`--perception camera|oracle`).
- **Tunable input parameters — Isaac** (CLI of `isaac/run_isaac.py`): `--seed`, `--items <subset>`, `--manifest-extra` (adds the edge set incl. the 11/10 mm cubes, Ø9 mm rod, 2 mm card), `--spawn-gap A,B`, `--spawn-offset-y`, `--friction-mult`, `--mass-mult`, `--inject-jam SLUG@Y` (chute snag → arm recovery), `--inject-tray-fault ZONE` (dead tilt actuator → end-line call-out), `--perception rtx|oracle`, `--record --camera ... --fps`, `--depth-stills N` (sensor's-eye RGB/depth/macro frames).
- **Prepared scenarios:** nominal (`base`), borderline threshold attacks (`borderline`), close-spaced arrivals (`close_spacing`), degraded sensing (`low_confidence`), jam recovery (`fault_jam`), failed transfer (`failed_transfer`), 1.3× overload (`stress_mix`); the Isaac matrix additionally sweeps friction ×0.7, mass ×1.3, close spacing, off-centre feed and runs both fault drills.
- **Bring your own STL (требование экспертов):** любой свой тестовый STL можно
  прогнать через официальные правила и через ФИЗИЧЕСКИЙ контур:
  `python tools/expert_check.py path/to/model.stl` — класс + полная трасса
  решения (габариты OBB, r/R по секциям, категория, причина);
  `python tools/expert_check.py model.stl --register --mass 0.4` — регистрирует
  товар для симуляторов и печатает готовые команды запуска (CPU-твин без GPU
  или Isaac с RTX-перцепцией). Сводка прогона показывает, как ЖИВАЯ перцепция
  измерила товар, какой класс назначила и куда физически доставила.
- **GPU fallback:** если тестовый стенд отличается от нашей среды, полный
  контур проверяется БЕЗ GPU цифровым двойником MuJoCo
  (`python -m cell.validate`, детерминированный, 22 прогона × гейты), а
  RTX-результаты приложены как видео + матричные артефакты
  ([docs/report/isaac_evidence/](docs/report/isaac_evidence/)).
- **Engineering pack:** [архитектура ПАК](docs/report/architecture.md) ·
  [схема компоновки](docs/report/figures/layout_plan.png) (генерируется из
  `cell/params.py`) · [кинематика каретки](docs/report/figures/carrier_kinematics.png) ·
  [спецификация узлов](docs/report/node_spec.md) ·
  [before/after доказательства механики](docs/report/isaac_evidence/xbelt/proof_pack/README.md).
- **Perception evidence (jury panels):** side-by-side «RGB + RTX depth +
  segmentation + fused dims + route decision» panels built from real sensor
  frames by `tools/make_perception_panels.py` —
  [panels](docs/report/isaac_evidence/xbelt/perception/panels/) ·
  [edge-case panels](docs/report/isaac_evidence/xbelt/perception/panels_edge/)
  (11 mm cube→B vs 10 mm cube→C at the 10.8 mm certification floor) ·
  [perception_demo.mp4](docs/report/isaac_evidence/xbelt/perception/perception_demo.mp4) ·
  final [metrics end card](docs/report/isaac_evidence/xbelt/videos_final/endcard.png).
  The overlaid segmentation masks and item point clouds are the measuring
  pipeline's OWN export (`RTXPerception.export_masks` → `vision_mask_*.png`,
  `vision_cloud_*.npy`, incl. the close-range macro head), pixel-aligned with
  the saved stills — not a visualization-side re-derivation.
- **Cloud links** (large binaries: full showcase videos, sensor stills/NPY, CAD sources) — collected here with per-link descriptions when uploaded 🔜; the in-repo evidence set lives under [docs/report/isaac_evidence/](docs/report/isaac_evidence/).

## 9. Team & contacts 🔜

| Role | Person |
|---|---|
| Lead / integration | — |
| Perception & ML | — |
| Simulation & mechanics | — |
| Report, video, presentation | — |

---

*Language note: working docs are in English; the submitted README/report/presentation will be delivered in Russian (the jury's language) — translation is a scheduled roadmap step, not an afterthought.*
