# STEP BY STEP — execution guide

The [ROADMAP](ROADMAP.md) says *what* and *when*; this file says *how*, step for step, at the keyboard level. Everything runs on Windows 10/11 or Linux, no GPU required (GPU only speeds up YOLO training). All software is from the organizers' allowed list.

---

## Step 0 — Environment (30 min)

```powershell
# 0.1 Python 3.11+ already present (3.14 works). Create the project venv:
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# 0.2 Core dependencies — pin them from day one (reproducibility is scored):
python -m pip install numpy scipy trimesh shapely pybullet simpy opencv-python open3d matplotlib pandas pyyaml
python -m pip freeze > requirements.txt

# 0.3 Git + GitHub (the repo IS the submission hub):
git init
git add README.md ROADMAP.md STEP_BY_STEP.md tools/ docs/ requirements.txt
git commit -m "Bootstrap: docs, reference classifier, ground truth"
# create private GitHub repo, add teammates + (later) the organizers' checker account
```

Notes:
- Keep the official PDFs and the `extracted/` STL/STEP sets in the repo (they are small enough and experts will look for them).
- If `open3d` has no wheel for your Python yet, drop to Python 3.11 in the venv — do not fight it, nothing else needs 3.14.

## Step 1 — Reference classifier = the rules, executable (already built, keep improving)

`tools/classify_mesh.py` implements the official rules verbatim:

1. OBB extents via `trimesh.bounds.oriented_bounds` → dims sorted desc.
2. Gate 1: any dim < 10 mm → **C**; dims (desc) exceed (450, 320, 320) → **C**.
3. Gate 2: slice the mesh perpendicular to each principal axis at 19 stations (`trimesh.intersections.mesh_plane`); per section take the convex outer contour; `R` = min enclosing circle (`shapely.minimum_bounding_radius`), `r` = Chebyshev center radius (exact LP for convex polygons, `scipy.optimize.linprog`); if max `r/R ≥ 0.8` → **D**.
4. Else → **B**.

Verify against the committed ground truth whenever you touch it:

```powershell
python tools/classify_mesh.py --all "extracted/doc-1782987733/Stl" --json docs/ground_truth/item_ground_truth.json
```

Expected: Box S / LunchBox / Detergent → **B**; Box L / Pouf / Pen → **C**; Bottle / Plate / Helmet / Sack / Cylinder(hex) → **D**. If a change flips any verdict, you broke a rule — the JSON diff shows which.

**Unit tests to add now** (`tests/test_rules.py`, pytest): perfect cylinder → D; cube → B (0.707 < 0.8); regular hexagonal prism → D (0.866); regular pentagonal prism → B (cos 36° = 0.809 → actually D! good — that's the kind of edge you want to *know*); 451 mm box → C; 9 mm sheet → C; oversized cylinder → C not D (priority).

> **Build log (July 5):** Steps 0–3 are DONE — with one engine change: **MuJoCo instead of PyBullet**, because PyBullet publishes no Windows wheels on PyPI (Linux-only; verified) while MuJoCo 3 ships wheels for Windows and Linux. The package layout below was kept (`cell/…`), the belt is contact-gated with deceleration zones and an escapement gate, the arm is a 4-axis palletizer with closed-form IK (`cell/arm_ik.py`), and grasp verification + watchdog abort/retry are already in. Venv is Python 3.12 via `uv` (3.14 has no wheels for several sim packages). Results: 100% routing on 6 seeds, cycle 2.35 s, capacity ≈1000 items/h.

## Step 2 — cell v0: close the loop with an oracle (3–4 days) ✅

Create `cell/` as a package:

```
cell/
├── world.py        # floor 6000×10000, conveyor A, accumulator, zone B belt, C/D cages (URDF/primitives)
├── conveyor.py     # kinematic belt: applies 1 m/s surface velocity to contacting bodies
├── items.py        # spawn official STLs (trimesh → pybullet collision + visual), randomized pose/order
├── arm.py          # UR10 URDF (standard model), IK via pybullet.calculateInverseKinematics, joint-speed limits
├── gripper.py      # v0: suction = fixed constraint on contact; later: finger gripper
├── controller.py   # state machine IDLE→WAIT_ITEM→PICK→TRANSFER→PLACE→HOME; consumes category messages
├── bus.py          # tiny in-process pub/sub: topics `item_detected`, `item_classified`, `routing_cmd`, `cell_event`
├── metrics.py      # every event timestamped → runs/<ts>/events.csv
└── run_sim.py      # entrypoint: --scenario scenarios/base.yaml --gui/--headless --seed N
```

Layout constants from the official scheme ([doc-1783009942.pdf](doc-1783009942.pdf)): work zone 6000×10000 mm; A belt width 500, top at h=700, fixed; B belt same profile, fixed relative to A (see drawing) — place the arm base and the two 1200×800×800 cages so that accumulator, B infeed and both cages are inside the UR10 reach (~1300 mm); check reach in sim, adjust base once, freeze.

Oracle mode: `items.py` tags each spawned item with its ground-truth category; `bus` publishes it as if perception had run. **The point of v0 is the closed loop, not realism.**

Acceptance run:

```powershell
python -m cell.run_sim --scenario scenarios/base.yaml --gui --seed 42
# 10 mixed items; watch: belt carries → stops at accumulator → arm picks → correct zone; events.csv written
```

Commit tag: `v0-loop-closed`. From this moment you always have something submittable.

## Step 3 — SimPy flow model (1 day, parallelizable)

`flow/model.py`: arrival process (configurable inter-arrival), accumulator queue (finite capacity!), arm as a resource with cycle-time distribution measured in Step 2, three sinks. Outputs: throughput, queue length over time, utilization, p95 waiting time → `flow/out/*.csv` + PNG plots.

Run 1000-item campaigns over arrival-rate sweeps; find the arrival rate where the accumulator overflows — that number *is* your claimed capacity, and the plot goes in the report.

## Step 4 — Perception replaces the oracle (1–1.5 weeks)

`perception/` package, stage by stage — keep the oracle switchable (`--perception oracle|geometric|full`) so regressions are always attributable:

1. **Camera** (`camera.py`): PyBullet synthetic RGB-D, overhead, mounted upstream so classification finishes before arrival (look-ahead). Save intrinsics; calibrate belt plane from 4 markers.
2. **Segmentation** (`segment.py`): depth-difference vs empty belt → mask → connected components; OpenCV. (Belt is textureless in sim — depth is the honest signal and transfers to a real RealSense-class sensor, which is your "industrial realism" argument.)
3. **Measurement** (`measure.py`): mask + depth → point cloud (Open3D) → remove belt plane → OBB → dims. Log dims error vs ground truth per frame; target ±5 mm.
4. **Circularity** (`circularity.py`): slice the observed point cloud at several heights → 2D contour → same r/R math as the reference classifier (reuse the exact functions from `tools/classify_mesh.py` — one implementation, two callers; divergence here is a bug class you eliminate by construction). The camera sees only the top surface; either rotate analysis to use the silhouette + side profile from a second angled camera, or fuse frames as the item moves — document whichever you ship.
5. **Detector** (`detect.py`, optional but strong): Ultralytics YOLO n-size, trained on ~2k synthetic renders (PyBullet/Blender, randomized pose/light/texture/distractors) for robust detection & tracking under occlusion/nose-to-tail arrivals. Export ONNX for the report. Category still comes from geometry.
6. **Decision** (`decision/engine.py`): rules with priority + confidence = margin to thresholds (|dim−limit|, |ratio−0.8|); low-confidence policy: route to D + `flagged=true` in the log (justify: wrong-to-B is the expensive error; D is human-inspected anyway).

Acceptance: 100-run randomized campaign, `--perception full`, ≥ 95% category accuracy, zero silent failures (every item has a decision trace).

## Step 5 — Executive depth: grasping, faults, safety (1 week)

1. **Hybrid gripper** (`gripper.py` v2): suction for flat/boxy tops (plate, boxes, lunchbox, detergent); parallel fingers for bottle, hex-prism, pen-class, sack. Grasp-point selector: top-surface centroid + normal for suction; antipodal width-fit for fingers from the OBB.
2. **Placement per zone**: B = gentle place on moving belt (match belt speed vector before release — nice detail, huge realism points); C/D = controlled low drop into the cage, spiral fill pattern so 50 items don't pyramid.
3. **Fault handling** (`controller.py` v2): grasp verification (constraint force / relative pose after lift); retry ×2 with re-perceived pose; then divert to D with flag. Jam detection: accumulator occupancy sensor + timeout → pause intake logic. Conveyor-stop input honored. E-stop → safe halt state; resume procedure.
4. **Safety layout** (FreeCAD, `docs/cad/`): fence line, light curtain at the human aisle, e-stop locations, service access to cages. Export STEP + one dimensioned PNG for the report/deck.
5. **Timing budget table** (`docs/timing.md`): per-phase times (analytic) vs simulated distributions — the Exec-L4 "validated vs calculations" evidence. Keep it honest: where sim disagrees with calc, explain why.

Acceptance: 50-item run — 100% correct final routing (after retries), first-attempt grasp ≥ 90%, cycle time p95 within budget, all fault scenarios in `scenarios/faults/` pass with logged detection→recovery.

## Step 6 — Borderline & robustness campaigns (2–3 days)

- `scenarios/borderline.yaml`: programmatically scaled variants — boxes at 448/452 mm, sheets at 9/11 mm, prisms pentagon→hexagon→octagon (ratios 0.809/0.866/0.924), squashed bottles (ellipse ratio sweep across 0.8).
- Output: accuracy-vs-margin curve (PNG + CSV). This chart is your answer to rubric line "Устойчивость определения категории в пограничных случаях" (5 pts) and half of the УГТ CV-axis level 3→4 argument.
- Nose-to-tail arrivals and touching items: show the tracker splits or the system degrades safely (flags, not misroutes).

## Step 7 — Reproducibility packaging (2 days, do not defer to the end)

```dockerfile
# Dockerfile (sketch): python:3.11-slim, pip install -r requirements.txt (pinned),
# default CMD = headless base scenario writing /out/events.csv + summary.json
```

- `docker compose up` must produce metrics on a clean machine — test on a teammate's laptop, not just yours.
- README (Russian final version): quickstart, versions, the exact parameters an expert may vary (belt speed, mix, seed, threshold, arm speed), scenario catalogue with one-liners, cloud-storage links each with a description.
- Everything an expert could ask "can I see X?" about → a path or link in README.

## Step 8 — Report, video, deck (parallel, final week)

- **Report** (PDF): follow the mandated structure 1:1 (approach/data → classification logic → executive design → the connection between them incl. timing → verifications → limitations → next steps). Reuse the campaign plots; every number cites its CSV.
- **Video** (MP4, ≤3 min core): nominal mix montage → borderline items with on-screen ratio/dims overlay → one fault + recovery → cycle-time dashboard. Record from sim at fixed camera angles; narrate or subtitle in Russian.
- **Deck** (≤7 min): recommended structure from the task statement, one slide per point, the УГТ matrix self-assessment slide (claim L4×L4 and show *why*), backup slides for the Q&A drill list in the ROADMAP.

## Step 9 — Dry run & submit

- Fresh-clone rehearsal: `git clone` → README → demo works, on a machine that never saw the project.
- Submission checklist: README hub ✅ report ✅ deck ✅ code ✅ sim files ✅ run instructions with versions ✅ cloud links described ✅ video ✅.
- Two timed rehearsals of the defence; assign who answers what (rules edge cases / perception / mechanics / metrics).

---

## Standing rules while you work

1. **Never break the loop.** Any feature that can't run end-to-end by evening goes behind a flag.
2. **One implementation of the rules** (`tools/classify_mesh.py` functions) shared by ground truth, perception and tests.
3. **Every claim gets a CSV.** If it isn't logged, it didn't happen — the rubric pays for evidence, not adjectives.
4. **Seeded randomness everywhere** (`--seed`); the jury must be able to replay exactly what the video shows.
5. **Commit at every green gate**, tag the milestones (`v0-loop-closed`, `v1-perception`, `v2-executive`, `v3-evidence`, `v4-submission`).
