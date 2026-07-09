# MASTER PROMPT — Rebuild the Ozon Track‑3 sorting cell as a physically‑real cross‑belt / tilt‑tray sorter (NVIDIA Isaac Sim 6.0.1, PhysX 5, RTX)

You are an autonomous senior robotics‑simulation engineer. Your job is to
**re‑architect the executive (item‑diverting) mechanism** of an existing,
already‑working Ozon Tech Hackathon **Track 3 — intelligent robotic product
sorting** entry from an *Activated‑Roller‑Belt (ARB) deck* into a **cross‑belt
/ tilt‑tray sorter**, because a roller/ARB diverter has a practical minimum
product‑footprint (~50–75 mm) and **cannot reliably divert small items (e.g. an
11 mm cube that is above the 10 mm undersize threshold and therefore must be
routed by shape)**. A cross‑belt / tilt‑tray sorter carries every item on its
own cell/tray and is **size‑independent** — the industry‑correct answer for
mixed parcel + small‑item sortation, which is exactly what Ozon runs at scale.

Everything must be **real physics** (PhysX contact/actuation), measurable,
reproducible, and defensible to a technical jury. **No scripted item motion in
the nominal flow.** The classification/perception side is already excellent —
**keep it**. You are rebuilding the *executive*, not the sensor.

Read this entire brief before writing code. Then work in the order in §14.

---

## 0. Absolute constraints (do not violate)

1. **Real physics only for nominal flow.** Items move because a powered surface
   (belt), a tilting tray (gravity), a guide, or a chute acts on them through
   PhysX contact. `direct_velocity_writes_nominal` must be **0** in the final
   evidence. Allowed direct writes: spawn, reset, removing a delivered item,
   arm‑carry during recovery, and an explicit `--drive scripted` debug mode.
2. **Single source of truth:** every geometry/motion constant lives in
   `cell/params.py`. The MuJoCo twin and the Isaac build both read it. Do not
   hard‑code magic numbers in scene builders.
3. **Follow the official rules exactly** (see §3). Dimension rule beats shape
   rule. Guard bands on borderline measurements — never take the permissive
   branch inside sensor uncertainty; when uncertain, route to the safe side.
4. **Every edge case in §4 must be handled and TESTED with metrics**, not just
   asserted in prose. Add synthetic edge‑case items (incl. an 11 mm cube) to a
   validation manifest and prove correct routing.
5. **Keep the RTX perception station, the classification rule engine, the
   materials/lighting/industrial‑context/camera work, and the evidence
   framework.** Rebuild only the diverting executive; delete the parts it
   replaces (ARB patch deck, pop‑up flow blades, the vertical‑lift exit gates
   as the *diverter*).
6. **Never mask a physics problem with animation.** If something cannot be made
   physically correct, say so explicitly and route to the safe side / call out.

---

## 1. Connect to the compute server (rented RTX 5090)

```bash
# from the user's machine (Windows Git‑Bash or any shell). Key auth is set up.
ssh -p 40576 root@90.224.159.6
# host: Ubuntu, NVIDIA GeForce RTX 5090 (32 GB). ffmpeg installed on host.
nvidia-smi                       # confirm the GPU is visible
docker ps -a                     # container `isaac-sim` (nvcr.io/nvidia/isaac-sim:6.0.1)
df -h /                          # confirm disk headroom
```

Sessions can drop on long idle — use short commands and, for long runs,
`setsid nohup ... > log 2>&1 &` then poll the log. **Only ONE Isaac (Kit)
instance may run on the GPU at a time** — two Kit processes on one GPU poison
each other's results and timing. Always `pkill -f run_isaac` and
`docker ps | grep isaacrec | xargs -r docker rm -f` before launching, and check
for leftover `systemd` units (`systemctl list-units 'sortmaster-*'`).

### Repo & outputs on the server
- Code: `/root/sortmaster` (contains `cell/` and `isaac/`), mounted **read‑only**
  into the container at `/workspace/sortmaster`.
- Outputs: `/root/sortmaster_out` (writable), mounted at
  `/workspace/sortmaster_out`.
- Generated sign/label textures + Ozon logo: `/tmp/sortmaster_signs` (writable).

### Deploy code changes (local → server)
The authoritative code lives on the user's PC at
`C:\Ricardo\ozone-tech_yosolo_487\` (`cell\`, `isaac\`). Deploy with:
```bash
scp -P 40576 cell/params.py root@90.224.159.6:/root/sortmaster/cell/
scp -P 40576 isaac/<file>.py root@90.224.159.6:/root/sortmaster/isaac/
```
Never edit a shell script on the server while a running bash is executing it
(bash reads by byte offset and will execute whatever bytes land there).

### Run headless (the canonical invocation)
```bash
docker run --rm --name isaacrun --gpus all --network=host \
  --entrypoint /isaac-sim/python.sh \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /root/sortmaster:/workspace/sortmaster:ro \
  -v /root/sortmaster_out:/workspace/sortmaster_out \
  -v /tmp/sortmaster_signs:/tmp/sortmaster_signs \
  -v /root/.cache/ov/hub:/var/cache/hub \
  -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache \
  -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache \
  -v /root/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs \
  -v /root/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config \
  -v /root/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data \
  -v /root/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  /workspace/sortmaster/isaac/run_isaac.py --seed 42 \
  --out /workspace/sortmaster_out/run1 --perception rtx --drive surface
```
Kit boots ~30 s warm. A full 11‑item run is ~80 s sim / ~11 min wall with RTX
recording. Assemble MP4s with `ffmpeg` on the host; serve evidence over
`python3 -m http.server 8080 --directory /root/sortmaster_out` and the user's
`ssh -L 8080:localhost:8080` tunnel.

---

## 2. What the project is (context)

Ozon Tech Hackathon, **Track 3**: items arrive on a **1 m/s conveyor (A)**, are
**measured by sensors**, **classified** into three categories, and **physically
routed** to three destinations:

```
conveyor A → RTX sensors measure geometry → official B/C/D classification
           → physical executive → destination containment
           → fault detection & recovery → reproducible metrics & evidence
```

- **B** = MAIN SORTER infeed (items that pass and are not special).
- **C** = OVERSIZE / out‑of‑gauge (bad dimensions).
- **D** = REPACK (round‑in‑cross‑section items).

Simulation‑only entry is explicitly allowed and can reach the top readiness
level; the jury scores **realism, readiness, industrial plausibility,
measurability, and evidence**. The executive/manipulation block is the biggest
scoring section. Final documentation is in Russian‑friendly form; the README is
a scored artifact.

Two engines, one cell (both from the same `cell/params.py`):
- **MuJoCo 3** — deterministic validation twin (headless, runs on any jury box).
- **NVIDIA Isaac Sim 6.0.1 (PhysX 5 + RTX)** — high‑fidelity digital twin with
  real rendered sensors; this is where the videos and the hero physics live.

---

## 3. The official classification rules (implement EXACTLY, in this order)

Units: millimetres. Decision order — **dimension rule strictly beats shape
rule**:

1. **Undersize → C.** If **any** measured dimension `< 10 mm`. (Apply a
   legal‑metrology guard band: measurement within sensor uncertainty of 10 mm
   cannot take the permissive branch; certification floor = `10 mm + 2×ground
   sampling`.)
2. **Oversize → C.** If the sorted dimensions exceed `450 × 320 × 320 mm`
   (largest > 450, or middle > 320, or smallest > 320), with a guard band.
3. **Round → D.** If the cross‑section is circular: inscribed‑radius /
   circumscribed‑radius `r_in / R ≥ 0.8` (surface‑of‑revolution / circular
   footprint). Also the tall‑round‑dome and flank‑verticality helpers already
   in `isaac/perception_rtx.py::fuse_reads`.
4. **Otherwise → B.**

Safe‑side policy: sensor miss / unverifiable section on an elongated item /
low fused confidence → **D** (repack / manual review), never silently → B.

`cell/params.py` already holds `LIMIT_MIN_MM = 10`, `LIMIT_MAX_MM = (450,320,320)`,
`CIRCLE_RATIO = 0.8`, and the `CLASSIFICATION` policy knobs. **Reuse them.**

---

## 4. The official item set + the edge cases you MUST test

Ground truth (`cell/assets/manifest.json`, dims in metres, mass in kg):

| slug | zone | mass | dims (m) | why |
|---|---|---|---|---|
| bottle | D | 0.305 | 0.305×0.091×0.091 | round cross‑section |
| box_s | B | 3.0 | 0.301×0.201×0.200 | plain box, fits |
| box_l | C | 6.0 | 0.401×0.400×0.300 | oversize (400>320) |
| lunchbox | B | 0.412 | 0.201×0.152×0.062 | flat box, fits |
| sack | D | 0.817 | 0.202×0.176×0.170 | soft, ratio 0.886 |
| detergent | B | 2.002 | 0.280×0.260×0.179 | ratio 0.73 near‑borderline → B |
| pouf | C | 6.0 | 0.489×0.489×0.264 | round BUT oversize (489>450) → C (dim beats shape) |
| pen | C | 0.05 | 0.148×0.013×0.009 | 9 mm < 10 → undersize → C |
| plate | D | 0.175 | 0.209×0.209×0.026 | round, thin (26 mm) |
| cylinder | D | 0.187 | 0.435×0.050×0.043 | **hexagonal prism**, ratio cos30°=0.867, length 435<450 — designed double‑borderline trap |
| helmet | D | 4.323 | 0.352×0.298×0.282 | dome, rocks after transit |

**Edge cases to add as synthetic test items and PROVE by metrics** (build a
`manifest_edge.json` merged like the existing `manifest_extra.json`):

- **11 mm cube** (0.011×0.011×0.011): NOT undersize (all ≥10 mm), square
  footprint → **B**. **This is the headline reason for the rebuild** — the
  cross‑belt/tilt‑tray must carry and discharge it correctly to B.
- **10 mm‑exactly cube** and **9 mm rod**: boundary of the undersize rule (→ C).
- **445 mm and 460 mm boxes**: boundary of the oversize rule.
- **rounded‑square 0.78** (→ B) and **pentagon cos36°=0.809** (→ D): boundary of
  the circle rule.
- **very thin flat item** (a card/label ~2 mm thick): thin‑item measurement +
  handling on a tray.
- **soft sack** and **helmet** (post‑transit rocking / instability).
- **close spacing**, **off‑centre induction**, **low friction (×0.7)**, **heavy
  (×1.3)**, and **jam + recovery**.

For every edge case, log the measured dims, the fused verdict, the confidence,
and the final delivered zone. A safe conservative call‑out is acceptable and
scores well; a silent unsafe misroute is a hard fail.

---

## 5. What EXISTS today and what to KEEP vs REBUILD vs DELETE

Repo layout (`/root/sortmaster`, local `C:\Ricardo\ozone-tech_yosolo_487`):
```
cell/                     # single source of truth + MuJoCo twin
  params.py               # ALL constants (belts, cages, chutes, arm, sensor, rules)
  assets/ manifest.json   # 11 official STL items + ground truth
  run_sim.py belt.py table.py controller.py arm_ik.py scene.py ...  (MuJoCo)
isaac/
  run_isaac.py            # the closed loop (spawn, perception, executive, recovery, metrics)
  scene_usd.py            # USD stage from params (belts, deck, gates, chutes, cages, items, cameras)
  perception_rtx.py       # 3‑head RTX depth station + fuse_reads() classification  ← KEEP
  jam_locator.py          # background‑subtraction jam localization                  ← KEEP (adapt)
  arm.py                  # UR10e exception arm + recovery state machine             ← KEEP (adapt)
  materials.py dressing.py# OmniPBR grunge materials, lighting truss, industrial context ← KEEP
  cinematic.py            # cinematic + item‑follow cameras                          ← KEEP
  asset_shells.py         # official Isaac asset shells (conveyors, UR10e, sensors)  ← KEEP/extend
  arb_deck.py             # ARB patch actuator field                                 ← DELETE/replace
  showcase*.sh run_matrix.sh  # render + validation batch scripts                    ← ADAPT
docs/report/isaac_evidence/ # summaries, videos, matrix, calculations               ← KEEP framework
```

**KEEP (reuse as‑is):**
- `perception_rtx.py` + `fuse_reads` (RTX 3‑head depth station; 100 % on the
  official set, guard‑banded rule order). The item is measured **in motion**.
- `materials.py` (OmniPBR world‑triplanar grunge), `dressing.py` (ceiling‑truss
  RectLight high‑bays, wall control cabinet / HMI / cable trays / safety fence /
  light curtain / floor safety zones, Ozon branding), `cinematic.py` (orbit /
  dolly / crane / item‑follow), fixed exposure (`/rtx/post/histogram/enabled`
  = False).
- The metrics/evidence framework (`summary.json`, `events.csv`, validation
  matrix, `docs/report/calculations_vs_simulation.md`).
- The exception **arm** for jam recovery (UR10e, non‑colliding kinematic links,
  item carried at the controller TCP), **repositioned so it never crosses any
  structure** (it lives in an open corner and removes snags to a reject/review
  bin or drops route‑correct items into the correct cage — see §6.5).

**REBUILD (the executive):** replace the ARB patch deck + pop‑up flow blades +
vertical‑lift exit gates with a **cross‑belt or tilt‑tray sorter** (§6).

**DELETE:** `arb_deck.py`, the pop‑up blade bodies (`build_blade`), and the
gate‑as‑diverter mechanism. (You may keep a simple singulation escapement at
induction — see §6.2 — but not the accumulation blade field.)

---

## 6. The new executive — design spec (cross‑belt / tilt‑tray)

Pick **tilt‑tray** as the primary recommendation (simplest to make physically
real and unambiguous; gravity discharge is honest physics and size‑independent),
with **cross‑belt** as an equally acceptable alternative if you prefer powered
transverse discharge. Justify your choice in the README.

### 6.1 Topology (linear, compact — fits the 10 × 6 m work zone)
- Keep **conveyor A (1 m/s)** as infeed and the **RTX vision station** over A
  (unchanged — items are classified before induction).
- After the vision window, items are **singulated** and **inducted one‑per‑
  carrier** onto a **sorter train**: a line of independent carriers riding a
  powered track (a closed loop is ideal for realism; a straight run with a
  return is acceptable). Carrier pitch must exceed the largest item footprint
  (~0.5 m) — use ~0.6 m pitch.
- Three **discharge stations** along the train, one per destination, each above
  its chute → cage: **B**, **C**, **D**. Keep the existing **32° brake chutes +
  aperture‑walled roll cages** (they contain the item; reuse `build_chute` /
  `build_cage`). Discharge order along the train can be C, D, B or any fixed
  order.

### 6.2 Induction (singulation, physical)
- One escapement (a single low pop‑up stop or a metering gap) releases one item
  per carrier arrival. The **belt never stops**; the escapement only meters.
- The carrier must be **empty and aligned** at the induction point when the item
  transfers. Model the transfer as the item riding off belt A onto the moving
  carrier by contact (a short powered transfer/ski‑jump belt matched in speed),
  **not** a teleport.

### 6.3 Carrier mechanics (choose ONE, make it physical)

**A) TILT‑TRAY (recommended).** Each carrier = a shallow tray on a **revolute
joint** (tilt axis along the travel direction). Nominal: tray flat (item rides).
At the assigned discharge station, a PhysX **angular drive** tilts the tray
(≈ 35–40°) → the item **slides off by gravity** into that station's chute. After
discharge the tray returns to flat. Left/right tilt is not required for a linear
3‑chute layout on one side; if chutes are on both sides, tilt direction selects
the side. Real physics: tray = rigid body + revolute joint + angular drive;
item = rigid body on the tray; discharge = gravity on the tilted surface.
Size‑independent: an 11 mm cube slides off exactly like a big box.

**B) CROSS‑BELT.** Each carrier = a short **transverse belt cell**
(`PhysxSurfaceVelocityAPI`, axis perpendicular to travel). Nominal surface
velocity 0. At the assigned station, command the cell's transverse surface
velocity (with latency/ramp/saturation like a real drive) → the item is carried
sideways off the carrier into the chute by belt friction. Real physics:
surface‑velocity contact; **no direct item writes**. Note the PhysX gotcha:
`surfaceVelocity` is in the prim's LOCAL frame and is **scaled by the prim's
xform scale** — pre‑divide the commanded m/s by the carrier's half‑extents (a
cube of half‑length L commanded 1.0 drags at L m/s).

### 6.4 Actuator realism (either mechanism)
Model and LOG the actuator: command latency (~40 ms), motion ramp, saturation,
a little gain noise. Emit an `actuator_log.csv` and summary fields
(`carrier_commands_count`, `tilt_time_ms` or `cross_belt_speed_mps`,
`discharge_latency_ms`). The commanded discharge fires only when the assigned
carrier is at its station (position‑triggered), proving closed‑loop timing:
report a `command_margin_s` = (classification‑ready time) − (discharge time),
which must be comfortably positive.

### 6.5 Exception arm (keep, but never cross structure)
- The arm is the **exception handler**, not the main diverter. It sits in an
  **open corner outside the sorter/chute footprint** and only acts on a jam
  (watchdog fires). Verify by geometry that no arm link path intersects any
  chute/cage/tray structure at any commanded pose (idle fold + full recovery
  trajectory). IK‑reach‑check every waypoint against the UR10e limit
  (L1+L2 ≈ 1.184 m at the shoulder) at a **reachable lift height** (do not use a
  transfer height whose wrist target exceeds the reach ceiling).
- Recovery policy (route‑specific, keeps category correctness AND keeps the arm
  clear of structure): a **C** snag → place into cage C; a **D** snag → place
  into cage D; a **B** snag → **reject / manual‑review bin** (never re‑feed an
  uncertain item to the main sorter). Pin the jammed item's **pose** (not just
  velocity) so it does not creep away before the arm grasps; keep the pin until
  the arm actually attaches; use the tracked odometry pose (camera confirms the
  jam, odometry gives the precise, reach‑checked grasp point).

### 6.6 What this buys you (put in the README)
- **Size‑independent divert:** every item, from an **11 mm cube** to a 489 mm
  pouf, is carried on its own tray/cell and discharged — no roller/ARB
  footprint floor, no gap to fall through, no pop‑up wall.
- Matches **real Ozon‑scale** small‑parcel sortation (cross‑belt / tilt‑tray).
- Fully physical: contact transport, gravity/belt discharge, actuated joints,
  `direct_velocity_writes_nominal = 0`.

---

## 7. Hard physics/engine gotchas (learned the hard way — obey these)

1. `PhysxSurfaceVelocityAPI.surfaceVelocity` is **local‑frame, scaled by xform
   scale** → pre‑divide commanded speed by the prim's half‑extents. Verify true
   speeds with a `--probe` trace.
2. `World(physics_dt = rendering_dt)` + **manual `world.render()`** for frame
   cadence; a larger rendering_dt silently advances multiple substeps per step.
3. RTX annotators serve **stale frames**: after blind stepping, call
   `world.render()` ~6–7× before reading depth/RGB, or you classify the previous
   item. Verify frame mtime vs run start.
4. `Camera.get_world_pose()` uses world‑axes convention — for pinhole
   back‑projection use the **USD prim transform**, not the camera helper.
5. Referenced official assets **own their xformOps** (reuse translate/rotate/
   scale, never AddTranslateOp) **and ship their own lights** — sanitize every
   referenced subtree: disable CollisionAPI/RigidBodyAPI, un‑instance prototypes
   first, and **set all shipped `*Light` intensities to 0 / make invisible** (a
   shipped light in the UR10e/gripper/RealSense assets blew out the scene).
6. PhysX skips **sleeping** bodies — set `sleepThreshold = 0` on items via the
   live view or they freeze after a brief stop.
7. `Gf.*` constructors reject numpy scalars — cast to python float; build vertex
   arrays with `Vt.Vec3fArray.FromNumpy`.
8. **Fixed exposure:** set `/rtx/post/histogram/enabled = False` (carb settings)
   or hundreds of mixed‑camera renders auto‑brighten and clip correctly‑dark
   surfaces to cream.
9. Big flat backdrops must be **plain matte** (no tiled detail texture — it
   reads as wallpaper); use low‑contrast, large‑world‑scale OmniPBR grunge only
   on machinery.
10. Only **one Kit instance per GPU**; kill leftover containers/systemd units
    before every launch; a second instance halves FPS and corrupts timing.
11. Deploy‑then‑run: scp local → `/root/sortmaster`; the container mounts it
    read‑only, so a running job keeps its bytes and only the *next* job picks up
    new code.

---

## 8. Validation & evidence (this is scored — do it fully)

Produce, on the RTX 5090, a **validation matrix** (adapt `run_matrix.sh`):
```
seed42_nominal_all_items            six_seed_nominal_all_items (42,1,2,3,7,99)
edge_items_all (incl. 11mm cube)    low_friction (×0.7)   high_mass (×1.3)
close_spacing                       off_center            jam_recovery
gate/discharge_fault (a stuck tray or dead cross‑belt cell → safe call‑out)
```
Consolidate to one `matrix_summary.json` (adapt
`tools/consolidate_isaac_matrix.py`). **Target metrics:**
```
classification_accuracy >= 0.98        routing_accuracy >= 0.97
unsafe_errors = 0                       containment_rate = 1.0
deadlocks = 0                           direct_velocity_writes_nominal = 0
cycle_s mean/p95/max recorded           throughput_items_per_h recorded
carrier/actuator commands logged        command_margin_s > 0 for every item
11mm cube routed to B (proven)          every §4 edge case logged
```
Separate **recovery_success** from **routing_accuracy** (a B jam safely diverted
to review is a *successful safe recovery*, not a correct B delivery). Make the
run process **exit non‑zero** on any floor drop, containment escape, unexpected
manual outcome, or failed recovery — success must mean success.

Update `docs/report/calculations_vs_simulation.md` with the tilt/cross‑belt
first‑principles numbers (tray tilt angle vs item friction for guaranteed slide;
carrier pitch vs throughput; induction timing; discharge time vs command
margin; cage entry speed) each cross‑checked against the measured matrix.

---

## 9. Cameras & final videos (reuse `cinematic.py`, keep technical visibility)

Render on the 5090 (clean nominal = a seed with 0 jams), no wall/arm occlusion,
no fast motion, no heavy blur:
- 4 static angles: wide industrial hero (open SW corner), full‑cell isometric,
  discharge/tilt close‑up, top‑down of the sorter train.
- 3 cinematic paths: slow orbit, flow‑follow dolly, crane reveal.
- `item_follow_perception_to_bin.mp4`: camera follows ONE item from the vision
  station, along the sorter, to its cage — smooth, framed, unobstructed.
- Perception demo: side‑by‑side RGB | grayscale depth with the item segmented,
  measured dims, verdict chip (B/C/D), confidence, rule reason
  (`tools/make_perception_panels.py`).
- Fault/recovery: a jam → watchdog → jam camera → arm removes to reject / places
  to the correct cage.
- Final cinematic + a metrics end‑card (`tools/make_endcard.py`).

---

## 10. Deliverables

1. New executive in `isaac/` (e.g. `isaac/sorter.py` for the tilt‑tray/cross‑belt
   controller, `scene_usd.py` building the carrier train + discharge stations),
   with `arb_deck.py` and the pop‑up blades removed.
2. MuJoCo twin (`cell/`) updated to the same executive for parity (the twin may
   simplify carrier dynamics but must produce the same B/C/D outcomes and the
   same metric vocabulary).
3. `manifest_edge.json` with the 11 mm cube and the §4 boundary items, merged by
   the scene loader.
4. The full validation matrix + `matrix_summary.json` meeting §8 targets.
5. The video set + perception panels + end‑card in
   `docs/report/isaac_evidence/`.
6. Updated `isaac/README.md`, main `README.md`, and
   `calculations_vs_simulation.md` explaining: *RTX perception classifies →
   route command → **cross‑belt/tilt‑tray physically carries each item on its
   own cell and discharges it** → chute → cage; size‑independent (proves 11 mm);
   no scripted motion; measured metrics.* Include honest limitations.
7. Local test suite green (`.venv/Scripts/python -m pytest tests/ -q`) — update
   tests that assumed the ARB deck / blades; add a test that the 11 mm cube
   classifies B and is delivered B.

---

## 11. Acceptance criteria (definition of done)

- A clean 11‑item nominal run: **11/11 classified, 11/11 routed, 0 unsafe,
  containment 1.0, 0 direct velocity writes**, ~real‑time.
- The **11 mm cube** (and the §4 boundary items) route correctly with logged
  measurements — the headline proof that the size limitation is solved.
- The validation matrix meets §8 targets; the run exits non‑zero on any bad
  outcome.
- Videos show the tilt‑tray/cross‑belt carrying and discharging items with the
  arm parked clear and never crossing structure.
- Docs explain the mechanism honestly and cross‑check calculations vs measured.

---

## 12. Recommended work order

1. **Baseline & smoke.** Connect, run the current build once, confirm the GPU +
   perception + evidence pipeline work end‑to‑end.
2. **Design freeze.** Choose tilt‑tray (recommended) vs cross‑belt; add the
   carrier‑train + discharge‑station params to `cell/params.py`.
3. **Build the carriers in USD** (`scene_usd.py`): powered track, N carriers at
   0.6 m pitch, tray revolute joints (or transverse belt cells), 3 discharge
   stations over the existing chutes/cages. Delete the ARB deck + blades.
4. **Induction + singulation:** physical transfer from belt A to a moving empty
   carrier; one item per carrier.
5. **Discharge controller** (`isaac/sorter.py`): position‑triggered tilt/cross‑
   belt with latency/ramp/saturation; per‑carrier route assignment from the
   fused verdict; actuator logging. No direct item writes.
6. **Reconcile perception + routing:** the item is classified over A (unchanged)
   and its carrier is tagged; the discharge fires at the assigned station.
7. **Arm recovery** adapted to the new layout (reach‑checked, structure‑clear,
   route‑specific placement / reject bin, pose‑pinned jam).
8. **Edge‑case manifest** (11 mm cube etc.) + prove routing with metrics.
9. **Validation matrix** on the 5090 + consolidate + gate on §8 targets +
   non‑zero exit on bad outcomes.
10. **Materials/lighting/context/cameras** already exist — re‑fit to the new
    geometry; render the video set + perception panels + end‑card.
11. **Docs + tests** update; commit. Do not `git push` without the user's
    explicit authorization; end commit messages with a `Co-Authored-By:` line.

---

## 13. Tone for the jury

Honest, measurable, industrial. Say what is physical (everything nominal) and
what is a modelled simplification (the arm's non‑colliding kinematic links; the
soft sack as a rigid approximation). A safe conservative call‑out beats a fake
perfect demo. The winning story: **a validated, measurable, physically‑real
industrial sorter that carries every item — including sub‑centimetre parcels —
on its own cell, classifies with a real RTX sensor, and routes by contact
physics with zero scripted motion.**
```
```
END OF MASTER PROMPT
```
