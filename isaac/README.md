# `isaac/` — SortMaster cell in NVIDIA Isaac Sim (PhysX 5 + RTX)

The **same** sorting cell as the MuJoCo validation engine, built from the one
source of truth (`cell/params.py`) and taken further: real rendered sensors in
the loop and a physically-driven conveyor executive. Runs headless on a GPU
server; produces the jury-facing RTX video, depth-camera captures, and a
metrics `summary.json` in the same vocabulary as the MuJoCo runs. Results and
the cross-engine argument:
[docs/report/isaac_evidence/](../docs/report/isaac_evidence/README.md).

**Reference result (ARB-deck build, 12-run matrix / 132 item trials):**
nominal classification **66/66 = 100%** from the RTX depth station in
motion · nominal routing **65/66 = 98.5%** (the exception: a 9 mm pen
micro-stall answered by a safe operator call-out — never a wrong feed) ·
low/high-friction, high-mass, close-spacing, off-center and jam-drill
sweeps all **11/11** · **0 unsafe errors and containment 1.0 in every
run** · 6,064 logged actuator commands, **0 direct velocity writes** ·
0.71–0.86× real time with the sensors in the loop. Consolidated:
[docs/report/isaac_evidence/validation_arb/matrix_summary.json](../docs/report/isaac_evidence/validation_arb/matrix_summary.json).

## Architecture (what makes this an *Isaac* solution)

- **Perception is a real sensor.** A 3-head RTX depth station (overhead
  1024×768 + two side profiler heads at ±0.45 m — the `VIRTUAL_SENSOR`
  geometry from `cell/params.py`) renders true depth; the item is measured in
  motion, multi-read fused with legal-metrology guard bands, and classified by
  the official rule order (dims gate first, then circle-in-section at
  r_in/R ≥ 0.8). Calibrated **33/33 = 100%** on the official set across three
  rest yaws (`validate_rtx.py`); 11/11 in the closed-loop run. Safe-side
  policies throughout: sensor miss → manual lane, unverifiable section on an
  elongated item → D, guard-banded limits never take the permissive branch.
- **The executive is driven by contact physics, not scripts.** Belts, the
  table entry strip and the B-connector are kinematic conveyors with
  `PhysxSurfaceVelocityAPI` (the Isaac Conveyor-Belt-utility mechanism):
  items are carried by friction, at the **designed speeds** (belt A at the
  official 1.0 m/s, table at 0.8 m/s — surface-velocity commands are
  normalized for PhysX's local-frame × scale semantics and probe-verified).
  The routing zone is an **ARB actuator deck**: a 4×7 matrix of local
  150×157 mm surface-velocity patches (`isaac/arb_deck.py`,
  `cell/params.ARB_DECK`), each an independent actuator with a **40 ms
  command pipeline, a 6 m/s² velocity ramp, 1.2 m/s saturation and
  per-command gain noise** — only the patches under the routed item (plus a
  0.10 m pre-spin halo) receive the divert command; the rest keep feeding
  forward. Every command is logged (`actuator_log.csv`) and summarized
  (`actuator_commands_count`, `actuator_latency_ms`,
  `max_surface_speed_mps`).

  **Mechanical embodiment (honest model statement):** the deck renders as
  the **official `ConveyorBelt_A49` right-angle transfer module** (found by
  sweeping the full A42–A49 tail of the 6.0 conveyor set; silver carry
  rollers + interleaved transfer wheel packs — the real industrial
  mechanism class), fitted as a sanitized shell with its deck top exactly
  at the ride plane, plus a 4×7 **module-status LED matrix** on the south
  face tinted per actuator state. The physics is and stays **patch-level
  ARB actuation**; roller-by-roller bearing/contact simulation is
  intentionally not used — neither required by the evidence nor validated.
  (A procedural angled-roller module builder is kept in
  `asset_shells.py` as the offline fallback; the A49 shell won the
  side-by-side render comparison.) Exit gates carry industrial hardware:
  anodized panels with route-color stripes, guided lift rams sliding past
  fixed actuator bodies, and photoeye brackets at freight height. Flow discipline is enforced by physical
  **pop-up stop blades** (escapement, pre-gate hold, two zone-accumulation
  stops, table induction) with a raise-safety interlock; **powered
  nose-overs** guide discharge onto the 32° brake chutes. No per-item
  velocity writes anywhere in the nominal flow — `summary.json` proves it:
  `"nominal_motion_model": "surface_contact_only"`,
  `"direct_velocity_writes_nominal": 0`.
- **Items have material-class physics.** `cell/params.MATERIALS` assigns
  each slug friction/restitution/damping by material (cardboard, PET, HDPE,
  ABS shell, ceramic, soft sack/pouf with damping); the chute pairs friction
  by `min` (guaranteed slide), the brake pad by `max` (guaranteed braking
  even for slippery items). Robustness sweeps scale the whole table:
  `--friction-mult 0.7/1.3`, `--mass-mult 1.3`.
- **Faults are handled on sensor data.** A routing-zone depth camera holds a
  background model of the empty cell; when the zero-displacement watchdog
  fires, background subtraction localizes the stuck item (best case ~20–30 mm
  error), known hardware (blades, open gate panels, cage interiors) is
  excluded, and the **4-axis exception arm** (closed-form IK from
  `cell/arm_ik.py`, the `cell/controller.py` cycle) picks at the **camera
  fix** and re-delivers onto the item's lane; unreachable/repeat failures
  escalate to an operator call-out (MANUAL).

## What's here

| File | Role |
|---|---|
| `scene_usd.py` | USD stage from `cell/params.py`: surface-velocity conveyors, pop-up blades (prismatic drives), normally-closed exit gates, 32° chutes + nose-overs, aperture-walled cages, convex-hull items from the official STLs, all cameras. No importers — binary STLs parse straight into `UsdGeom.Mesh`. |
| `run_isaac.py` | The closed loop: spawning, flow discipline, RTX multi-read classification, ARB deck command, watchdog + camera-fix arm recovery, containment tracking. Writes `summary.json` + `events.csv` + `actuator_log.csv`, optional MP4 frames + vision stills. |
| `arb_deck.py` | The ARB actuator deck controller: per-patch command pipeline (latency), velocity ramp, saturation, per-command gain noise, state machine (`forward`/`divert:B|C|D`), per-patch visual activation, actuator command log + metrics. |
| `run_matrix.sh` | Host-side validation matrix: 6-seed nominal + friction/mass/spacing/off-center/jam sweeps → per-run `summary.json` (consolidate with `tools/consolidate_isaac_matrix.py`). |
| `perception_rtx.py` | Depth → world cloud → identity-gated segmentation → dims (percentile extents), footprint circularity, mirrored-hull section r_in/R, dome score, side-head flank verticality → official rule order; `fuse_reads()` = guard-banded multi-read fusion. |
| `jam_locator.py` | Background-subtraction jam localization on the routing-zone depth camera (grid clustering, hardware exclusions, route-corridor association). |
| `arm.py` | Kinematic 4-axis palletizer (MuJoCo parity: collision-free links, item carried at the TCP) + the recovery state machine. |
| `validate_rtx.py` | Static calibration: 11 items × N yaws under the real cameras vs ground truth → `rtx_validation.json`. |
| `dressing.py` | Industrial presentation + route-storytelling layer (visuals only, physics/sensor-safe): dark rubber belts + end drums + skirts + hazard-striped edges + support legs + white direction chevrons, roller deck + omni-puck field, labeled roll cages («C OVERSIZE»/«D REPACK», tubes, casters), Intel RealSense D455 sensor assets (official Isaac models, hand-made housings as offline fallback), Ozon design language (wall wordmark, blue/magenta accents), warehouse lighting. Runtime route storytelling (`RouteVizRuntime`): items tint with their category, carry a floating «>> B SORTER / C OVERSIZE / D REPACK» flag, the ACTIVE ROUTE lamp panel + routing-deck arrows + chevron trails to each container brighten for the commanded route. Verified bit-identical physics with the layer on (same sim time, same 11/11). |

## Run it (inside the isaac-sim container)

```bash
# full official set, RTX perception + surface-conveyor executive (defaults)
/isaac-sim/python.sh /tmp/sortmaster/isaac/run_isaac.py \
    --seed 42 --out /tmp/sortmaster_out/final_seed42

# with the RTX video + vision stills (the demo run)
/isaac-sim/python.sh /tmp/sortmaster/isaac/run_isaac.py \
    --seed 42 --out /tmp/sortmaster_out/final_seed42 --record --camera overview

# perception calibration (static, 3 yaws)
/isaac-sim/python.sh /tmp/sortmaster/isaac/validate_rtx.py \
    --out /tmp/sortmaster_out/rtxval --yaws 0,35,120

# fault drill: inject a snag so the watchdog + jam camera + arm fire
/isaac-sim/python.sh /tmp/sortmaster/isaac/run_isaac.py \
    --inject-jam box_s@8.05 --out /tmp/sortmaster_out/jam_drill

# robustness / fault-case runs (SUPER_REALISTIC plan P3/P6)
#   --friction-mult 0.7     low-friction material sweep
#   --mass-mult 1.3         heavy-item sweep
#   --spawn-offset-y 0.10   off-center entry
#   --spawn-gap 3.5,4.5     close spacing
#   --probe all             1 Hz trace of every item + flow state (diagnosis)

# regression baselines
#   --perception oracle   ground truth + latency (debug)
#   --drive scripted      legacy per-item velocity drive
```

Cameras for `--record`: `overview | top_view | routing | lookahead`.

Branding note: `dressing.py` composites the Ozon wordmark from
`isaac/assets/ozon_logo.png` — copy it to `/tmp/sortmaster_signs/ozon_logo.png`
on the sim host (that directory is also the dressing's generated-texture
working dir; mount it read-write into the container).

### Full validation matrix (host-side)

```bash
scp -P <port> isaac/run_matrix.sh root@<host>:/root/ && ssh ... \
    "bash /root/run_matrix.sh /root/sortmaster_out/final_arb"
# 6-seed nominal + low/high friction + high mass + close spacing +
# off-center + jam drill; then consolidate:
python tools/consolidate_isaac_matrix.py <matrix_dir>
```

### From the host: assemble + serve the evidence

```bash
docker exec isaac-sim /isaac-sim/python.sh /tmp/sortmaster/isaac/run_isaac.py ...
bash /root/make_evidence.sh final_seed42        # frames -> MP4 + index.html
# systemd-run --unit=sortmaster-http python3 -m http.server 8080 --directory /root/evidence
# view over the SSH tunnel (ssh -L 8080:localhost:8080 ...):
#   http://localhost:8080/final_seed42/
```

## Outputs

- `summary.json` — classification accuracy (n, correct, sensor misses,
  reads/item), routing accuracy, unsafe errors, containment (rate, violations,
  cage-entry speed, bounce height), cycle stats, throughput, flow-discipline
  counters, per-item margins.
- `events.csv` — `item_spawned/detected/classified` (with fused features and
  reasons), `routing_cmd`, `table_entry` (command margin), `item_delivered`,
  `jam_detected`, `jam_located` (camera fix + error vs truth),
  `recovery_started/attached/released/job_done`, `containment_violation`.
- `frames/*.png` → MP4 (`/root/make_mp4.sh`); `vision_rgb_*.png`,
  `vision_depth_*.{png,npy}` → jury-facing side-by-side perception panels
  via `tools/make_perception_panels.py` (RGB | sensor's-eye depth with the
  item segmented, measured dims, verdict chip, confidence, rule line).
- Video set v2 (`isaac/showcase2.sh`, evidence in
  `docs/report/isaac_evidence/final_arb/`): clean nominal runs FIRST
  (full-cell overview with no arm intervention + per-route B/C/D deck
  close-ups + sensor-station pass), then the fault program (jam → UR10e
  recovery; gate stuck-closed → timeout → safe call-out), then the
  cinematic metrics end-card (`tools/make_endcard.py`).

## Requirements

Nothing to install — the official `nvcr.io/nvidia/isaac-sim:6.0.1` container
ships Python 3.12, USD/PhysX and PIL. The repo inputs are pure Python
(`cell/params.py`, `cell/assets/manifest.json`) and the 11 official STL meshes.

## Conveyor design choice (defense note)

The official test set includes a **9 mm pen**, so every item-contact surface
in the cell is **continuous**: belt conveyors (official A05 asset) for
infeed/vision/entry and an **Intralox-class Activated Roller Belt** for the
routing zone — a continuous belt with small steering rollers embedded flush
in its surface. This is deliberately NOT an open roller bed: small items
cannot fall between or jam under anything, which is exactly what the physics
simulates (vectored surface velocity on a continuous surface) and exactly how
real mixed-parcel ARB sorters are built. Roller drive hardware remains
visible where it belongs — end drums, side frames, and the flush ARB caps.

## Collision proxies (visual mesh ≠ collision mesh)

Every item renders its **true official STL** (this is what the RTX depth
station measures — the sensor sees real geometry, including the cylinder's
rounded end caps and the helmet dome), while **contact** uses a documented
proxy: a PhysX convex hull of that surface (≤64 vertices, mirroring the
MuJoCo twin's `maxhullvert=64`), with manifest-true mass and the material
table above. The proxy choice is deliberate: hulls are the robust,
jury-reproducible baseline, and the perception/physics split means proxy
simplification can never leak into classification. Known limitation and
upgrade path (documented, not hidden): concave items (helmet interior,
plate rim) contact as their hulls; per-category compounds (capsule stacks
for bottles, convex decomposition for the helmet) are the next fidelity
step and slot into `build_item()` without touching the flow logic.

## Design parity & documented deltas

The flow logic mirrors `cell/belt.py`, `cell/table.py` and `cell/run_sim.py`;
the drive mechanism is *more* physical than the MuJoCo twin (surface-velocity
conveyors and real stop blades instead of velocity writes). Engine-specific
tolerances are localized to this package (canonical `cell/params.py`
untouched): tight contact offsets, a hood lift over the chute apertures, and
powered nose-overs at the crest handoffs — PhysX resolves the discharge
dynamics differently from MuJoCo (free tipping is snappier), and each delta is
validated by the containment gates (violations must be 0, `cage_max_z ≤ 0.56 m`
≪ aperture top 0.83 m). Full rationale in
[docs/report/isaac_evidence/README.md](../docs/report/isaac_evidence/README.md).

**Strict-realism audit (2026-07-09, all found by scene forensics, physics
bit-identical throughout):** (1) the A05 conveyor shells' interior
`Rollers`/`Rubberbands` sub-meshes sat visibly UNDER the belt band ("item
rides a flat belt over stray rollers") — hidden, sides closed with steel
skirting; (2) the single stretched belt-A tile pushed the asset's tail
drive drums 3.4 m past the belt end into the cage-C volume — every shell
segment is now bbox-fitted to its exact span; (3) the routing deck is a
procedural 45° steerable-wheel sorter top (8×14 wheels on shafts with
bearing brackets, crowns exactly at the ride plane) — no official asset
ships a lateral ARB mechanism (full A01–A49 sweep documented); (4) the
"blown-out C bin / spotlight on the arm" was root-caused by pixel
forensics to **light prims shipped inside the referenced UR10e / gripper /
RealSense assets** — `sanitize()` now disables lights in every referenced
shell; the powered discharge belts were also recolored from pale steel to
dark rubber, the classification tint desaturated, and all statics carry
matte industrial PBR materials with per-prim value variation instead of
glossy displayColor plastic; (5) lighting was rebuilt as a **ceiling truss
carrying downward-facing RectLight area high-bays** (`Dressing._truss` +
`_highbay`): visible steel roof girders + roof deck over the whole cell,
four soft area lights illuminating the floor evenly — no local point light
near the arm, no hot specular spots. Two fixture bugs were caught (a
RectLight emits from its −Z face, so an earlier RotateX(180) lit the roof
not the floor; the visible reflector panel was moved above the emitter so
it stops occluding the beam), and exposure is pinned
(`/rtx/post/histogram/enabled=False`) so hundreds of mixed-camera renders
don't auto-brighten into a clipped bin. Before/after evidence:
[docs/report/isaac_evidence/final_arb/before_after_cbin_lighting.png](../docs/report/isaac_evidence/final_arb/before_after_cbin_lighting.png).

**PhysX gotcha worth knowing (found by probing, fixed 2026-07-08):**
`PhysxSurfaceVelocityAPI.surfaceVelocity` is applied in the prim's *local
frame scaled by its xform scale*. A cube-based belt with half-length 3.45
commanded `(1,0,0)` dragged freight at exactly 3.450 m/s. All surface-velocity
commands are therefore pre-divided by the prim's half-extents
(`make_conveyor`, `ArbDeck.step`), and the true speeds are probe-verified
(`--probe all`): belt A 1.000 m/s (official spec), table/deck 0.800 m/s.
Timing-sensitive results from before the fix are superseded by the current
validation matrix.

## Engineering calculations cross-check

Every design number (time on deck, lateral displacement, friction budget,
chute entry speed, cycle time, actuator response vs command margin) is
derived from first principles and cross-checked against the measured matrix:
[docs/report/calculations_vs_simulation.md](../docs/report/calculations_vs_simulation.md).
Example: calculated cage entry ≈ 2.09 m/s at μ 0.40; measured 2.081 m/s
(bottle, seed 42).
