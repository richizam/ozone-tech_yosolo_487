# `isaac/` — SortMaster cell in NVIDIA Isaac Sim (PhysX 5 + RTX)

The **same** sorting cell as the MuJoCo validation engine, built from the one
source of truth (`cell/params.py`) and taken further: real rendered sensors in
the loop and a physically-driven conveyor executive. Runs headless on a GPU
server; produces the jury-facing RTX video, depth-camera captures, and a
metrics `summary.json` in the same vocabulary as the MuJoCo runs. Results and
the cross-engine argument:
[docs/report/isaac_evidence/](../docs/report/isaac_evidence/README.md).

**Reference result (seed 42, full official set):** 11/11 delivered · 11/11
classified by the RTX depth station · 11/11 routed end-to-end · 0 unsafe ·
containment 1.0 (0 violations, cage entry ≤ 1.73 m/s) · ~1.2× real time.

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
  items are carried by friction. The routing zone is a switchable-vector
  **ARB sorter** aimed at the active item's exit; flow discipline is enforced
  by physical **pop-up stop blades** (escapement, pre-gate hold, two
  zone-accumulation stops, table induction) with a raise-safety interlock;
  **powered nose-overs** guide discharge onto the 32° brake chutes. No
  per-item velocity writes anywhere in the nominal flow.
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
| `run_isaac.py` | The closed loop: spawning, flow discipline, RTX multi-read classification, ARB zone command, watchdog + camera-fix arm recovery, containment tracking. Writes `summary.json` + `events.csv`, optional MP4 frames + vision stills. |
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

# regression baselines
#   --perception oracle   ground truth + latency (debug)
#   --drive scripted      legacy per-item velocity drive
```

Cameras for `--record`: `overview | top_view | routing | lookahead`.

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
- `frames/*.png` → `demo_overview.mp4`; `vision_rgb_*.png`,
  `vision_depth_*.{png,npy}`.

## Requirements

Nothing to install — the official `nvcr.io/nvidia/isaac-sim:6.0.1` container
ships Python 3.12, USD/PhysX and PIL. The repo inputs are pure Python
(`cell/params.py`, `cell/assets/manifest.json`) and the 11 official STL meshes.

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
