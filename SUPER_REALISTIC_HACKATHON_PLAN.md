# SortMaster: Super-Realistic Isaac Sim Plan for Winning the Hackathon

This document is a development plan, not a defense script. Its purpose is to define what must be built next so SortMaster becomes a physically convincing, measurable, and reproducible industrial simulation for the Ozon Tech hackathon.

## Goal

Build a credible industrial demonstration of an intelligent robotic sorting cell:

```text
product on conveyor A
-> RTX sensors measure geometry
-> official B/C/D classification
-> visible physical actuation
-> realistic movement into B/C/D
-> destination containment
-> fault detection and recovery
-> reproducible metrics and evidence
```

The classification side is already strong. The biggest score gain now is the executive mechanism: the item diversion must look and behave like a real industrial actuator system, not like a scripted animation.

## Current State

### Strong Points

- Isaac Sim 6.0.1 / PhysX 5 is running on the rented RTX 5090 instance.
- A 5090 smoke test passed:
  - `perception=rtx`
  - `drive=surface`
  - `delivered=1/1`
  - `routing_accuracy=1.0`
  - `unsafe_errors=0`
  - `realtime_factor=1.12`
- The repo is copied to the 5090:
  - `/root/sortmaster`
  - `/root/sortmaster_out`
- Isaac Sim container is running:
  - `isaac-sim`
- The project already has:
  - official B/C/D rules;
  - official STL item assets;
  - Isaac scene;
  - RTX depth sensors;
  - conveyors using `PhysxSurfaceVelocityAPI`;
  - chutes/cages;
  - watchdog and jam recovery logic;
  - previous MuJoCo/Isaac validation and evidence.

### Weak Points for Winning

The weak point is not that the project lacks a demo. The weak point is that the transfer-table actuation may still look too abstract:

- the ARB/routing zone works, but it is too global;
- there is no local actuator deck like a real matrix of rollers/patches;
- actuator delays, acceleration ramps, dead zones, and saturation are not explicit enough;
- material/contact parameters need to be more realistic;
- actuator failure cases need stronger coverage;
- final visual evidence needs industrial close-ups;
- calculations must be cross-checked against Isaac results to support high readiness.

## Connection and Runtime

### Connect to the RTX 5090 Instance

Use this command from your local terminal:

```bash
ssh -p 40576 root@90.224.159.6 -L 8080:localhost:8080
```

The `-L 8080:localhost:8080` tunnel is kept because Isaac/Omniverse tooling or auxiliary dashboards may expose local services through port 8080.

### Check Instance Status

Run on the remote instance:

```bash
nvidia-smi
docker ps -a
df -h /
du -sh /root/sortmaster /root/sortmaster_out /root/docker 2>/dev/null
```

Expected state:

```text
GPU: NVIDIA GeForce RTX 5090, ~32 GB VRAM
Docker image: nvcr.io/nvidia/isaac-sim:6.0.1
Repo: /root/sortmaster
Outputs: /root/sortmaster_out
Container: isaac-sim
```

### Start the Isaac Streaming Container

If the container is not running:

```bash
docker start isaac-sim
```

If it must be recreated:

```bash
docker rm -f isaac-sim 2>/dev/null || true

docker run -d --name isaac-sim \
  --gpus all \
  --network=host \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ISAACSIM_SIGNAL_PORT=49100 \
  -e ISAACSIM_STREAM_PORT=47998 \
  -v /root/sortmaster:/workspace/sortmaster:ro \
  -v /root/sortmaster_out:/workspace/sortmaster_out \
  -v /root/.cache/ov/hub:/var/cache/hub \
  -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache \
  -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache \
  -v /root/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs \
  -v /root/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config \
  -v /root/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data \
  -v /root/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg \
  nvcr.io/nvidia/isaac-sim:6.0.1
```

### Run a Short Smoke Test

Use this to verify that Isaac, RTX perception, PhysX, assets, and output writing still work:

```bash
docker run --rm --gpus all --network=host \
  --entrypoint /isaac-sim/python.sh \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /root/sortmaster:/workspace/sortmaster:ro \
  -v /root/sortmaster_out:/workspace/sortmaster_out \
  -v /root/.cache/ov/hub:/var/cache/hub \
  -v /root/docker/isaac-sim/cache/main:/isaac-sim/.cache \
  -v /root/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache \
  -v /root/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs \
  -v /root/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config \
  -v /root/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data \
  -v /root/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg \
  nvcr.io/nvidia/isaac-sim:6.0.1 \
  /workspace/sortmaster/isaac/run_isaac.py \
  --seed 42 \
  --items box_s \
  --out /workspace/sortmaster_out/smoke5090 \
  --depth-stills 0 \
  --max-sim-s 100 \
  --perception rtx \
  --drive surface
```

Expected result:

```text
delivered=1/1
routing_accuracy=1.0
unsafe_errors=0
realtime_factor recorded
summary.json written
events.csv written
```

## Design Principle

Do not make the scene "prettier" first. Make the motion physically explainable.

Rule:

```text
If the item moves sideways, there must be a surface, roller, belt, pusher,
gate, blade, or guide that visibly and physically explains that movement.
```

Avoid:

```text
item.velocity = route_vector
teleport / snap
one magic routing zone deciding the whole motion
perfect motion with no latency, no actuator limits, and no recoverable errors
```

Prefer:

```text
discrete actuators
local surface velocities
material-dependent friction
contact geometry
physical guides
gates with travel time
recoverable faults
```

## Priority 1: Build a Physical ARB Deck with Local Actuators

### Problem

The current system has good foundations with `PhysxSurfaceVelocityAPI`, but a winning-level simulation needs the table to look like a real industrial actuator deck, not a single intelligent surface.

### Target

Replace or complement the global ARB/routing zone with a matrix of local actuators:

```text
ARB deck = N x M patches/rollers
each patch has its own surface velocity
each patch has state: idle, forward, divert_left, divert_right, brake
each patch has latency, ramp-up, saturation, and small noise
```

### Implementation Direction

Create something equivalent to this in Isaac:

```text
transfer table:
  cells:
    row 0: patch_00 patch_01 patch_02 ...
    row 1: patch_10 patch_11 patch_12 ...
    row 2: patch_20 patch_21 patch_22 ...
```

Each patch should:

- be a separate prim;
- use `PhysxSurfaceVelocityAPI`;
- have realistic dimensions, for example 120-180 mm;
- apply local velocity based on route state;
- change state only when the item is near or on top of it;
- use velocity ramps instead of instant vector changes.

### Acceptance Criteria

A close-up video must show:

- the item entering the deck;
- only selected patches activating;
- the item diverting through contact;
- the item reaching B/C/D;
- the deck returning to idle before the next item.

Minimum metrics:

```text
actuator_commands_count > 0
actuator_latency_ms recorded
max_surface_speed_mps recorded
no direct velocity writes during nominal routing
```

## Priority 2: Remove Scripted Motion from Nominal Flow

### Target

The final evidence run must use:

```text
--drive surface
```

Nominal movement must come from:

- conveyors;
- local surface velocities;
- gravity;
- chutes;
- collision/contact;
- gates/blades;
- guides.

### Allowed Direct Writes

Direct position/velocity writes are acceptable only for:

- initial spawn;
- reset;
- removal of completed items;
- arm carry during recovery;
- debug mode `--drive scripted`.

### Not Allowed in Final Evidence

Do not use `set_v` to command normal item diversion over the table.

### Acceptance Criteria

Add a metric like this to `summary.json`:

```json
"nominal_motion_model": "surface_contact_only",
"direct_velocity_writes_nominal": 0
```

This is valuable because it proves engineering maturity: the motion is simulated, not animated.

## Priority 3: Realistic Materials and Friction

### Target

Each item type should have reasonable physical properties:

```text
cardboard box: medium friction, low bounce
plastic bottle: low-medium friction, can roll
soft sack: high friction, high damping
helmet/dome: curved contact, rotation risk
detergent bottle: medium mass, irregular base
plate/cylinder: rolling risk
```

### Tasks

Create a material table in code:

```text
slug -> mass, static_friction, dynamic_friction, restitution, damping
```

Run sweeps:

```text
friction multiplier: 0.7, 1.0, 1.3
mass multiplier: 0.8, 1.0, 1.3
spawn yaw: multiple orientations
spacing: nominal and close
```

### Acceptance Criteria

Final evidence should report:

```text
N seeds
N items
N material perturbations
0 unsafe errors
containment = 1.0
routing accuracy >= target
```

Ideal target:

```text
>= 6 seeds
>= 66 item trials
>= 3 material sweeps
```

## Priority 4: More Credible Collisions

### Problem

Simple convex colliders are robust, but they can make the physics look less credible.

### Target

Use collision fidelity by item category:

- boxes: clean box colliders;
- bottles/cylinders: cylinder/capsule compound colliders;
- helmet/dome: simplified convex decomposition;
- sack: compound rigid approximation with soft-looking behavior;
- complex objects: separate collision proxy and visual mesh.

### Acceptance Criteria

Each item should have:

```text
visual mesh != necessarily collision mesh
documented collision proxy
realistic mass
reasonable center of mass
```

This makes the project look engineered, not just "STL imported into a scene".

## Priority 5: RTX Sensors with Realistic Imperfections

### Already Strong

RTX-camera classification is already a strong part of the project. Do not reopen it as the main problem.

### Improvements

Add controlled imperfections:

- depth noise;
- limited dropout;
- occlusion from tilted items;
- readout latency;
- multi-read fusion;
- confidence score;
- safe-side routing when uncertain.

### Acceptance Criteria

For each item, log:

```text
raw_measurements
fused_measurement
classification
confidence
route_command_time
table_entry_time
command_margin_s
```

`command_margin_s` must prove that the route decision is ready before physical actuation is needed.

## Priority 6: Actuation Faults and Recovery

### Required Fault Cases

A high-level solution must show more than the happy path:

```text
1. item arrives off-center;
2. item enters rotated;
3. item is partly over two actuator patches;
4. low friction causes weak lateral motion;
5. heavy item responds slowly;
6. chute receives the item at high speed;
7. output jam;
8. sensor is correct but actuator response is delayed.
```

### Correct Behavior

The system should:

- detect that the item is not progressing;
- block the next item if needed;
- trigger the watchdog;
- call the exception arm or operator callout;
- recover or fail safe;
- never send an unsafe item to B silently.

### Acceptance Criteria

Evidence runs:

```text
nominal run: all items
fault run: injected jam
low friction run
close spacing run
heavy item run
```

Each run should generate:

```text
summary.json
events.csv
overview video
transfer-deck close-up video
optional depth stills
```

## Priority 7: Final Industrial Video Evidence

### Required Shots

Do not record only one wide view. The final evidence needs layers:

1. Industrial overview:
   - whole cell;
   - A, B, C, D visible;
   - complete flow.

2. Sensor station:
   - item passing through RTX cameras;
   - overlay with dimensions/circularity/confidence.

3. Transfer deck close-up:
   - patches/rollers activating;
   - gates/blades moving;
   - physical contact visible.

4. Chute/cage close-up:
   - entry into C/D;
   - hood/rails;
   - item contained.

5. Fault recovery:
   - jam;
   - watchdog;
   - arm recovery or operator callout.

### Visual Acceptance Criteria

The judges should understand without extra explanation:

```text
why the item goes to B/C/D
what physical mechanism moves it
what happens if it fails
why this could exist in a real warehouse
```

## Priority 8: Calculations Cross-Checked Against Simulation

Simulation alone is not enough for maximum readiness. It should be backed by simple engineering calculations.

Create or update a calculations document with:

```text
belt speed = 1 m/s
item mass range
ARB surface speed
required lateral displacement
available table length
time on deck
friction assumptions
chute angle 32 deg
entry speed to cage
cycle time
throughput items/hour
actuator response time
safe stopping/jam timeout
```

Key formulas:

```text
lateral_displacement = effective_lateral_velocity * time_on_deck
time_on_deck = deck_length / forward_velocity
```

The calculations should prove:

```text
the item has enough time to divert
the lateral velocity is not absurd
cage entry is not violent
the system returns to ready before the next item
```

## Priority 9: Final Validation Matrix

### Minimum Runs

```text
seed42_nominal_all_items
six_seed_nominal_all_items
close_spacing
low_friction
high_mass
fault_jam_recovery
recorded_showcase
```

### Target Metrics

```text
classification_accuracy >= 0.98
routing_accuracy >= 0.97
unsafe_errors = 0
containment_rate = 1.0
deadlocks = 0
operator_callouts explained, not silent failures
realtime_factor recorded
cycle_time mean/p95/max recorded
```

### What Scores Well

A safe callout can be better than a fake perfect demo. The important proof is:

```text
fault detected
fault contained
fault recovered or escalated
no unsafe feed to B
```

## How to Use the RTX 5090

### Use It For

- Isaac runs with RTX perception;
- material sweeps;
- final videos;
- close-up rendering;
- multi-seed validation;
- profiling bottlenecks;
- screenshots and depth stills.

### Do Not Use It For

- copying old evidence for hours;
- storing historical outputs already available locally or on the 5070 Ti;
- waiting on slow transfers;
- Markdown/document editing.

The 20 GB old `/root/evidence` folder is not required for development. The 5090 should generate new final evidence.

## Recommended Work Order

### Day 1: Real ARB Deck

1. Create a matrix of patches/rollers in Isaac.
2. Give each patch its own surface velocity.
3. Activate patches based on route state.
4. Log actuator commands.
5. Run three items: B, C, D.

### Day 2: Physics and Materials

1. Add the material table.
2. Add collision proxies.
3. Run mass/friction sweeps.
4. Tune velocities and gates.
5. Confirm no nominal `set_v`.

### Day 3: Faults

1. Off-center entry.
2. Low friction.
3. Heavy item.
4. Jam.
5. Recovery.
6. Operator callout if unrecoverable.

### Day 4: Final Evidence

1. Six-seed run.
2. Overview showcase video.
3. Transfer deck close-up.
4. Sensor close-up.
5. Fault recovery video.
6. Consolidated `summary.json`.

### Day 5: Technical Documentation

1. Update README/validation report.
2. Add calculations vs simulation.
3. Add results table.
4. Add honest limitations.
5. Build a reproducible final package.

## Definition of "Super Realistic" for This Hackathon

It does not mean cinematic rendering. It means:

```text
real scale geometry
plausible sensors
classification with noise and latency
visible physical actuators
contact/friction/gravity
possible faults
safe recovery or escalation
reproducible metrics
calculations supporting the simulation
videos showing the mechanism, not only the result
```

## Key Decision

The best use of time is not a weak physical prototype. For this project, the strongest path is:

```text
high-fidelity industrial Isaac Sim
+ physical patch/roller actuation
+ multi-seed validation
+ calculations cross-check
+ strong visual evidence
```

This matches the expert clarification: a physical prototype is not mandatory, but realism, readiness, and industrial plausibility are evaluated. A validated, measurable, realistic simulation can score higher than an improvised physical mockup.
