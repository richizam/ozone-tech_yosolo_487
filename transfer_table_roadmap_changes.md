# Roadmap Update: Transfer Table as Main Routing Mechanism + Robotic Arm for Exceptions

> ⚠️ **Note (2026-07-06):** the "Isaac Sim optional" stance in this document
> is an OLD version. Isaac Sim is now an **obligatory deliverable**: the port
> lives in `isaac/` and runs on the team GPU server. See README §5.1.

## Context

The current simulation proves the closed loop: items enter through conveyor A, are classified, and are routed to B/C/D. However, the current execution concept relies on a robotic arm as the primary actor that picks and places every object.

That is functional in simulation, but it creates a weak point for real-world plausibility: a single robot gripper would need to handle many unknown materials, shapes, weights, textures, and orientations. That is not the strongest industrial architecture for arbitrary product sorting.

The recommended change is to make the **transfer/diverter table the main routing mechanism** and keep the **robotic arm as an exception-handling device** for jams, failed transfers, misaligned objects, or abnormal cases.

---

## Core Design Change

### Current logic

```text
Conveyor A
  → perception / classification
  → accumulator
  → robotic arm picks every item
  → robotic arm places item into B, C, or D
```

### Proposed logic

```text
Conveyor A
  → perception / classification
  → widened accumulator / transfer table
  → non-grasping routing to B, C, or D
  → robotic arm only handles exceptions
```

This makes the design more robust because the normal routing path does not depend on grasping the object.

---

## Why This Change Is Better

### 1. Avoids universal gripping risk

A robot arm with one gripper cannot be assumed to reliably pick every possible item:

- cardboard boxes;
- plastic bottles;
- soft sacks;
- helmets;
- plates;
- cylinders;
- deformable objects;
- smooth, porous, dusty, or irregular surfaces.

A transfer/diverter table does not need to understand how to grip each object. It only needs to move the object along a controlled surface.

### 2. Better industrial plausibility

In logistics and sorting systems, non-grasping routing is often more realistic for mixed items. The system should push, divert, transfer, or guide objects rather than pick every object individually.

### 3. Higher throughput potential

A robot arm becomes a bottleneck if it must pick every item. A transfer table can route items continuously or semi-continuously, while the arm is reserved for low-frequency exceptions.

### 4. More aligned with the official layout

The official diagram shows conveyor A feeding into a wider final area. The current simulation looks more like a straight conveyor ending near a robot arm. Adding a widened transfer/accumulator area makes the model visually and functionally closer to the intended layout.

### 5. Better fault handling story

The arm becomes a recovery mechanism rather than the core throughput mechanism. This improves the engineering narrative:

```text
Normal item → routed by table
Abnormal item → handled by robotic arm
Unsafe / uncertain item → diverted to D or manual review
```

---

## Proposed Physical Concept

### Main subsystem: transfer/diverter table

The transfer table receives objects after the perception zone and routes them according to the classification decision.

Possible implementations:

1. **Powered roller table with lateral diverters**
   - Default path goes to B.
   - Side diverter sends items to C or D.
   - Simple and robust.

2. **Multi-directional roller table**
   - Uses controllable roller directions.
   - Can send items left, forward, or right.
   - More elegant but more complex.

3. **Pop-up transfer module**
   - Object arrives on the main table.
   - A transverse belt or roller section lifts and transfers the object sideways.
   - Good industrial plausibility.

4. **Pusher-based diverter**
   - Simplest to simulate.
   - Acceptable for a hackathon if timing and collision behavior are modeled.

### Recommended option for the roadmap

Use a **tri-directional powered transfer table** as the target design, and implement it in MuJoCo as simplified controlled surface zones.

```text
                 B
                 ↑
A → perception → [ transfer table ]
              ↙                  ↘
             C                    D
```

---

## Role of the Robotic Arm

The robotic arm should no longer be responsible for every item.

### New role: exception handler

The arm intervenes only when the table cannot complete the normal routing cycle.

Examples:

- item stuck on the accumulator;
- item rotated into an unstable position;
- two items too close together;
- failed transfer;
- object partially outside the table;
- object classified as low-confidence;
- jam detected;
- object requires removal to D/manual review.

### Arm behavior

```text
If normal routing succeeds:
    arm stays idle

If jam / failed transfer / misalignment:
    stop or slow local zone
    arm clears, repositions, or removes item
    resume normal routing
```

This makes the arm much more believable because it no longer needs to be a universal high-throughput picker.

---

## Changes Needed in the Simulation Structure

## Phase 1 — Preserve the current working version

Before changing architecture, freeze the current version.

### Actions

- Tag the current version as the last `arm-primary` working baseline.
- Save current metrics.
- Save a short video of the current robot-arm routing.
- Keep this version as fallback.

### Deliverables

```text
tags/v0-arm-primary
runs/baseline_arm_primary/
docs/metrics/arm_primary_summary.json
videos/arm_primary_demo.mp4
```

---

## Phase 2 — Add the widened accumulator / transfer area

The current scene should visually match the official layout better.

### Actions

- Add a widened area after conveyor A.
- Make the final section look like an accumulator / transfer table.
- Keep the current arm temporarily.
- Add labels A, B, C, D.
- Add a fixed top-view camera and an isometric presentation camera.

### Expected result

The scene should read visually as:

```text
A narrow conveyor → widened transfer zone → B/C/D destinations
```

### Deliverables

```text
cell/scene_transfer_table.py or updated scene generator
fixed cameras: overview, top_view, table_view
updated visual demo screenshots
```

---

## Phase 3 — Implement table-based routing

The table becomes the main executor.

### Simulation simplification

In MuJoCo, the table does not need to model every roller. It can be implemented as controlled surface zones applying directional motion to the object.

Example:

```text
center zone velocity → forward to B
left diverter zone velocity → left to C
right diverter zone velocity → right to D
```

### Control logic

```python
if category == "B":
    route = "forward"
elif category == "C":
    route = "left"
elif category == "D":
    route = "right"
```

### Actions

- Add transfer-table actuator states.
- Add route commands from the controller to the table.
- Log the requested route and actual destination.
- Validate that objects physically arrive at B/C/D.

### Deliverables

```text
cell/transfer_table.py
cell/controller.py updated
runs/transfer_table_seed*/events.csv
runs/transfer_table_seed*/summary.json
```

---

## Phase 4 — Move the arm to exception handling

The arm is no longer used in normal routing.

### Exception conditions

Add explicit detection logic for:

- item not moving after command;
- item outside expected corridor;
- two items too close;
- failed arrival within time limit;
- item stuck at transfer table boundary;
- object classified with low confidence.

### Exception response

```text
jam detected
  → pause local routing
  → arm moves to recovery pose
  → arm pushes/repositions/removes object
  → object goes to D/manual review if uncertain
  → table resumes
```

### Deliverables

```text
cell/exception_handler.py
cell/arm_recovery.py
scenarios/fault_jam.yaml
scenarios/fault_close_items.yaml
metrics: exception recovery success rate
```

---

## Phase 5 — Update metrics

The new architecture needs different metrics.

### Keep existing metrics

- classification accuracy;
- executive routing accuracy;
- end-to-end accuracy;
- unsafe errors;
- conservative errors;
- cycle time.

### Add new metrics

- table routing success rate;
- average routing time by destination;
- jam detection rate;
- arm intervention rate;
- recovery success rate;
- false jam rate;
- throughput with and without exceptions.

### Example summary

```json
{
  "routing_mode": "transfer_table_primary",
  "items_processed": 120,
  "table_routing_success": 0.983,
  "arm_intervention_rate": 0.042,
  "recovery_success": 0.960,
  "unsafe_errors": 0,
  "mean_cycle_time_s": 1.45,
  "estimated_capacity_items_per_hour": 1800
}
```

---

## Phase 6 — Update documentation and narrative

The README and final report should no longer present the robotic arm as the main routing device.

### Replace this idea

```text
A UR10-class arm picks every item and places it into B/C/D.
```

### With this idea

```text
A tri-directional powered transfer table performs normal high-throughput routing without grasping. A robotic arm acts as an exception-handling unit for jams, failed transfers, misalignment, and low-confidence cases.
```

### README architecture should become

```text
Conveyor A
  → vision station
  → perception core
  → decision engine
  → cell controller
  → transfer table route command
  → B/C/D
  → exception handler triggers arm only when needed
```

### Report sections to update

- Architecture;
- Executive mechanism;
- Manipulation logic;
- Timing and throughput;
- Fault handling;
- Engineering trade-offs;
- Simulation validation;
- Limitations and future physical prototype.

---

## Recommended New Roadmap

## Milestone 0 — Baseline preservation

**Goal:** Do not lose the working system.

- Save current arm-primary simulation.
- Save metrics and demo video.
- Tag repository.

**Exit gate:** current system can still be run with one command.

---

## Milestone 1 — Visual/layout correction

**Goal:** Make the simulation match the official layout better.

- Add widened accumulator / transfer zone.
- Add A/B/C/D labels.
- Add fixed cameras.
- Add floor/grid and destination zones.

**Exit gate:** top-view screenshot clearly resembles the official sorting cell structure.

---

## Milestone 2 — Transfer table primary routing

**Goal:** Remove the need for grasping during normal operation.

- Implement table routing to B/C/D.
- Route objects physically through simulated surface motion/diverters.
- Log route command, actual route, and destination.

**Exit gate:** at least 95% routing accuracy in nominal scenarios, with zero unsafe errors.

---

## Milestone 3 — Arm exception handling

**Goal:** Reintroduce the arm as a recovery device.

- Detect jams and abnormal object states.
- Use the arm to clear or move problematic objects.
- Route uncertain cases to D/manual review.

**Exit gate:** fault scenarios recover without unsafe sorter entries.

---

## Milestone 4 — Performance validation

**Goal:** Prove the new architecture is better than arm-primary routing.

Compare:

```text
arm-primary routing
vs
transfer-table-primary routing + arm exceptions
```

Metrics:

- cycle time;
- throughput;
- intervention rate;
- routing accuracy;
- unsafe errors;
- conservative errors;
- recovery success.

**Exit gate:** transfer-table architecture is faster, more robust, or more realistic, with documented trade-offs.

---

## Milestone 5 — Final presentation mode

**Goal:** Make the demo easy to understand.

- Add fixed camera views.
- Add route color coding.
- Add visible labels.
- Add terminal or overlay text per item.
- Record MP4 video.
- Keep the MuJoCo simulation reproducible.

**Exit gate:** a viewer understands the system within 5 seconds.

---

## Updated Repository Structure

Suggested additions:

```text
cell/
  transfer_table.py          # table routing physics/control
  exception_handler.py       # jam and abnormal-case detection
  arm_recovery.py            # arm used only for recovery
  scene_transfer.py          # widened table layout / presentation cameras

scenarios/
  base_transfer.yaml
  nominal_mix.yaml
  borderline_only.yaml
  fault_jam.yaml
  fault_close_items.yaml
  fault_failed_transfer.yaml

docs/
  metrics/
    transfer_table_validation.csv
    transfer_table_summary.json
  report/
    executive_mechanism_tradeoff.md
  diagrams/
    architecture_transfer_table.png
    state_machine_exception_handling.png

videos/
  transfer_table_demo.mp4
  fault_recovery_demo.mp4
```

---

## Updated Controller State Machine

```text
WAIT_FOR_ITEM
  → PERCEIVE
  → CLASSIFY
  → SELECT_ROUTE
  → TABLE_ROUTE_ACTIVE
  → VERIFY_DESTINATION
  → COMPLETE
```

Exception branch:

```text
TABLE_ROUTE_ACTIVE
  → JAM_OR_FAILURE_DETECTED
  → PAUSE_TABLE
  → ARM_RECOVERY
  → VERIFY_SAFE_STATE
  → ROUTE_TO_D_OR_RESUME
```

---

## Updated Decision Policy

Classification remains unchanged:

```text
1. Dimension gate first → C
2. Circular cross-section → D
3. Otherwise → B
```

Routing changes:

```text
B → forward path / main sorter infeed
C → oversize side route
D → repack/manual-review side route
```

Exception policy:

```text
If routing confidence is low or the object behaves unexpectedly:
    stop normal routing
    trigger arm recovery
    send to D/manual review if unresolved
```

---

## What Not to Do

Do not redesign perception again unless it fails.

Do not move everything to Isaac Sim before the MuJoCo engineering model is updated.

Do not make the robot arm responsible for all items if the design goal is arbitrary product material handling.

Do not model every roller in detail unless needed. A simplified controlled transfer surface is enough for the first validated simulation.

Do not prioritize visual realism over measurable routing behavior.

---

## Recommended Technical Priority

1. Preserve working baseline.
2. Add widened transfer-table layout.
3. Implement table-based routing.
4. Convert arm into exception handler.
5. Add fault scenarios.
6. Update metrics.
7. Improve visuals.
8. Optional: Isaac Sim / Blender high-fidelity video.

---

## Final Opinion

The current arm-primary simulation is a good proof that the loop can close, but it is not the strongest final architecture for arbitrary product sorting.

The better engineering direction is:

```text
Transfer table = normal high-throughput routing
Robotic arm = exception recovery
Perception = unchanged geometry-first classifier
Controller = routes normal cases and escalates abnormal cases
```

This design is more industrially plausible, easier to justify, more robust to unknown materials, and closer to the visual intent of the official layout.
