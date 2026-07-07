# SortMaster Engineering Improvement Agent Brief

> ⚠️ **Partially superseded (2026-07-06).** The instruction below to *not*
> migrate to Isaac Sim is an OLD-version constraint. Current direction: the
> Isaac Sim implementation is **obligatory** and lives in `isaac/` (built from
> the same `cell/params.py`, run on the team GPU server). The MuJoCo engine
> remains the deterministic validation twin. See README §5.1.

## Purpose

This document is written for an AI coding/engineering agent that will modify the SortMaster repository.

Focus only on engineering implementation. Do not work on presentation, speech, storytelling, defense text, timeline planning, or slide content. The goal is to make the simulated sorting cell more physically convincing, measurable, reproducible, and robust.

## High-level target

Upgrade the current MuJoCo-based SortMaster simulation so that it clearly demonstrates a complete closed loop:

```text
virtual sensor measurement
→ geometric classification
→ route command generation
→ active transfer-table execution
→ physical movement to B/C/D
→ destination containment verification
→ metrics/logging
→ fault recovery when needed
```

The primary executive mechanism must remain the transfer table. The robotic arm must be used only for exception recovery, not normal routing.

---

# 1. Preserve the current architecture

## Requirements

Keep these architectural decisions intact:

```text
Physics/simulation core: MuJoCo
Primary executive: tri-directional transfer table
Exception mechanism: robotic arm for jams/recovery only
Classification logic: geometry-first rule engine
Nominal perception mode: camera / virtual sensor
Debug perception mode: oracle
```

Do not migrate the core simulation to Isaac Sim, Webots, Gazebo, Unreal, or Blender.

Do not make the robotic arm the main normal-flow sorting mechanism again.

Do not replace the rule-based B/C/D classifier with a learned object-name classifier.

---

# 2. Make physical routing unmistakable

## Problem to solve

The simulation must make it visually and physically clear how each item reaches B, C, or D. The object should not appear to teleport, slide randomly, or be routed by invisible logic.

## Required behavior

Implement or improve visible route mechanisms for all three destinations:

```text
B → main sorter connector
C → oversize cage / roll container
D → repack cage / roll container
```

Each route must have a physically plausible path from the transfer table to the destination.

## Engineering tasks

Add or improve:

- visible directional rollers on the transfer table;
- active route zones on the table;
- side guides;
- chutes or short connector conveyors;
- diverter flaps or gates where useful;
- physical boundaries that guide the object into the correct destination;
- route-specific visual indicators on the mechanism;
- route command state visible in the simulation overlay or debug panel.

## Acceptance criteria

For every item in `base.yaml` running with `--perception camera --executive table`:

```text
1. The item reaches the transfer table.
2. The table activates a route-specific physical behavior.
3. The item moves through a visible path.
4. The item enters the correct B/C/D destination.
5. The item remains contained.
6. A log row records the route command and final result.
```

---

# 3. Add robust containment validation

## Problem to solve

Routing is not complete unless the item stays inside the assigned destination. If objects bounce out, escape, or settle outside the cage volume, the routing result should not be considered successful.

## Required behavior

Define routing success as:

```text
routing_success = correct destination reached AND item remains contained at cycle end
```

## Engineering tasks

Implement destination containment zones for B, C, and D.

For C and D cages, improve geometry if necessary:

- higher walls;
- backstops;
- side walls;
- soft internal boundaries;
- entry funnels;
- damping/friction zones;
- final settle-check volume.

Add code-level checks:

```text
entered_destination_zone
settled_in_destination_zone
contained_at_cycle_end
containment_violation
```

Split failure types:

```text
classification_error
routing_error
containment_error
unsafe_error
conservative_error
```

## Metrics to add or verify

```text
containment_rate
containment_violations
contained_success_count
containment_error_count
cage_entry_speed_mean_mps
cage_entry_speed_max_mps
cage_settle_mean_s
```

## Acceptance criteria

A run should not mark an item successful if it reached the correct destination briefly but later exited the containment zone.

---

# 4. Reduce unrealistic cage entry behavior

## Problem to solve

Current metrics may show high cage entry speeds, for example values around 1.5–1.8 m/s. Even if containment succeeds, this can look like the item is being thrown rather than controlled.

## Required behavior

Make C/D cage entry look physically controlled.

## Engineering tasks

Tune or add:

- longer chutes;
- friction strips;
- damping at cage entry;
- deceleration rollers;
- soft landing zones;
- better backstop geometry;
- route speed limiting near cage entry;
- collision materials tuned for stable settling.

## Target

Do not hard-code fake success. Reduce physical aggressiveness while keeping throughput reasonable.

Track before/after metrics:

```text
cage_entry_speed_mean_mps
cage_entry_speed_max_mps
cage_settle_mean_s
containment_rate
throughput_items_per_h
```

---

# 5. Clarify and parameterize the virtual sensor model

## Problem to solve

The `camera` perception mode must not look like hidden ground truth. It should be implemented and documented as a plausible virtual industrial sensor.

## Required behavior

The sensor should be modeled as a virtual depth / dimensioning station, similar to an industrial DWS or multi-head depth profiling tunnel.

It should classify from simulated measurements, not from:

```text
object name
STL filename
pre-known category label
ground truth dimensions directly injected into decision logic
```

## Engineering tasks

Add or formalize a sensor configuration block, preferably in YAML or a clearly visible Python config.

Suggested structure:

```yaml
virtual_sensor:
  type: multi_head_depth_profiler
  model: ray_cast_depth_grid
  conveyor_speed_mps: 1.0
  sensor_height_m: 1.20
  fov_width_m: 0.70
  fov_length_m: 0.90
  ray_spacing_mm: 5
  update_rate_hz: 20
  depth_noise_mm: 3
  processing_latency_ms: 80
  decision_margin_s: 0.50
```

Use implementation-true values. Do not invent values that are not actually used.

## Sensor parameters to expose

At minimum expose:

```text
sensor type
sensor position x/y/z
sensor height
field of view width/length
sampling density or ray spacing
update rate
noise model
processing latency
blind zones / occlusion assumptions
decision deadline before transfer-table entry
```

## Acceptance criteria

A developer or reviewer should be able to inspect the repo and answer:

```text
What does the virtual sensor simulate?
Where is it located?
How dense is its sampling?
Does it have noise?
Does it have processing latency?
How does it produce dimensions/circularity?
How is it different from oracle mode?
```

---

# 6. Make `camera` mode and `oracle` mode explicitly different

## Problem to solve

The repo must make it obvious that `camera` mode is simulated perception, while `oracle` mode is only a debugging baseline.

## Required behavior

`camera` mode:

```text
virtual sensor measurements
→ measured dimensions / circularity
→ rule engine
→ route command
```

`oracle` mode:

```text
ground-truth category or dimensions
→ route command
```

Oracle must not be used as the default evidence for the final nominal run.

## Engineering tasks

Make mode explicit in:

- CLI output;
- run folder name;
- `summary.json`;
- `events.csv`;
- optional viewer overlay.

Add fields:

```text
perception_mode
sensor_model
oracle_used_for_classification
```

For `camera` mode, `oracle_used_for_classification` must be `false`.

## Acceptance criteria

The following commands should produce clearly distinguishable logs:

```powershell
.\.venv\Scripts\python.exe -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception camera --executive table
.\.venv\Scripts\python.exe -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception oracle --executive table
```

---

# 7. Prevent apparent camera blocking

## Problem to solve

In viewer mode, the object may appear to slow down or freeze under the camera. If this is only viewer/perception lag, logs should prove it. If the simulation is actually blocking the conveyor to classify, that must be fixed.

The input conveyor A should not stop for classification in the base scenario.

## Required behavior

Classification must happen while the object is moving. The route command must be ready before the item reaches the transfer table.

## Engineering tasks

Add timing instrumentation:

```text
detection_time_s
sensor_capture_start_s
sensor_capture_end_s
classification_start_s
classification_end_s
route_command_time_s
table_entry_time_s
route_complete_time_s
containment_confirm_time_s
perception_latency_ms
command_margin_s
conveyor_a_speed_mps
conveyor_a_stopped_for_classification
```

Compute:

```text
perception_latency_ms = 1000 * (classification_end_s - sensor_capture_start_s)
command_margin_s = table_entry_time_s - route_command_time_s
```

Add summary stats:

```text
perception_latency_ms_mean
perception_latency_ms_p95
command_margin_s_min
command_margin_s_mean
items_with_negative_command_margin
conveyor_a_stop_count
```

## Acceptance criteria

For nominal `camera/table` runs:

```text
conveyor_a_stopped_for_classification = false for all items
command_margin_s > 0 for all normal items
items_with_negative_command_margin = 0
```

If viewer lag exists, it should be documented by comparing `sim_time_s` and `wall_time_s`, not hidden.

---

# 8. Add cycle time metrics and remove null cycle fields

## Problem to solve

`cycle_mean_s`, `cycle_p95_s`, and `cycle_max_s` must not be null in final metrics.

## Required behavior

Define cycle time consistently and compute it for every item.

Recommended definition:

```text
cycle_start = item enters detection zone
cycle_end = containment confirmed in assigned destination
cycle_time = cycle_end - cycle_start
```

Alternative definitions are acceptable only if clearly named.

## Engineering tasks

Add per-item event fields:

```text
cycle_start_s
cycle_end_s
cycle_time_s
```

Add summary fields:

```text
cycle_mean_s
cycle_p95_s
cycle_max_s
cycle_min_s
```

Keep existing transit metrics if useful, but do not use transit time as an undefined substitute for cycle time.

## Acceptance criteria

`summary.json` should not contain null for:

```text
cycle_mean_s
cycle_p95_s
cycle_max_s
```

unless zero items were processed, which should be treated as run failure.

---

# 9. Strengthen event logging

## Required per-item event fields

Ensure `events.csv` or equivalent structured log contains at least:

```text
run_id
seed
scenario
item_id
asset_name
perception_mode
executive_mode
sensor_model
detected_time_s
measured_length_m
measured_width_m
measured_height_m
measured_circularity
classification_confidence
rule_dimension_result
rule_circularity_result
predicted_category
ground_truth_category
classification_correct
route_command
route_command_time_s
table_entry_time_s
active_table_direction
destination_expected
destination_reached
destination_entry_time_s
contained_at_cycle_end
containment_violation
unsafe_error
conservative_error
arm_intervention
recovery_action
recovery_success
cycle_time_s
transit_time_s
perception_latency_ms
command_margin_s
```

## Summary metrics

Ensure `summary.json` contains:

```text
items
routed_correctly
routing_accuracy
classification_accuracy
executive_accuracy
end_to_end_accuracy
unsafe_errors
conservative_errors
containment_rate
containment_violations
cage_entry_speed_mean_mps
cage_entry_speed_max_mps
cage_settle_mean_s
arm_interventions
recovery_success
routing_no_intervention
throughput_items_per_h
transit_mean_s
transit_p95_s
cycle_mean_s
cycle_p95_s
cycle_max_s
perception_latency_ms_mean
perception_latency_ms_p95
command_margin_s_min
command_margin_s_mean
items_with_negative_command_margin
conveyor_a_stop_count
sim_time_s
wall_time_s
seed
perception
executive
scenario
```

---

# 10. Expand scenario coverage

## Required scenarios

Create or verify these scenario files:

```text
scenarios/base.yaml
scenarios/borderline.yaml
scenarios/fault_jam.yaml
scenarios/close_spacing.yaml
scenarios/low_confidence.yaml
scenarios/failed_transfer.yaml
scenarios/stress_mix.yaml
```

## Scenario definitions

### `base.yaml`

Purpose: normal official object set.

Should verify:

```text
normal camera/table routing
all three categories B/C/D
containment
nominal throughput
zero arm intervention
```

### `borderline.yaml`

Purpose: classification threshold cases.

Include:

```text
near oversize threshold
near undersize threshold
near circularity threshold
ambiguous orientation
objects close to 450 x 320 x 320 mm threshold
objects close to minimum dimension threshold
```

Expected behavior:

```text
no unsafe errors
borderline low-confidence items may be conservatively routed to D or safe review
```

### `fault_jam.yaml`

Purpose: exception handling.

Should force or simulate:

```text
item stuck on transfer table
watchdog timeout
normal route paused or isolated
arm intervention
item cleared to correct cage or safe recovery zone
system reset
```

Metrics:

```text
arm_interventions > 0
recovery_success_rate
average_recovery_time
no deadlock
```

### `close_spacing.yaml`

Purpose: close-arrival behavior.

Should test:

```text
two items close together
single-item discipline in measurement window
gating or spacing behavior
no merged point-cloud classification error
no unsafe route
```

### `low_confidence.yaml`

Purpose: uncertain perception behavior.

Should test:

```text
partial sensor view
high measurement noise
borderline shape
ambiguous circularity
```

Expected behavior:

```text
route conservatively
never send uncertain unsuitable item to B
```

### `failed_transfer.yaml`

Purpose: command issued but physical movement fails.

Should test:

```text
route command generated
item fails to exit table as expected
watchdog detects no progress
recovery action triggered
```

### `stress_mix.yaml`

Purpose: longer stability run.

Should test:

```text
50–100 items
mixed categories
randomized order
multiple seeds
stable metrics
no deadlock
```

---

# 11. Add batch validation runner

## Problem to solve

The project should not require manual one-off commands to build evidence.

## Required behavior

Add a script or CLI mode to run multiple scenarios and seeds and generate a validation summary.

## Engineering tasks

Implement one of:

```powershell
.\.venv\Scripts\python.exe -m cell.validate --matrix configs/validation_matrix.yaml
```

or:

```powershell
.\.venv\Scripts\python.exe -m cell.run_batch --matrix configs/validation_matrix.yaml
```

Example matrix:

```yaml
runs:
  - scenario: scenarios/base.yaml
    seeds: [1,2,3,4,5,6,7,8,9,10]
    perception: camera
    executive: table
  - scenario: scenarios/borderline.yaml
    seeds: [1,2,3,4,5,6,7,8,9,10]
    perception: camera
    executive: table
  - scenario: scenarios/fault_jam.yaml
    seeds: [1,2,3]
    perception: camera
    executive: table
  - scenario: scenarios/stress_mix.yaml
    seeds: [1,2,3,4,5]
    perception: camera
    executive: table
```

Output:

```text
runs/<timestamp>/summary.json
runs/<timestamp>/events.csv
runs/<timestamp>/validation_matrix.csv
runs/<timestamp>/validation_report.md
```

## Acceptance criteria

One command should produce a table with scenario-level metrics:

```text
scenario
seeds
items
classification_accuracy
routing_accuracy
containment_rate
unsafe_errors
conservative_errors
cycle_p95_s
throughput_items_per_h
arm_interventions
recovery_success_rate
deadlocks
```

---

# 12. Improve fault detection and recovery logic

## Required behavior

The system must detect when routing has failed or stalled and recover safely.

## Engineering tasks

Implement or verify a watchdog with states similar to:

```text
IDLE
ITEM_DETECTED
CLASSIFIED
ROUTE_COMMAND_SENT
ROUTING_ACTIVE
VERIFY_EXIT
VERIFY_CONTAINMENT
RESET
JAM_DETECTED
ARM_RECOVERY
OPERATOR_CALLOUT
```

Trigger jam/failure if:

```text
item does not exit transfer table within timeout
item velocity drops below threshold for too long
item is outside expected route corridor
item enters wrong destination corridor
item remains in measurement/table zone too long
```

Recovery behavior:

```text
pause or isolate affected zone
activate exception arm
clear item to assigned cage or safe recovery zone
resume normal operation
log recovery result
```

Metrics:

```text
jam_detected_count
arm_interventions
recovery_success_rate
average_recovery_time_s
operator_callouts
deadlocks
```

## Acceptance criteria

`fault_jam.yaml` and `failed_transfer.yaml` should demonstrate nonzero interventions with successful recovery and no deadlock.

---

# 13. Improve active transfer-table state model

## Problem to solve

The transfer table should behave like an active actuator, not a passive surface.

## Required behavior

The table must have explicit route states:

```text
TABLE_IDLE
TABLE_ROUTE_B
TABLE_ROUTE_C
TABLE_ROUTE_D
TABLE_STOP
TABLE_RECOVERY_LOCKOUT
```

## Engineering tasks

Expose route state in:

- controller code;
- logs;
- viewer overlay;
- optional material/visual indicator.

For each route command:

```text
ROUTE_B → activate B direction until exit confirmed
ROUTE_C → activate C direction until exit confirmed
ROUTE_D → activate D direction until exit confirmed
```

After completion:

```text
reset table state to TABLE_IDLE
confirm no item remains on table
accept next item
```

## Acceptance criteria

The logs should show one route state transition sequence per item.

---

# 14. Protect against merged-object perception errors

## Problem to solve

If two objects are too close under the sensor, the point cloud may merge and produce wrong dimensions or circularity.

## Required behavior

The system must enforce or verify single-item discipline in the measurement window.

## Engineering tasks

Implement one or more:

- upstream spacing gate;
- measurement-window occupancy check;
- object tracking IDs;
- split connected components in point cloud/depth mask;
- low-confidence fallback if multiple objects are detected;
- route to D/review for ambiguous merged detections.

Add metrics:

```text
multi_object_window_count
merged_detection_count
spacing_gate_activations
low_confidence_fallback_count
```

## Acceptance criteria

`close_spacing.yaml` should not produce unsafe errors.

---

# 15. Add low-confidence safe fallback

## Required behavior

If the classifier cannot confidently decide that an item is B, it must not send it to B.

## Recommended rule

```text
If low confidence or ambiguous measurement:
    route to D or safe review lane
```

## Engineering tasks

Expose confidence components:

```text
dimension_margin
circularity_margin
sensor_coverage_score
point_count
occlusion_score
classification_confidence
```

Add thresholds to config:

```yaml
classification:
  min_confidence_for_B: 0.80
  low_confidence_route: D
```

Log:

```text
low_confidence
fallback_route
fallback_reason
```

## Acceptance criteria

`low_confidence.yaml` should produce zero unsafe errors, even if conservative errors increase.

---

# 16. Update README for engineering reproducibility only

## Required README fixes

Remove or clearly mark outdated arm-primary/PyBullet wording.

README must state:

```text
Primary executive: transfer table
Exception mechanism: robotic arm
Core simulator: MuJoCo
Perception modes: camera and oracle
Camera mode: virtual depth/ray-cast sensor
Oracle mode: ground-truth debug baseline
```

Add exact commands:

```powershell
.\.venv\Scripts\python.exe -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception camera --executive table --viewer
.\.venv\Scripts\python.exe -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception camera --executive table
.\.venv\Scripts\python.exe -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception oracle --executive table --viewer
```

Add validation command if implemented:

```powershell
.\.venv\Scripts\python.exe -m cell.validate --matrix configs/validation_matrix.yaml
```

Add a brief section:

```text
Virtual sensor model
```

with implementation-true parameters.

Do not include marketing language or speech notes.

---

# 17. Maintain backward-compatible debugging modes

## Required behavior

Do not delete useful baselines.

Keep:

```text
--perception oracle
--perception camera
--executive table
--executive arm
```

If `--executive arm` exists as historical baseline, label it clearly as:

```text
arm-primary-baseline
```

The default should be:

```text
--perception camera --executive table
```

unless there is a strong reason not to change defaults.

---

# 18. Add automated tests or smoke checks

## Required checks

Add lightweight tests or smoke scripts for:

```text
base camera/table run completes
summary.json contains no null cycle fields
containment_rate is present
perception mode is logged
executive mode is logged
command_margin_s is computed
camera mode does not use oracle classification
fault_jam produces arm intervention
```

Example smoke command:

```powershell
.\.venv\Scripts\python.exe -m cell.smoke_test
```

or use `pytest` if already configured.

## Acceptance criteria

A reviewer can run one test command and verify the critical engineering claims without opening the viewer.

---

# 19. Final engineering acceptance checklist

Before considering the changes complete, verify all items below.

## Nominal routing

```text
[ ] base.yaml runs with --perception camera --executive table
[ ] all B/C/D routes are physically visible
[ ] routing_accuracy is reported
[ ] classification_accuracy is reported
[ ] executive_accuracy is reported
[ ] containment_rate is reported
[ ] unsafe_errors is reported
[ ] arm_interventions = 0 in nominal base runs
```

## Sensor model

```text
[ ] virtual sensor configuration exists
[ ] sensor type is documented
[ ] sensor position/height/FOV/sampling are exposed
[ ] noise and latency are exposed or explicitly set to zero
[ ] camera mode does not classify from object name or pre-known category
[ ] oracle mode is clearly marked as debug baseline
```

## Timing

```text
[ ] detection_time_s logged
[ ] route_command_time_s logged
[ ] table_entry_time_s logged
[ ] perception_latency_ms logged
[ ] command_margin_s logged
[ ] command_margin_s > 0 in nominal runs
[ ] conveyor_a_stopped_for_classification = false
[ ] cycle_mean_s, cycle_p95_s, cycle_max_s are not null
```

## Containment

```text
[ ] destination containment zones implemented
[ ] containment confirmed after settling
[ ] containment violations counted
[ ] routing success requires containment
[ ] cage entry speeds logged
[ ] cage entry behavior looks physically controlled
```

## Faults and recovery

```text
[ ] fault_jam.yaml exists
[ ] failed_transfer.yaml exists
[ ] watchdog detects stalled routing
[ ] exception arm is triggered only on fault/recovery
[ ] recovery_success is reported
[ ] deadlocks are reported
```

## Batch validation

```text
[ ] multiple seeds can be run without manual repetition
[ ] scenario-level validation table is generated
[ ] events.csv and summary.json are produced for each run
[ ] validation_report.md or equivalent is generated
```

---

# 20. Do not implement these changes

Avoid scope creep.

Do not:

```text
migrate the validated core to Isaac Sim
rewrite the whole project around Blender/Unreal
make the robotic arm primary again
replace geometric rules with object-name classification
hide failures by hard-coding success
remove oracle mode if it is useful for debugging
remove arm-primary baseline if it is useful for comparison
change the official B/C/D rule priority
stop the input conveyor under the camera in the base scenario
```

---

# 21. Expected final repository state

After completing this brief, the repository should support:

```powershell
.\.venv\Scripts\python.exe -m cell.run_sim --scenario scenarios/base.yaml --seed 42 --perception camera --executive table --viewer
```

and produce a run where:

```text
perception = camera
executive = table
classification uses virtual sensor measurements
route commands are generated before table entry
objects are physically routed through visible mechanisms
items remain contained in B/C/D
timing metrics are complete
cycle metrics are not null
fault scenarios can be run separately
README explains how to reproduce the evidence
```

The resulting project should be an engineering-valid simulation, not only a visual animation.
