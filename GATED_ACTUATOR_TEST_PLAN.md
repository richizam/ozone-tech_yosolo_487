# Gated Actuator Test Plan

This document defines a separate experiment for testing physical route gates together with the ARB actuator deck. It should be used only after the current ARB work stabilizes, or in a separate branch/run, so it does not interfere with the active validation path.

## Purpose

Test whether adding physical gates/interlocks makes the sorting cell safer, more industrial, and easier to validate.

The core idea:

```text
RTX classification decides the target zone
-> only the selected exit gate opens
-> all non-selected gates remain physically closed
-> ARB patch actuators drive the item toward the selected route
-> exit sensors verify the item crossed the correct gate
-> containment logic verifies the item stayed in the destination
```

This is not a replacement for the ARB actuator deck. It is a safety and routing layer on top of it.

## Why This Is Worth Testing

### 1. More industrial behavior

Real sorters rarely rely on one perfect motion field. They combine:

- powered belts/rollers;
- pop-up actuators;
- side guides;
- route gates;
- flaps/blades;
- interlocks;
- verification sensors.

Adding gates makes the solution look less like a simulation trick and more like a real warehouse cell.

### 2. Safer routing

If the system selects route C, then routes B and D should be physically blocked. This makes the design robust against:

- weak lateral motion;
- item slip;
- rotation;
- partial contact with patches;
- delayed actuator response;
- low friction;
- heavy items.

### 3. Better scoring narrative

The rubric rewards physical routing correctness, manipulation quality, abnormal-case handling, and return-to-ready behavior.

The key engineering claim becomes:

```text
The route is not only commanded in software; non-selected exits are physically interlocked.
```

That is a strong readiness argument.

## Test Hypothesis

Adding normally-closed route gates will:

```text
increase routing robustness
reduce wrong-exit risk
improve visual explainability
make faults easier to detect
make reset/readiness measurable
```

The risk:

```text
gates can create new jams if timing, clearance, or geometry is wrong
```

So the test must measure both success and failure modes.

## Proposed Mechanism

### Default State

```text
B gate: closed
C gate: closed
D gate: closed
ARB patches: idle or slow forward
next item: held upstream until route is prepared
```

### Route B

```text
open B gate
keep C and D gates closed
activate forward/centering ARB patches
verify B exit crossing
close B gate
reset ARB deck
release next item
```

### Route C

```text
open C gate
keep B and D gates closed
activate C-side divert patches
verify C exit crossing
close C gate
reset ARB deck
release next item
```

### Route D

```text
open D gate
keep B and C gates closed
activate D-side divert patches
verify D exit crossing
close D gate
reset ARB deck
release next item
```

## Gate Model

Start simple but physically explicit:

```text
gate geometry: rectangular blade/flap/stop bar
motion: prismatic or revolute transform
state: closed, opening, open, closing, fault
travel time: 150-400 ms
open clearance: item height + safety margin
closed overlap: enough to block wrong exit
collision: enabled while closed/opening/closing
visual state: colored beacon or small indicator
```

Avoid:

```text
instant open/close
invisible collision walls
gates passing through items
gates that solve all motion artificially
```

## Controller State Machine

Recommended route controller:

```text
WAIT_ITEM
MEASURE
CLASSIFY
PREPARE_ROUTE
OPEN_SELECTED_GATE
ROUTE_ACTIVE
VERIFY_EXIT
CLOSE_GATE
RESET_DECK
RELEASE_NEXT
FAULT_RECOVERY
```

The controller should explicitly log state transitions.

## Observer and Feedback Layer

A lightweight observer is enough:

```text
state estimate:
  x, y, z
  vx, vy
  yaw estimate if available
  route confidence
  stuck_score
```

Inputs:

```text
RTX depth reads
known conveyor velocity
ARB patch commands
gate states
item contact/progress events
```

Useful residual:

```text
expected_progress = model(conveyor_speed, active_patches, route)
measured_progress = item_position_now - item_position_previous
residual = expected_progress - measured_progress
```

If the actuator is active and measured progress is too low:

```text
stuck_score += 1
if stuck_score > threshold:
  enter FAULT_RECOVERY
```

## Metrics to Add

Each run should write these fields into `summary.json`:

```json
{
  "gate_interlocks": {
    "enabled": true,
    "gate_commands_count": 0,
    "gate_open_latency_ms": 0,
    "gate_travel_time_ms": 0,
    "wrong_gate_open_events": 0,
    "gate_timeout_faults": 0,
    "gate_item_contact_events": 0
  }
}
```

Each event should be logged in `events.csv`:

```text
gate_commanded
gate_opening
gate_open
gate_closing
gate_closed
gate_timeout
wrong_gate_blocked
exit_verified
route_reset
```

## Minimal Test Matrix

### Stage 1: Single Item Per Route

```text
box_s -> B
box_l -> C
bottle -> D
```

Pass condition:

```text
3/3 delivered
0 unsafe errors
0 wrong_gate_open_events
0 gate_timeout_faults
containment_rate = 1.0
```

### Stage 2: All Official Items, Seed 42

```text
all manifest items
seed = 42
perception = rtx
drive = surface
gates = enabled
```

Pass condition:

```text
n_delivered == n_items
routing_accuracy >= 0.97
unsafe_errors = 0
containment_rate = 1.0
```

### Stage 3: Fault Cases

Run:

```text
low friction
high mass
off-center spawn
close spacing
delayed gate opening
gate stuck closed
gate stuck open
```

Expected behavior:

```text
fault is detected
next item is not released into an unsafe state
item is recovered or operator callout is logged
no silent wrong delivery
```

## Success Criteria

This idea is worth keeping if it improves safety and explainability without introducing too many jams.

Keep the gate layer if:

```text
smoke B/C/D = 3/3
seed42 all-items works
wrong-route risk decreases
gate logs are clean
videos clearly show selected route opening
faults fail safe
```

Reject or simplify the gate layer if:

```text
gate geometry causes frequent jams
timing becomes fragile
the ARB deck alone is more reliable
the visual complexity does not improve scoring evidence
```

## Recommended Implementation Order

1. Add visible non-colliding gate placeholders first.
2. Add animated gate state without collision.
3. Add collision only in closed state.
4. Add route scheduler.
5. Add exit verification sensors.
6. Add logs and metrics.
7. Run 3-item smoke.
8. Run all-items seed42.
9. Run fault cases.

This avoids debugging geometry, control, and collision all at the same time.

## Final Evidence Shot List

If the gate experiment works, record:

```text
1. route B: B gate opens, C/D stay closed, item goes to sorter
2. route C: C gate opens, item diverts into oversize cage
3. route D: D gate opens, item diverts into repack cage
4. wrong route blocked: non-selected gate visibly closed
5. fault recovery: gate timeout or jam -> safe stop/callout
```

The visual message should be:

```text
software decision + physical interlock + actuator movement + verification
```

That is a strong industrial story.

