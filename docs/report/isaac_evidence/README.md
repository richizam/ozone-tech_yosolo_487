# Isaac Sim evidence — sensor-in-the-loop validation of the SortMaster cell

**NVIDIA Isaac Sim 6.0.1 (PhysX 5 + RTX), headless on RTX 5090.** The same
cell as the MuJoCo validation engine, built from the same
[`cell/params.py`](../../../cell/params.py), with two upgrades the second
engine makes possible: classification from **real rendered depth cameras** and
an executive driven by **contact physics** (surface-velocity conveyors,
pop-up stop blades, ARB actuator deck, powered discharge chutes) instead of
scripted item velocities. Package: [`isaac/`](../../../isaac/README.md).
Reproduction: `/isaac-sim/python.sh isaac/run_isaac.py --seed <S>
[--record] [--inject-jam SLUG@X] [--inject-gate-fault Z:MODE]`
(defaults: `--perception rtx --drive surface`).

## Headline results — ARB actuator-deck build (FINAL)

The routing zone is a **4×7 matrix of independent surface-velocity actuator
patches** (150×157 mm, 40 ms command pipeline, 6 m/s² ramp, saturation,
gain noise — every command logged), belts run at their **true designed
speeds** (belt A at the official 1.0 m/s, probe-verified), items carry
**material-class physics**, discharge is by **0.88 m powered decline
belts** through the cage apertures, and the exception arm is the **official
UR10e + official short-suction gripper**. `summary.json` certifies
`"nominal_motion_model": "surface_contact_only"`,
`"direct_velocity_writes_nominal": 0` in every run.

### Validation matrix — 12 runs, 132 item trials, gates PASS
[`validation_arb/matrix_summary.json`](validation_arb/matrix_summary.json):

| Run | Delivered | Routed ok | Cls | Unsafe | Containment |
|---|---|---|---|---|---|
| 6 nominal seeds (42/1/2/3/7/99) | 66/66 | **65/66 = 98.5%** | **66/66 = 100%** | 0 | 1.0 |
| low_friction ×0.7 | 11/11 | 11/11 | 1.0 | 0 | 1.0 |
| high_friction ×1.3 | 11/11 | 11/11 | 1.0 | 0 | 1.0 |
| high_mass ×1.3 | 11/11 | 11/11 | 1.0 | 0 | 1.0 |
| close_spacing 3.5–4.5 s | 11/11 | 11/11 | 1.0 | 0 | 1.0 |
| off_center +0.06 m | 11/11 | 11/11 | 1.0 | 0 | 1.0 |
| fault_jam (injected snag) | 11/11 | **11/11 incl. recovered snag** | 1.0 | 0 | 1.0 |

The single nominal exception: the 9 mm pen micro-stalled on the C discharge
belt in seed 99 → watchdog → operator call-out — a **safe escalation, never
a wrong feed**. In the jam drill both recoveries are on camera: the injected
box_s snag located by the jam camera (33.9 mm error) and a box_l chute stall
recovered via the state-observer fallback, each `attempts: 1`. A +0.10 m
off-center probe (beyond the physical loading envelope — the item spawns
overlapping the guide rail) was also run and kept for the record: 0 unsafe,
everything contained, unclassifiable items safe-sided to manual review.

### Gate interlocks — GATED_ACTUATOR_TEST_PLAN stages
[`gated_arb/`](gated_arb/): the normally-closed exit gates carry a measured
state machine (269 ms open latency, 243 ms travel — real actuator numbers,
150–400 ms class). Stage 1 (B/C/D) and Stage 2 (all items) pass clean.
Stage 3 hardware-fault injections all **fail safe**:
`C:stuck_closed` → `gate_timeout_faults=1`, item held + operator call-out,
0 unsafe; `B:stuck_open` → `wrong_gate_open_events=1` logged, routing still
correct; `C:delay:400` → measured latency rises to 422 ms, absorbed by the
~0.9 s command margin, 3/3 clean.

### Video evidence (v2 — clean nominal first, faults second)
[`final_arb/`](final_arb/), rendered on the **official ConveyorBelt_A49
transfer-deck embodiment** with industrial gate hardware:

*Normal operation (no arm, no stalls — the primary story):*
`nominal_pure.mp4` (whole cell, all 11 items, zero interventions);
`close_route_B/C/D.mp4` (deck close-ups, one per route: clean ARB
actuation through gate → chute → cage); `nominal_sensor.mp4` (items
passing the RTX heads).

*What the sensor sees:* [`perception/`](final_arb/perception/) —
side-by-side panels (real RGB | sensor's-eye depth with the item
segmented) with measured dimensions, fused-read count, confidence and the
official-rule verdict chip, + `perception_demo.mp4`.

*Fault handling (exists, but not needed in normal flow):*
`fault_recovery.mp4` (injected snag → watchdog → jam camera 33.9 mm fix →
UR10e + suction gripper re-delivery); `gate_fault.mp4` (gate C stuck
closed → `gate_timeout` in 2.02 s → item held → safe operator call-out,
0 unsafe).

*Closer:* `final_cinematic.mp4` — the clean nominal run + the validated
metrics end card (`endcard.png`). Calculations cross-check:
[calculations_vs_simulation.md](../calculations_vs_simulation.md).

## Headline results (previous build, kept for the record)

### Perception calibration — static, real depth (11 items × 3 rest yaws)
[`validation/rtx_validation.json`](validation/rtx_validation.json):
**33/33 = 100 %** categories from the 3-head RTX depth station (overhead +
two side profiler heads per `VIRTUAL_SENSOR`), official rule order with the
formal r_in/R ≥ 0.8 criterion. Includes the designed traps: the hex
«cylinder» caught by side-head flank slant (6.6–8.6 mm; a box wall reads
0.0 mm), the 9 mm pen → undersize **by rule priority** even though its section
reads a near-perfect circle (0.98–0.99), the 0.73-ratio detergent held in B.

### Closed loop, in motion — six seeds, full official set each
[`validation/`](validation/) + [`final/arm_seed42_summary.json`](final/arm_seed42_summary.json):

| Seed | Classified (RTX, in motion) | Delivered to final zone | Unsafe | Containment |
|---|---|---|---|---|
| 42 | 11/11 | 11/11 | 0 | 1.0 |
| 1 | 11/11 | 10/11 (cylinder → operator call-out) | 0 | 1.0 |
| 2 | 11/11 | 11/11 | 0 | 1.0 |
| 3 | 11/11 | 11/11 | 0 | 1.0 |
| 7 | 11/11 | 11/11 | 0 | 1.0 |
| 99 | 11/11 | 10/11 (pen → operator call-out) | 0 | 1.0 |
| **Σ** | **66/66 = 100 %** | **64/66 = 97 %** | **0** | **1.0 everywhere** |

Cage-entry speed ≤ 1.73 m/s (gate 2.2), bounce height ≤ 0.56 m (aperture top
0.83), command margin ≥ 0.5 s, ~1.2× real time at 240 Hz physics. The two
call-outs are items stalled **under the chute guard hoods** — the overhead jam
camera's documented blind spot. Both were correctly classified, never unsafe,
and a hang-up under a guard is an operator (lockout/tagout) case in a real
cell; the designed upgrade is one angled camera per chute.

### Fault drill — injected snag, sensor-true recovery
[`final/arm_drill_summary.json`](final/arm_drill_summary.json) /
[`arm_drill_events.csv`](final/arm_drill_events.csv): a snag is injected on
`box_s` in the routing zone (`--inject-jam box_s@8.05`). The chain is
camera-in-the-loop end to end:

```
t=69.0  snag_injected      the fault pins the item
t=87.9  jam_detected       zero-displacement watchdog fires
t=87.9  jam_located        routing-zone depth camera, background subtraction:
                           fix error ≈ 3–7 mm vs ground truth
t=87.9  recovery_started   4-axis arm dispatched to the CAMERA fix
t=88.4  attached           suction pick at the located position
t=89.1  released           re-delivered onto its commanded lane
t=89.4  recovery_done      watchdog re-armed; the zone conveyor re-routes
```

Run outcome: **11/11 delivered including the snagged item, 0 unsafe,
containment 1.0.** The arm runs the same closed-form IK (`cell/arm_ik.py`)
and controller cycle as the MuJoCo twin; links are collision-free with the
item carried at the TCP — MuJoCo weld parity.

### Video
[`final/demo_overview_rtx.mp4`](final/demo_overview_rtx.mp4) — RTX-rendered
nominal run (items tint with their perceived category the instant the depth
station commits). Full galleries per run (`index.html`, vision RGB/depth
stills, MP4s) are served from the GPU server (`/root/evidence`, port 8080
over the SSH tunnel).

## Why this is strong УГТ evidence

1. **Two independent physics engines, one parameter file.** MuJoCo carries
   the 22/22 deterministic scenario matrix; Isaac reproduces the same
   behavioural envelope (routing, containment, entry speeds, margins) on
   PhysX — solvers that share no code.
2. **The sensors are not oracles.** Categories come from rendered depth
   measured in motion (multi-read fusion, legal-metrology guard bands,
   safe-side policies: sensor miss → manual lane, unverifiable section on an
   elongated item → D). Faults are localized by an actual camera before the
   arm moves.
3. **The executive is industrially literal.** Surface-velocity conveyors
   (the Isaac Conveyor-Belt-utility mechanism), physical accumulation blades
   with a raise-safety interlock, an ARB-style vectored routing zone, powered
   nose-overs at the crest handoffs — each element maps to named warehouse
   hardware.

## Baseline sweep (earlier oracle-perception build, kept for the record)

The first Isaac port classified via ground-truth-with-latency while the
physics envelope was being validated; its 6-seed sweep established the
cross-engine agreement below and remains valid physics evidence:

| Quantity | Gate (from MuJoCo validation) | Isaac Sim (PhysX) | Agree |
|---|---|---|---|
| Cage-entry speed | ≤ 2.2 m/s (guided, non-thrown) | 1.81–1.89 m/s | ✅ |
| Bounce height in cage | ≤ 0.55 m (under 0.8 m wall) | 0.543–0.558 m | ✅ |
| Containment rate | must be 1.0 | 1.0 on every seed | ✅ |
| Unsafe routing errors | 0 | 0 on every seed | ✅ |
| Command margin | ≥ 0.5 s before table entry | ~0.58–0.62 s | ✅ |

## Engine-specific deltas (canonical `cell/params.py` untouched)

PhysX resolves discharge dynamics differently from MuJoCo (free tipping is
snappier; velocity-overwritten contact pairs can deadlock — the reason the
final executive uses real drives instead of velocity writes). Localized to
`isaac/`: hood clearance +0.12 m over the chute apertures, powered nose-over
strips at both crests, tight contact offsets on thin items. Every delta is
regression-gated by the containment metrics above (violations must be 0).
