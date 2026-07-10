# `isaac/` — SortMaster cell in NVIDIA Isaac Sim (PhysX 5 + RTX)

The high-fidelity digital twin of the Track-3 sorting cell: the same
`cell/params.py` geometry as the MuJoCo validation twin, rebuilt on the
Omniverse stack with **real rendered sensors in the loop** and a **physically
actuated tilt-tray sorter** as the executive. Everything nominal moves by
contact physics: surface-velocity belts carry the freight, a knife-edge nose
hands it to a moving tray, a revolute-joint tilt drive discharges it by
gravity, a 32° brake chute contains it. `direct_velocity_writes_nominal = 0`
is asserted by the validation gate. Consolidated results:
[docs/report/isaac_evidence/xbelt/](../docs/report/isaac_evidence/README.md).

## The executive: a linear tilt-tray sorter (why)

The previous executive (an Activated-Roller-Belt patch deck) diverts by
driving the item's footprint across powered rollers — which puts a hard
floor (~50–75 mm) on the smallest divertible product. The official rules
draw the sortable/undersize boundary at **10 mm**: an 11 mm cube is a legal,
sortable item that an ARB deck physically cannot handle. A **tilt-tray
sorter carries every item on its own tray** and discharges by gravity, so
the divert is **size-independent** — the 11 mm cube and the 489 mm pouf ride
and discharge identically. Tilt-tray/cross-belt is also what real
parcel-scale sortation (including Ozon's) runs, and «поворотные лотки» is one
of the mechanism classes named in the official problem brief.

```
belt A (1 m/s, FIXED) ──► RTX vision station (dual-range, in motion)
      │ verdict B/C/D committed BEFORE the escapement
      ▼
escapement (normally closed) ──► synchronized release to an EMPTY tray
      ▼ knife-edge nose, 60 mm drop, landing offset logged
tilt-tray train (9 carriers, 0.6 m pitch, 0.5 m/s, true-time return run)
      │ position-triggered tilt: latency 40 ms + 160°/s ramp + gain noise
      ├─ C station (x 7.45, south) ──► 32° chute ──► roll-cage C
      ├─ B station (x 8.40, north) ──► 16.6° powered incline ──► belt B (FIXED)
      ├─ D station (x 8.75, south) ──► 32° chute ──► roll-cage D
      └─ REVIEW  (x 9.05, north) ──► chute ──► manual-review pen
         (low confidence / double occupancy / discharge-miss fallback)
```

**Physical (PhysX):** belt transport (kinematic surface velocity — the
Conveyor Belt utility mechanism), the knife-edge handoff, tray carriage and
gravity discharge (dynamic tray on a revolute joint + angular position
drive), chute descent + brake pads, cage containment, the escapement and
pre-gate hold stop blades (prismatic joints + linear drives), the B incline.

**Modelled (stated honestly):** the traction chain is position-controlled —
kinematic carrier bodies follow the chain schedule every physics step (a
real sorter chain is speed-servo'd; the per-carrier "encoder" is the chain
coordinate). The tray, its joint, its drive and every freight interaction
remain force-based. The enclosed end modules "wrap" EMPTY carriers between
the top run and the under-deck return run (the return runs at true return
time, so carrier availability is never optimistic). The exception arm's
links are non-colliding kinematic visuals driven by the validated 4-axis IK
controller; the carried item is welded to the controller TCP (MuJoCo weld
parity). Soft items (sack, pouf) are rigid approximations with grip/damping
materials.

## Perception: dual-range RTX depth station

Four rendered RTX depth heads measure every item **in motion** over the
window x 5.65–6.05:

* **overhead metrology head** (1024×768 at x 5.55 — upstream of the window
  centre so the macro head's mount never enters its optical path);
* **two side profiler heads** (flank verticality: a lying cylinder/hex slants,
  a box wall is vertical — section evidence an overhead view cannot measure);
* **close-range MACRO head** (768×768 at z 1.42, GSD ≈ 0.4 mm): engages when
  the smallest overhead dimension is below 25 mm. Legal-metrology guard bands
  scale with the measuring head: undersize certification floor = 10 mm + 2×GSD
  → **16 mm** on the overhead head, **10.8 mm** on the macro head. An 11 mm
  cube honestly certifies as sortable (B); a 10 mm cube and the 9 mm pen stay
  conservatively C. The measuring volume is bounded above by the 0.5 m max
  inbound envelope, exactly like a real dimensioner's specified volume.

Fusion (`perception_rtx.fuse_reads`) applies the OFFICIAL rule order
(dimensions strictly before shape; r_in/R ≥ 0.8 circle criterion) with
frame-completeness gating (a truncated/stale read is rejected, never
evidence), dual-bound dimension fusion (25th percentile certifies undersize,
75th certifies oversize — each check uses the bound that cannot hide a
violation), and safe-side rules: sensor miss → D (manual review lane), an
elongated item whose section cannot be verified → D, persistent weak circle
evidence → D. Zero valid reads never route to B.

## What's here

```
scene_usd.py      USD stage from params: belts + knife nose, carrier train,
                  stations, chutes, cages, review pen, cameras, items
sorter.py         the executive controller: chain schedule, synchronized
                  escapement release, carrier tagging, position-triggered
                  tilt with latency/ramp/noise, review fallback, stuck-tilt
                  timeout, station lockout during arm recovery, evidence
run_isaac.py      the closed loop: spawn → perception → induction →
                  discharge → containment watch → metrics (summary.json,
                  events.csv, actuator_log.csv, reads_log.json)
perception_rtx.py dual-range RTX depth station + guard-banded rule fusion
jam_locator.py    background-subtraction jam localization (real camera)
arm.py            UR10e exception arm (chute snags → route-correct cage)
materials.py      OmniPBR world-triplanar grunge library
dressing.py       industrial presentation layer (lighting truss, signage,
                  cabinets, fences, route storytelling) — visuals only
asset_shells.py   official conveyor/robot/sensor assets as sanitized shells
cinematic.py      cinematic camera paths + item-follow camera
run_matrix.sh     the validation matrix (below)
showcase5.sh      the final video set
```

## Run it (inside the isaac-sim container)

```bash
# full official set, RTX perception + tilt-tray executive (defaults)
/isaac-sim/python.sh isaac/run_isaac.py --seed 42 \
  --out /workspace/sortmaster_out/run1

# with video frames + vision stills (demo run)
... --record --fps 30 --camera deck_front --depth-stills 3

# edge cases merged in (incl. the 11 mm cube headline proof)
... --manifest-extra
... --manifest-extra --items edge_cube11,edge_cube10,edge_rod9,pen

# fault drills
... --inject-jam box_l@2.62        # chute snag -> watchdog -> jam camera -> arm
... --inject-tray-fault D          # dead tilt actuator -> end-line call-out

# robustness sweeps
#   --friction-mult 0.7 | --mass-mult 1.3 | --spawn-offset-y 0.06
#   --spawn-gap 3.0,4.0 (close spacing) | --probe all (1 Hz diagnosis trace)
```

### Full validation matrix (host-side)

```bash
bash isaac/run_matrix.sh /root/sortmaster_out/xbelt_matrix
python3 tools/consolidate_isaac_matrix.py /root/sortmaster_out/xbelt_matrix
# gates: classification >= 0.98, nominal routing >= 0.97, unsafe = 0,
# containment = 1.0, direct velocity writes = 0, command margin > 0,
# 11 mm cube delivered to B. Non-zero exit on any failure.
```

## Outputs (per run)

* `summary.json` — classification block (accuracy, reads/item, misses),
  `sorter` block (carrier commands, tilt_time_ms, discharge_latency_ms,
  landing offsets, wraps, double-occupancy, review fallbacks, stuck-tilt
  flattens, end-line call-outs), routing/containment/cycle/margins/throughput,
  per-item rows with the full timing ladder.
* `events.csv` — every event of every item's life: `item_classified` →
  `escapement_release` → `induction_landed` (with offset) → `routing_cmd` →
  `tilt_cmd` → `tilt_full` → `discharge_confirmed` → `item_delivered`, plus
  jam/recovery/fault events.
* `actuator_log.csv` — every tilt command (t, carrier, station, target).
* `reads_log.json` — every RAW perception read behind every fused verdict
  (dims per read, head used, guard bands, circle features).
* `vision_rgb_*.png`, `vision_depth_*.png/.npy`, `vision_macro_*.png` —
  sensor's-eye stills (`tools/make_perception_panels.py` builds the
  side-by-side panels + perception_demo.mp4).

## Exit gate

The run exits non-zero on ANY bad physical outcome: an undelivered item, an
unsafe misroute into B, a floor drop, a containment escape, a negative
command margin, or a failed recovery. MANUAL/REVIEW deliveries are safe
designed outcomes (they already cost routing accuracy). Success means
success.

## Engine gotchas this build encodes (learned the hard way)

1. `PhysxSurfaceVelocityAPI.surfaceVelocity` is local-frame and scaled by the
   prim's xform scale — commanded speeds are pre-divided by half-extents.
2. `World(physics_dt = rendering_dt)` + manual `world.render()`; RTX
   annotators serve stale frames — reads flush the pipeline (10×) and the
   fusion rejects incomplete frames (an item never pauses in the window).
3. Runtime joint-frame rewrites (`physics:localPos0`) are NOT applied by
   omni.physx — the carrier legs are kinematic poses, never joint edits.
4. Referenced official assets own their xformOps and ship their own lights —
   every referenced subtree is sanitized (colliders off, lights zeroed).
5. PhysX skips sleeping bodies — items and trays run `sleepThreshold = 0`.
6. Fixed exposure (`/rtx/post/histogram/enabled = False`) or mixed-camera
   renders auto-brighten and clip dark surfaces.
7. A camera mount inside another head's frustum shadows its depth image —
   the overhead head sits upstream so the macro rig never occludes it (the
   sensor layout is part of the measuring instrument).
8. Only ONE Kit instance per GPU; kill leftover containers before launching.

## Design notes for the jury

* **Sweep corridor**: nothing collidable intrudes into the tilting tray
  edge's swept volume (y 2.69–3.31, down to z 0.44) — chute rails start
  0.12 m down-slope, the B-incline rails start at y 3.45, and the debris
  catch pan under the top run obeys the analytic bound
  `pan_z_top ≤ pivot_z − tan(tilt)·pan_y_half − margin` (a containment test
  locks it). Two field lessons live here: a rail tip at the chute mouth
  pinned a discharging tray at 13°, and a first-cut pan (0.33 half-width,
  top 0.515) stood across the discharge fall path — every heavy item
  toppled over it, but the 9 mm pen rolled along its face and dribbled off
  the chute's east edge to the floor, deterministically, in every perturbed
  run. Small rolling freight is the sharpest probe of discharge geometry.
* **Deep-drop discharge with a chamfered mouth**: the chutes start 50 mm
  under the tilted tray lip, so a sliding long box can never bridge
  tray→chute and yaw-wedge; a 59° mouth chamfer strip catches freight 30 mm
  below the lip — thin flat freight lands FLAT instead of tipping onto its
  rim over the drop (a rim-rolling Ø200 plate once cleared the stub rails),
  while ≥ 25 mm of free fall keeps the bridging cure. The 32° run is a
  polished slide sheet (μ 0.28, μ/tan32 = 0.45): even a 9 mm rod with
  convex-hull faceting slides decisively. Trays carry central-strip end
  fences (rounds stay contained in the dish centre, a 0.4 m box's corners
  clear the fence ends).
* **Every failure mode has a designed safe terminal**: misclassified → guard
  bands / safe-side D; unverifiable → REVIEW pen; wedged discharge →
  stuck-tilt flatten → next-station / REVIEW fallback; dead tilt actuator →
  end-line operator call-out; chute snag → jam camera + arm → route-correct
  cage (station locked out while the arm works); induction miss → the
  matched carrier and its follower are flagged SUSPECT and get a
  precautionary purge tilt into REVIEW at their next pass (a phantom load
  can never circumnavigate the loop untagged); anything unreachable →
  operator call-out. Nothing fails silently, and the run's exit code proves
  it.
* **Rolling smalls get their own release model**: a blade-held lying rod
  re-accelerates by rolling (contact line reaches belt speed while the
  centre is still slow), not by sliding lock — the escapement aims thin,
  round-sectioned, elongated freight with `a_roll = 0.42·a_slide`. Without
  it the pen landed ~0.2 m behind tray centre under close-spacing holds and
  crossed the association gate.
