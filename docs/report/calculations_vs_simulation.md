# Engineering Calculations vs. Simulation (Isaac Sim / PhysX)

The simulation is backed by first-principles calculations, and every
calculated value is cross-checked against measured Isaac results. All design
inputs come from `cell/params.py` (single source of truth). The executive is
a **linear tilt-tray sorter** (поворотные лотки — one of the mechanism
classes named in the official task brief): every item rides its own shallow-V
tray and is discharged by gravity at its station, which makes the divert
**size-independent** — an 11 mm cube is carried and discharged exactly like a
489 mm pouf. That is the engineering reason this executive replaced the
ARB/roller deck (roller diverters have a practical ~50–75 mm minimum product
footprint; see `docs/report/executive_mechanism_tradeoff.md`).

## 1. Inputs

| Quantity | Value | Source |
|---|---|---|
| Belt A speed | 1.0 m/s | official scheme (FIXED), `BELT_A.speed` |
| Carrier train speed | 0.5 m/s | `SORTER.v_mps` |
| Carrier pitch | 0.6 m (> 0.5 m max footprint) | `SORTER.pitch` |
| Carriers on the loop | 9 (top run 2.70 m + true-time return) | `SORTER.n_carriers` |
| Tray | 0.59 × 0.62 m, 2° V-dish, 30 mm end lips | `SORTER.tray_*` |
| Tray surface friction (pairs `min`) | μ ≤ 0.32 | `SORTER.tray_mu` |
| Discharge tilt | 38° | `SORTER.tilt_deg` |
| Tilt drive ramp | 160 °/s, latency 40 ms, gain noise 1 % | `SORTER.tilt_rate_dps/latency_s/noise_frac` |
| Induction drop | 60 mm (knife nose 0.70 → tray 0.64) | `BELT_A.nose_x`, `SORTER.tray_top` |
| Chute angle / friction | 32°, μ ≤ 0.40 (`min`), brake pad μ 0.45 (`max`) | `CHUTE` |
| B connector incline | 16.6° powered belt, 0.9 m/s | `B_CONNECT` |
| Item mass range | 0.05 – 6.0 kg | manifest (pen … box_l/pouf) |
| Jam watchdog timeout | 10 s | `JAM_TIMEOUT_S` |

## 2. Gravity discharge: tilt angle vs friction (the core guarantee)

A tilted tray discharges by gravity if `tan(θ) > μ`. The tray surface is a
smooth low-friction plate whose PhysX material pairs with `combine="min"`,
so **every** item pair resolves to μ ≤ 0.32 — including the soft sack whose
own μ is 0.95 (a real tilt tray is smooth ABS/steel for exactly this
reason):

```
slide onset:   θ_min = atan(0.32) = 17.7°   →  38° gives a 2.1× margin
slide accel:   a = g·(sin 38° − 0.32·cos 38°) = 9.81·(0.616 − 0.252) ≈ 3.57 m/s²
time to clear: t = √(2·0.31 / 3.57) ≈ 0.42 s   (half-width 0.31 m)
exit speed:    v_y ≈ 3.57 · 0.42 ≈ 1.5 m/s  (into the chute, guided by rails)
```

The 2° V-dish subtracts at most 2° from the effective angle on the uphill
half (36° still ≫ 17.7°) and prevents round items (bottle, plate) from
rolling off during carriage — the dish is why a PET bottle can ride a
low-friction tray at all.

Cross-check (measured): `tilt_time_ms` and `discharge_latency_ms` in
`sorter` summary; every C/D/B discharge lands inside its chute/connector
(containment 1.0, no floor drops) — see the validation matrix.

## 3. Induction: synchronized release onto a moving tray

The item is released by the escapement so that it rides off the knife-edge
nose and lands centred on its assigned tray:

```
gate → nose:   0.55 m at 1.0 m/s with re-acceleration ≈ 5 m/s²  → ≈ 0.65 s
free fall:     √(2·0.06/9.81) ≈ 0.11 s  → lands ≈ 0.11 m past the nose
landing slip:  Δv = 1.0 − 0.5 = 0.5 m/s, decel μ·g ≈ 3.1 m/s²
               slip distance = Δv²/(2a) ≈ 40 mm  ≪ tray half-length 295 mm
release rule:  drop the blade when an empty tray's centre will reach
               x_land = 7.01 m at the item's own predicted arrival time
               (tolerance ±50 ms → ±25 mm of tray offset)
```

The tray's 30 mm end lips bound the residual slip of rolling items: a bottle
arriving with 0.35–0.5 m/s of slip carries `v²·(3/4)/g ≈ 10–19 mm` of
climbing energy against a lip that needs ≈ 40 mm — it stays on the tray.

### 3.1 Rolling smalls: a different re-acceleration model

A blade-held LYING ROD (the 9 mm pen) does not re-accelerate at the sliding
lock μ·g: belt friction at the contact line both pushes it forward and spins
it up, so the contact point matches belt speed while the centre is still
slow — it leaves the friction-driven regime early and creeps up to belt
speed. Released with the sliding model it arrived ≈ 0.2 m behind tray
centre; at the twin's close-spacing stress this crossed the ±0.22 m
association gate and the freight rode an untagged tray around the loop.
Two-layer fix, both engines:

```
aim model:   thin (min dim < 50 mm), round-sectioned (mid/min < 1.6),
             elongated (max/min ≥ 2.5) freight released from a blade hold
             is aimed with a_roll = 0.42·a_slide  → lands ≈ −0.18 m,
             40+ mm inside the association gate
safety net:  induction miss ⇒ the matched carrier AND its follower are
             flagged SUSPECT ⇒ precautionary tilt into REVIEW at the next
             pass (empty tray: harmless flatten; loaded tray: the freight
             lands in the manual lane — never circumnavigates the loop)
```

Cross-check (measured): `landing_offset_mean_mm` / `landing_offset_max_mm`
per run (target: max well under ±150 mm, i.e. half the lip-to-lip span);
`suspect_purge_tilts` in the sorter block (0 in nominal runs — the net is
untriggered defense-in-depth).

## 4. Position-triggered discharge and command margin

The tilt command fires `trigger_lead_m = 0.28 m` upstream of the station so
the item (which keeps the carrier's 0.5 m/s while sliding) lands centred on
its chute:

```
cmd → slide onset:  latency 0.04 s + ramp to slide angle 17.7°/160°/s ≈ 0.11 s
onset → clear:      ≈ 0.42 s (from §2)
x drift during:     0.5 · (0.15 + 0.21) ≈ 0.18 m   (half the slide counted)
chute half-width:   0.31 m  → the landing is centred with margin
command margin:     the verdict is ready at the vision window exit + 80 ms;
                    the earliest tilt command fires ≥ (7.17 − 6.35)/0.5 +
                    handoff ≈ 2.3 s later  →  margin is structurally ≥ 2 s
```

Cross-check (measured): `command_margin_s.min` in every run must be > 0
(gated by the matrix consolidator; the run exits non-zero otherwise).

## 5. Chute descent and cage entry

The chutes start just under the tilted tray lip (z₀ 0.43 < lip 0.444) and
keep the validated 32° brake-chute design: μ_chute 0.40 < tan 32° = 0.625,
so nothing can rest statically on the slope; the high-friction brake pad
(`combine="max"`) then kills the residual speed inside the cage:

```
slope accel:  a = g·(sin 32° − 0.40·cos 32°) ≈ 1.87 m/s²
slope run:    0.51 m  → Δv² = 2·1.87·0.51 ≈ 1.9 m²/s²
entry speed:  √(1.5² + 1.9) ≈ 2.0 m/s at the pad, braked before the far wall
```

The cage aperture is crossed at z ≈ 0.24 (sill 0.20, header 0.83): lower and
slower than the previous 0.70-high discharge — containment is easier, and
the aperture flanks/header still close every fly-out window.

Cross-check (measured): `containment.cage_entry_speed_max_mps`,
`cage_max_z`, `containment_rate = 1.0` in every matrix run.

### 5.1 The discharge fall corridor and the catch-pan bound

Everything under the top run must clear the FULL-TILT tray plane
`z(y) = pivot_z − tan(38°)·|y − y₀|` and stay out of the fall corridor a
discharging item crosses between the lip (z 0.444) and the chute mouth
(z 0.395). The debris catch pan (catches sub-3 mm freight that arrives
under the escapement's skim gap) is sized against that bound:

```
pan edge:      |Δy| = 0.22  →  tray plane 0.616 − 0.781·0.22 = 0.444
pan top:       0.42  →  24 mm clear at FULL tilt (22 mm with +1% gain noise)
lip hang:      |Δy| = (0.62/2)·cos 38° = 0.244  →  pan edge 24 mm inboard
```

This bound was learned, not assumed: a first-cut pan (half-width 0.33, top
0.515) stood across the corridor. Heavy freight toppled over it unaffected —
only the 9 mm pen, rolling at tray-surface height, deflected along its face
and dribbled off the chute's east edge to the floor, in every perturbed run,
deterministically. `test_catch_pan_clears_the_tilt_sweep_and_fall_corridor`
now locks the analytic bound against any re-tuning.

Cross-check (measured): pen delivered to cage C with `floor_drops = 0` in
every matrix run of the fixed build; discharge-confirm latency back at the
nominal ~700 ms (the deflection had shown as a 1 200 ms outlier).

## 6. B connector incline: friction hold

Every B item is **non-round by rule** (round → D), so the 16.6° powered
incline (tan 16.6° = 0.30) holds each B item by friction with margin:

```
worst B pair:  detergent μ_d 0.38 with belt 0.72, combine average → 0.55
               0.55 > 0.30  →  holds with 1.8× margin (boxes: > 2×)
```

Cross-check (measured): route_B clips show detergent/lunchbox/box_s carried
up without slip; B deliveries at `y > 5.7` on the fixed belt B.

## 7. Throughput and cycle time

```
carrier-limited:  v/pitch = 0.5/0.6  → 0.83 carriers/s → 3000 items/h ceiling
vision-limited:   ~2.5 s per item (multi-read fusion in motion) → ~1400 items/h
demo pacing:      6–8 s spawn gaps → ~500 items/h (clean singulated evidence)
cycle time:       detect → deliver = vision (0.4 s) + belt run (0.6–0.9 s)
                  + tray ride (0.9–4.3 s by station) + chute/connector
                  (0.5–1.9 s) ≈ 3–8 s per item, fully pipelined
```

Cross-check (measured): `cycle_s.mean/p95/max` and
`throughput_items_per_h` per run.

## 8. Small-item certification (the 11 mm cube)

Dual-range DWS metrology: the overhead head (guard band 6 mm — undersize
certification floor 16 mm) hands small freight to the close-range macro head
(GSD 0.40 mm at the belt):

```
macro floor = 10 mm + 2·GSD = 10.8 mm
11 mm cube  → measured ≈ 11.0 ± 0.4 mm  ≥ 10.8  → certified sortable → B
10 mm cube  → 10.0 < 10.8               → undersize → C (guard band, safe side)
9 mm pen    → 9.0  < 10.8               → undersize → C
```

The guard band scales with the measuring head exactly as legal metrology
requires: a dimension within the sensor's uncertainty of the 10 mm limit can
never take the permissive branch.

Cross-check (measured): `reads_log.json` per read (`head: "macro"`,
`guards_mm`), the `edge_small` matrix run, and the `cube11_proof` field of
`matrix_summary.json`.

## 9. What is physical and what is modelled (honesty)

* **Physical (PhysX contact/actuation):** belt transport (surface-velocity
  kinematic belts), knife-edge handoff, tray carriage and gravity discharge
  (dynamic tray on a revolute joint + angular drive with latency/ramp/noise),
  chute descent, brake pads, cage containment, escapement/hold stop blades
  (prismatic joints + drives), the B incline connector.
* **Modelled simplifications (stated):** the traction chain is
  position-controlled (kinematic shuttles on the chain schedule — the tray,
  its joint and its drive are the force path to the freight); the enclosed
  end-module wraps teleport EMPTY carriers between the top and return legs
  (the return leg runs at true return time, so carrier availability is never
  optimistic); the exception arm's links are non-colliding kinematic visuals
  driven by the validated IK controller and the carried item is welded to
  the TCP (MuJoCo parity); the soft sack/pouf are rigid approximations with
  grip/damping materials.
* **Zero scripted freight motion:** `direct_velocity_writes_nominal = 0` is
  asserted by the matrix gate; the only direct writes touch machine parts
  (drive targets, blade targets, chain poses) or fault-injection pins.
