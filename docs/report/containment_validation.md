# Containment validation — routed ≠ done

**Claim being proven:** after the cell routes an item to its zone, the item *stays inside the
correct container* — it does not bounce out, slide out, or fly over a wall. This is the
manipulation-quality evidence for the rubric lines «Качество манипуляции и работы с объектом»
(отсутствие избыточного брака при манипуляции) and «Корректность физической маршрутизации»
(полный цикл: сигнал → действие → перемещение → возврат механизма).

## 1. Containment by design

Every hop of the physical route is guided and bounded; nothing is thrown and nothing free-falls
more than ~0.13 m:

| Stage | Bounding hardware | Design rule |
|---|---|---|
| Belt A tail | funnel rails | items enter the table centered |
| Transfer table | edge rails + **normally-closed lift gates** on all three exits | a route command is the *only* thing that opens a path off the table; gate reopens/closes tracked by lamps |
| Chutes to C/D | single 32° slope, side guides, **anti-fly-out hood** over the cage aperture, **brake pad** at the bottom | μ_slope = 0.40 < tan 32° = 0.62 → *no static rest point exists anywhere on the slope* (recovery drops always slide through); hood is parallel to the flow → no catch faces |
| Roll cages 1200×800×800 | walls closed except a chute-sized aperture: flanks + under-slope skirt + hood + brow strip; full-floor friction mat | an item can enter only along the guided path and cannot leave it backwards or upwards |
| Zone B | rails on the powered connector, fixed sorter conveyor | delivery = crossing the sorter exit line |

Geometry invariants are locked by unit tests ([tests/test_containment_design.py](../../tests/test_containment_design.py)):
slope-cannot-hold-items, hood clearance > tallest item, gate stroke > max item + arm-carry height,
apertures fit chutes, brake pads end inside cages.

Three refinements found by the scenario suite (close_spacing / borderline / stress campaigns):

- **Soft-faced brake pads and cage mats** (`solref 0.012`): a rigid landing contact can eject a
  thin, feather-light item (the 9 mm / 50 g pen pogo-launched off a stiff pad edge). Rubber-faced
  pads absorb the landing — physically standard, and it removed the escape class entirely.
- **Thin items get stiff overdamped contact** (`solref 0.004/2`): under a 6 kg neighbor in a
  shared cage, a *soft* contact sags past the 9 mm body's half-thickness and the solver ejects it;
  stiff + overdamped holds the static load AND kills restitution on impact.
- **Cage fill-level call-out** (`CONTAIN.cage_full_fraction`, modeled fill sensor): items are
  admitted only while the accumulated footprint stays under 60% of the cage floor — beyond that a
  cage-swap call-out fires and the item goes to manual handling. Piling above the 0.8 m walls (how
  an overfull cage loses items under 1.3× overload) is prevented at the source.

## 2. The metric

Delivery into a cage **starts** the check, it does not end it. From aperture crossing to the end
of the run, every C/D item is tracked ([cell/run_sim.py](../../cell/run_sim.py), `_watch_containment`):

- `contained` — the item never left the cage envelope (walls + hooded aperture zone) and never
  exceeded z = 1.2 m. Any escape publishes `cell_event: containment_violation` with position.
- `v_entry` — speed crossing into the cage: evidence of *guided* transfer (≈ √(2·g·h) equivalence:
  2.1 m/s ≈ a 23 cm drop).
- `cage_settle_s` — time to rest (< 0.1 m/s held 0.5 s).
- `cage_max_z` — bounce headroom under the 0.8 m wall.

`summary.json` aggregates: `containment_rate`, `containment_violations`,
`cage_entry_speed_mean/max_mps`, `cage_settle_mean_s`. **The run exits non-zero if any item
escapes — containment failures break CI exactly like misroutes.**

## 3. Campaign results (2026-07-05, all 11 official items per run)

| Run | Routing | Classification | Containment | Violations | v_entry max, m/s | Settle mean, s | Throughput, items/h |
|---|---|---|---|---|---|---|---|
| oracle seed 42 | 1.0 | 1.0 | **1.0** | 0 | 1.77 | 1.43 | 507 |
| oracle seed 7 | 1.0 | 1.0 | **1.0** | 0 | 2.13 | 1.45 | 508 |
| oracle seed 11 | 1.0 | 1.0 | **1.0** | 0 | 1.87 | 1.27 | 508 |
| oracle seed 3 | 1.0 | 1.0 | **1.0** | 0 | 2.05 | 1.60 | 512 |
| oracle seed 99 | 1.0 | 1.0 | **1.0** | 0 | 1.68 | 1.26 | 513 |
| oracle seed 123 | 1.0 | 1.0 | **1.0** | 0 | 1.89 | 1.77 | 500 |
| **camera** seed 42 | 1.0 | 1.0 | **1.0** | 0 | 1.77 | 1.43 | 507 |
| **camera** seed 7 | 0.909¹ | 0.909¹ | **1.0** | 0 | 2.13 | 1.45 | 508 |
| **fault drill** (snag injection, camera) | 1.0 | 1.0 | **1.0** | 0 | 1.37 | 1.41 | 397 |
| arm-primary baseline (regression) | 1.0 | 1.0 | **1.0** | 0 | 0.58 | 2.00 | 502 |

¹ the known borderline: «Моющее средство» (oval, true ratio 0.73) measured ≥ 0.8 in one pose and
diverted B→D — a **conservative** error (never feeds a bad item to the sorter; unsafe errors = 0
everywhere), consistent with the documented 97.7% camera end-to-end / 100% executive campaign.

Determinism: repeated seed-42 runs reproduce every metric bit-for-bit (507.3 items/h, v_entry
1.77 m/s), so the jury can regenerate this table with single commands:

```bash
python -m cell.run_sim --scenario scenarios/base.yaml --perception oracle --seed 42   # …7 11 3 99 123
python -m cell.run_sim --scenario scenarios/base.yaml --perception camera --seed 42
python -m cell.run_sim --scenario scenarios/fault_jam.yaml
python -m cell.run_sim --scenario scenarios/base.yaml --executive arm --seed 42
```

## 4. Exception path is contained too

The fault drill injects a snag (item stops responding to the table drive at the routing zone).
The **zero-displacement watchdog** (10 s window, < 0.06 m displacement — queue creep is flow, not
a fault) detects it; the arm lifts the item and **places it back on its lane** (release 2 cm above
the table — «мягкость обращения»); the still-assigned route re-delivers it through the normal
gate → chute → cage path. Recovery success 1.0; the recovered helmet enters cage D at 1.37 m/s and
stays contained. Two failed recoveries escalate to operator call-out (`MANUAL`) — the cell never
dead-locks.

## 5. What the jury sees (video evidence)

`--record` captures any run to MP4 (`--camera overview | top_view | routing`). The presentation
layer makes the closed loop visible: items are tinted with their perceived category at the moment
of classification; the matching lane markings, chute flow arrows and destination beacon pulse in
the zone colour; gate lamps track the physical gate opening; the andon tower switches green →
amber → red (pulsing) across idle / routing / jam-recovery. All of it is non-colliding geometry
in a sensor-masked geom group: physics and perception are bit-identical with the layer on or off.
