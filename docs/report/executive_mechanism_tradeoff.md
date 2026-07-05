# Executive mechanism trade-off: transfer table vs arm-primary

Both architectures are implemented in the same simulation (`--executive table|arm`)
and were measured on identical 8-seed camera-perception campaigns
(88 items each; seeds 42, 7, 123, 314, 2024, 55, 999, 4711).

## Measured comparison

| Metric | Arm-primary (baseline) | Transfer table + exception arm |
|---|---|---|
| End-to-end routing accuracy | 86/88 (97.7%) | 85/88 (96.6%) |
| Unsafe errors (C/D item into sorter) | **0** | **0** |
| Conservative errors (true B diverted) | 2 (detergent) | 3 (detergent) |
| Executive accuracy (did what perception said) | 100% | 100% |
| Arm interventions in nominal flow | 88 (every item) | **0** |
| Throughput under serial feed | 514.5 items/h | 516.8 items/h |
| Mean makespan, 11 items | 83.2 s | 80.2 s |
| Fault drill (injected snag) | n/a | jam detected +10 s → arm recovery → correct cage; unreachable snags escalate to operator call-out without deadlock |

Every error in both architectures is the same item — the detergent bottle
(max section ratio 0.728, the closest B item to the 0.8 circle threshold) —
always diverted in the safe direction (repack), always flagged
(`isolated_end_circle` / `policy_reroute_D`).

## Why the table is the primary architecture

1. **No universal-gripping assumption in the normal path.** The arm-primary
   design must grasp every material: soft sacks, smooth helmets, 9 mm pens,
   6 kg poufs. In simulation a suction weld makes this look easy; in reality
   it is the weakest claim of the whole cell. The table only needs items to
   rest on a driven surface.
2. **The arm becomes believable.** As an exception device it handles the rare
   jam at low duty cycle instead of being a 100%-duty universal picker.
3. **Throughput headroom.** Under the current serial escapement feed both
   architectures are gate-limited (~515 items/h). The table can pipeline
   (an item on the entry strip while another exits a lane) — the arm
   physically cannot overlap picks. Pipelining is a control-software change,
   no new hardware.
4. **Gentler handling.** Items are never lifted in the normal flow; lateral
   transfer + low-angle chutes into open-front roll containers.

## What the arm-primary baseline still proves

The preserved baseline (`git tag arm-primary-baseline`, `--executive arm`)
demonstrates full pick-and-place competence: live-AABB grasp planning,
carry-height planning, grasp verification, retry and abort logic. It remains
the fallback and the evidence that the team can do manipulation — the choice
of the table is an engineering decision, not a limitation.

## Detector benchmark: YOLO26n vs YOLO11n (tracking role)

Trained on 300 auto-labeled synthetic frames from the cell's own vision
station (12 epochs, 448 px, CPU; [docs/metrics/yolo_comparison.json](../metrics/yolo_comparison.json)):

| | YOLO26n | YOLO11n |
|---|---|---|
| Params | 2.38 M | 2.58 M |
| mAP50 (val) | 0.921 | **0.984** |
| mAP50-95 (val) | 0.672 | **0.725** |
| ONNX CPU latency (mean / p95) | **12.1** / 16.6 ms | 13.1 / **13.9** ms |
| NMS-free end-to-end export | **yes** | no |

**Choice: YOLO11n** for the tracking role at this training budget — noticeably
better detection quality; YOLO26's ~1 ms latency edge is irrelevant against
the ~100 ms geometric measurement budget. YOLO26's NMS-free export is genuinely
nicer to deploy and its head typically needs longer training — worth re-running
at the final training budget. Either way the detector only localizes/tracks:
the B/C/D category always comes from measured geometry, so the model choice
cannot affect classification correctness.

## Design details that came out of simulation (for the report)

- Chute contact friction must be surface-dominated (`priority=1`, polished
  steel µ≈0.12): items placed gently by the arm have no momentum and would
  otherwise freeze on a 33° slope against high-friction packaging.
- The C chute (oversize items!) must be sized for the largest products —
  the Ø489 mm pouf wedged in a 0.62 m chute during early runs.
- The vision window needs single-item discipline: a pre-gate hold line keeps
  followers upstream of the measurement volume; the sensing pipeline
  additionally isolates the tracked item's cluster along the belt axis.
- Exception-station placement is a reach trade-off: it covers the routing
  zone, all three lane exits and the D drop; upstream snags are operator
  territory (documented zone map in the safety concept).
