# Mechanism-credibility correction — proof pack (2026-07-14)

Owner rejected the build with three screenshots: (A) the tilting tray
clipping dark structural geometry, (B) unexplained colored pieces
inside/behind the roll cages, (C) a post + beam standing inside a chute
mouth. This pack is the required proof for the correction. Every item
below maps to the owner's checklist.

## Root causes (exact prims / sources)

| Defect | Prim(s) | Source | Root cause |
|---|---|---|---|
| Tray through chassis | `/World/sorter/car*/body`, `band_e/w`, `bevel_e/w`, `actuator`, `actuator_rod`, `bearing_e/w`, `pivot_shaft` | `isaac/scene_usd.py build_carrier` | Decorative full-width slab + flank hardware inside the swept sector; the tilt pivot is 34 mm above the slab, so any body wider than ±43 mm is sliced from ~6.3° tilt. PhysX silent: jointed bodies never self-collide, and all of it was visual-only (`collide=False`) — invisible to `audit_clearance.py`, which only traverses `CollisionAPI` prims. |
| Posts in chute mouth | `/World/statics/st{C,D,B,REVIEW}_post±1`, `st*_beam` | `isaac/scene_usd.py build_station` | Portal posts at ±0.30 stood inside the 0.70 m chute width (B: inside the 0.62 m incline belt), planted through the sliding surface. |
| Colored bars in cages | `/World/dressing/chute_sign_*`, `ozon_band`/`ozon_kick`, `st*_lamp` on the bad portals, soft-route-colored rails/cheeks | `isaac/dressing.py`, `scene_usd.py build_chute` | Duplicative colored signage on sticks in the arm aisle; brand strips running continuously ACROSS the four discharge cutouts (floating bars); route-colored functional hardware reading as plastic. |
| (found by audit) end-module clips | `train_end_e_cap/wheel/spoke*/axle` | `build_sorter_train` | Wheel assembly inside the carrier over-run strip; cap below the tilted-lip trace (REVIEW residual tilt reaches the module). |
| (found by audit) return-leg grazes | `chuteC` mouth plate, `chsh*/chtrim*` under-shells, `bnose_*`, `cheekCn`, `drumCn` | `build_chute`, `dressing.py` | Return trays passed +1.5 mm under the C-mouth plate underside (kinematic-static pairs raise no PhysX contacts); under-shell trims hung into the return band; B-connector tail hardware inside the fault envelope; catch pan 11.8 mm inside the ±46° joint-limit envelope; B east skirt in the REVIEW slide path. |

## Fixes (all in commit `8af8389`)

Open-frame trunnion carrier (spine → saddles → r8 shaft → tray knuckles,
slung tilt-drive gearbox/motor, chain link+stem — continuous load path,
every part placed from the swept-envelope math at the ±46° joint limit);
portals ±0.44 with base plates (B+REVIEW share one two-lane gantry);
end modules deepened, wheel assemblies moved beyond the over-run, east
cap raised above the tilted-lip limit trace; B-connector east skirt
deleted / tail drum tucked / legs under-belt / apron raised; chute
under-shells inset 150 mm with flush trims; `return_z` 0.26→0.24;
catch pan 0.39/0.18; chute signs removed; brand band segmented to the
skirt panels; rails/cheeks neutral steel; lip chamfer strips 3.5 mm
(chain-pitch gap restored to 10 mm).

## Verification (files here / on server)

- `../audits/sweep_before.txt` — **64 violations + 15 corridor hits**
  (worst −25.8 mm; the authoritative before-list with prim paths, poses,
  depths).
- `../audits/sweep_after.txt|json` — **0 violations, 0 corridor hits.**
  All remaining close pairs are documented design interfaces at positive
  clearance: mouth labyrinth 6.1–7.1 mm, joint-limit chassis pairs
  8.1–8.9 mm (gate 8 mm at the fault limit), chain-pitch 10.0 mm.
  Audit = `isaac/audit_visual_sweep.py`: tray posed at 0/25/50/75/100 %
  tilt both directions + ±46° joint limit at every station (clamped to
  the physical run), return-leg poses, adjacent carrier, and item
  fall-corridor occupancy — oriented min-distance vs EVERY prim,
  visual and collider alike (Cube/Cylinder/Mesh).
- `../audits/clearance_after.json` — collider + arm audit re-run:
  **tray violations 0, arm violations 0** (PASS).
- `../audits/gate_seed42_postfix.json` — repeated nominal run after the
  fixes: **11/11 delivered, routing 11 ok, unsafe 0, floor 0,
  classification 1.0, containment 1.0, direct velocity writes 0** —
  physics/metrics unchanged, no scripted item motion.

## Renders (identical angles)

- `ba_tilt_side.png` — BEFORE|AFTER at the owner's screenshot-2 angle
  (same camera `mech_c_side`, seed 42, items box_l+pen, same frame
  index). `proof_tilt_side.mp4` = side view of the FULL tilt + return
  cycle.
- `ba_decktop.png` — BEFORE|AFTER top-down over the chute mouths
  (screenshot-1 angle). `proof_decktop.mp4` = helmet/bottle/plate run.
- `ba_cage.png` — BEFORE|AFTER cage/arm quarter (screenshot-3 angle).
  `proof_cage.mp4` = full curated run on that camera.
- `proof_pouf.mp4` — largest supported item (489 mm pouf) carried and
  discharged to C OVERSIZE: delivered 1/1, contained 1.0, floor 0,
  8.8 s (see `after_pouf_263.png` for the discharge moment).

Collision-debug equivalent: the sweep audit's pair tables ARE the
numeric collision-debug output (min clearance per prim pair per pose);
`sweep_after.json` carries the full machine-readable set.
