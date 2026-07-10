# Presentation audit — visual-only industrial-realism overhaul

Scope: `isaac/dressing.py`, `isaac/materials.py`, `isaac/scene_usd.py`
(visual/`collide=False` aspects only). No collider transform, joint,
PhysX attribute, sensor pose or controller value was touched; the frozen
physics build is bit-identical. All new geometry is non-colliding and stays
out of the RTX measuring corridor (x 5.30–6.20 / y 2.6–3.4 / z < 2.3) —
the only new prims inside that x/y footprint sit above z 2.30 (gantry
drop, connector boxes, upper conduit runs) or replace existing validated
hardware at the same station (side-profiler conduit at y 2.55 / 3.45,
outside the y band).

## 1. Floating-fragment / prim verdict table

| Prim tag (file) | Verdict | Reason |
|---|---|---|
| `trB/trC/trD/trREVIEW` trail chevrons (dressing.route_viz) | REMOVED | Jury: the station→destination arrow decals (chutes, B incline, belt-B run) read as scattered plastic litter on camera. `_chevron` helper deleted with them; route colour now lives only in signage plates and lamp housings. |
| zone arrow quads (dressing.route_viz) | CONFIRMED ABSENT | No arrow quads exist; the `zone_arrows` viz key itself is gone — `RouteVizRuntime` now tracks lamps only. |
| `cage{Z}_accent` top-rail route stripes (dressing.cages_detail) | REMOVED | Thin route-colour strips floating 18 mm over the cage top rails = off-surface colour fragments outside the signage/lamp language. |
| `arm_zone` hazard-hatch strips (dressing.industrial_context) | REMOVED | Seven short angled colour strips on open floor (position/yaw mismatch bug made them skew randomly) — textbook "scattered litter" read; the printed floor-zone label stays. |
| per-item route flags: `make_flag`/`flag_textures` (dressing.RouteVizRuntime) | REMOVED | Flag spawn calls were already retired; the dormant quad builder + textures are now deleted too. `drop_flag`/`update` remain as no-op-compatible entry points for the executive. |
| REVIEW pen colour bands (dressing.cages_detail) | REMOVED / RETONED | Pen keeps a SINGLE accent: magenta label plate (+ housed lamps). Blue OZON-bg wall label → magenta route bg; magenta top-rail stripe deleted; interior sheet panels retoned light-gray 0.58–0.62, bound matte rough 0.70 (translucent sheets blew out white). |
| `lampB/C/D/REVIEW` console lamps (dressing.route_viz) | KEEP-HOUSED | Recessed ~11 mm inside new dark bezel frames; prim creation call, returned path and `bind=False` unchanged. |
| `st{B,C,D,REVIEW}_lamp` station beacons (scene_usd.build_station) | KEEP-HOUSED | Were bare colour blocks sitting on the portal beams (the "colored blocks on beams"). Now recessed in a dark bezel channel (bottom tray + end caps + visor); lamp prim path/size unchanged. |
| `lane_b` "B SORTER" board (dressing.cages_detail) | KEEP-HOUSED | Free-floating quad at z 1.35 beside belt B (read as a loose blue/white fragment near the B incline). Now carried by a floor signpost + dark backing plate. |
| `ozon_wall` board + `brand_accent` strip (dressing.ozon_brand) | KEEP-HOUSED (flushed) | Floated 20 mm proud of the north wall face (y 6.27 vs wall face 6.29); moved flush (4–6 mm stand-off). |
| `camcable` thin hanging cylinders, r 4 mm (dressing.sensors_hw) | REMOVED-REPLACED | Thin hanging-line prims (read as floating wires/debris). Replaced by rigid `camconduit` box runs into the cable tray; macro and jam heads got bend-segment conduits. |
| `camled` status LEDs (dressing.sensors_hw) | KEPT-FUNCTIONAL | 6 mm LEDs sit on the housing faces (within housing envelope), not floating. |
| `hzA` black dashes on belt-A guides (dressing.conveyor_details) | KEPT-FUNCTIONAL | Yellow/black safety edge striping resting on the guide tops; already suppressed when official shells are present. |
| `dirA/dirB` white chevrons (dressing.conveyor_details) | KEPT-FUNCTIONAL | Painted flow-direction marks 0.5 mm above the WIDE belt A / fixed belt B faces (on-surface conveyor paint) — the only painted arrows left in the cell. `dirCn` incline chevrons were removed earlier (read as loose strips on the narrow slope). |
| `ozon_band`/`ozon_kick` brand strips (dressing.ozon_brand) | KEPT-FUNCTIONAL | Embedded in the train south-skirt face; the new under-deck enclosure panel was placed at y 2.585 so it cannot z-fight them. |
| `sorter_deck` "TILT-TRAY SORTER" label (dressing.route_viz) | KEPT-FUNCTIONAL | Sits on the south beam/skirt face. The new south e-stop was moved to the east beam segment (x 9.165) so it does not cover this label. |
| Old `tube{zone}` route-coloured cage tubes + slab base (dressing.cages_detail) | REMOVED | Whole-machine route colour violated the colour language and made cages read as coloured plastic boxes. Replaced by the galvanized roll-cage shell; route colour survives only as label plate + top-rail accent stripe. |
| Old routing-deck accent light (8.25, 3.0) (dressing.lighting_env) | REMOVED | Sat directly over the arm/cage corner; printed hot specular patches on the cage shells ("spotlight on the robot"). Deck now lit by high-bays + aisle soft lights. |
| 'ARB' / 'gate' / 'blades-field' remnants | CONFIRMED NONE | No ARB/puck geometry exists (`asset_shells.conveyor_shells` returns no pills/rollers; `viz["arb_pills"/"arb_rollers"]` stay `None` for API compatibility). `egate`/`hold2` blades are validated physics machines, not decor. Stale ARB docstring line removed. |
| Duplicate decals | CONFIRMED NONE | `P.SIGNS`/`P.FLOOR_DECALS` are rendered only by the MuJoCo twin (`cell/scene.py`), never in the Isaac scene; Isaac signage comes solely from `dressing`. |

## 2. New prim groups per brief item

| Item | Prim group (tags) | Where |
|---|---|---|
| A. Roll cages | `cage{Z}_rail` (22 mm galvanized tube rails, top/mid/bottom), `cage{Z}_mesh` (C/D: translucent panels; REVIEW: solid light-gray sheet, rough 0.70), `cage{Z}_wire` (wire grid), `cage{Z}_apfrm`/`cage{Z}_apsill` (aperture frame at the chute crossing), `cage{Z}_bframe`/`cage{Z}_fork`/`caster{Z}` (base frame + 4 casters), `cage{Z}_plate` (label backing plate; REVIEW label bg is the magenta route colour — the pen's single accent). Top-rail accent stripes REMOVED. | dressing.cages_detail |
| A. Roll cages | collider walls/flanks/header/skirt now `opacity=0.10`, dark tint; corner posts galvanized | scene_usd.build_cage |
| B. Carriers | per-carrier children of `car{i}`: `band_e/w` (gap-revealing dark end bands), `bevel_e/w` (45° chassis top-edge strips), `pivot_shaft` (brushed cylinder at the pivot line, r 14 mm, clears the ±38° plate sweep), `bearing_e/w` (pillow blocks), `actuator`+`actuator_rod` (south-flank tilt actuator, ≥0.10 m clear of the discharge fall line) | scene_usd.build_carrier |
| B. Trays | plates rebound as powder-coated mid-grey (rough 0.50); lips recoloured darker steel + bound (were gold); `lipch_e/w` 45° chamfer strips on each lip top (visual, riding with the tray) | scene_usd.build_carrier |
| B. Under-deck return guards | VISUAL `train_enc_{s,n}{k}_s{0-3}` slotted machine-guard skirts (y 2.585 / 3.415 — outside 2.62–3.38; z 0.10–0.38; x 6.79–9.37 = end-module face to end-module face). Purpose: close every see-through gap along the return run EXCEPT the four ±0.36 station discharge cutouts (C 7.45 / D 8.75 south, B 8.40 / REVIEW 9.05 north); each panel is four horizontal strips with 12 mm slots so the return leg is glimpsed like through a real chain guard. Dark powder-gray (0.30,0.32,0.35). Old solid panels + `train_encseam_*`/`train_encvent_*` detail removed with them. | scene_usd.build_sorter_train |
| B. Under-deck shadow plate | VISUAL `train_floor_plate` (x 6.6–9.6, y 2.62–3.38, top z 0.015, matte near-black 0.06/0.06/0.07 rough 0.95). Purpose: whatever still shows through slots and skirt lines reads as machine shadow over a steel base plate, not a see-through void to the warehouse floor. | scene_usd.build_sorter_train |
| B. End modules (wrap storytelling) | VISUAL rework of `train_end_{e,w}` (x 6.59 / 9.57, y 3.0): solid box shells replaced by open steel frames — `_wall` outer end plate + `_cap` top plate + `_gfrm_*` 25 mm powder-steel guard borders; SOUTH/NORTH faces are `_guard_{s,n}` FRAMED TRANSPARENT GUARDS (inset panel displayColor (0.7,0.75,0.8), displayOpacity 0.25). Inside each: `_wheel` spoked END WHEEL (r 0.17, axis y, z 0.43 — tangent to the top run above and the z 0.26 return level below) + `_spoke{0-2}` (6 spoke arms) + `_hub` + wall-to-wall `_axle`, dark steel. Purpose: cameras SEE how carriers wrap under the deck instead of trays appearing from nowhere. All collide=False. | scene_usd.build_sorter_train |
| C. B transfer | `bnose_apron`/`bnose_skirt` (transition plates under the incline mouth, below the belt plane and the tray-sweep envelope), `bdrive_gearbox`/`bdrive_motor`/`bdrive_shaft` + "B-LIFT DRIVE" label (head-drum drive at y 4.23), `bconn_leg`/`bconn_brace` (support legs at y 3.62/4.06 > 3.42). End drums already existed (`drumCn`). | dressing.b_transfer |
| D. Chute shells | `chsh{Z}` (brushed under-shell 10 mm below the collider underside, exact 32° slope), `chtrim{Z}` (edge trim bands), `chpadtrim{Z}` (brake-pad end trim below pad surface), chute signage "C OVERSIZE"/"D REPACK"/"REVIEW" on `chsignpost{Z}`, yaw +90° (text faces +x / the routing–deck_front camera line), route-colour background | dressing.chute_shells |
| E. HMI/andon | `hmi_bevel` (console top bevel), `hmi_door_seam`/`hmi_handle`, `lampbezel{Z}` recessed lamp frames, `stack_mast`/`stack_lens`×3 (red/amber/green)/`stack_cap`, `hmi_floorduct` (console→deck floor conduit), `estop_plate`/`estop_btn` ×3 (console flank, south beam x 9.165, north beam x 7.40), pinch-point labels on the C/D portal posts (black on safety yellow) | dressing.route_viz |
| F. Sensors | `gantry_base`/`gantry_brace` (portal feet + knee braces), `cam_overhead_drop` (rigid drop tube, z 2.35–2.47), `profbase_*`/`profcollar_*` (profiler post feet + post→housing collars), `{tag}_conn` connector boxes on every housing (macro's moved to `cam_macro_conn` at z 2.44, above the corridor ceiling), `camconduit`/`macro_conduit`/`jam_conduit` rigid runs, `jammast_up`/`jammast_arm` (jam mast tied into roof deck + truss) | dressing.camera_box / sensors_hw |
| G. Fragments | see table above (housings added, debris removed) | dressing / scene_usd |
| H. Materials | `PRESETS` + `preset()` in materials.py: brushed_steel, powder_steel_dark, galvanized, belt_rubber, safety_yellow_worn. Bound: trays powder, chutes brushed (slide, chamfer, fill), cage shells galvanized, belts matte rubber (conveyor bind rough 0.85), B-connector guards worn yellow. Bevels: tray lips (B), chassis top edges (B), console top (E). `roughness`/`metallic` now thread through `box`/`cyl` (dressing) and `add_box`/`add_cylinder`/`_bind_vis` (scene_usd). Physics `phys_material`s untouched. | materials.py + bindings |
| I. Lighting | dome fill 150→190 (+27%), high-bays 18 000→22 000 (+22%), routing-deck accent REMOVED (hot cage speculars / light over the arm), vision accent kept at 6 800, two new soft aisle RectLights (2.8×1.4, neutral, z 4.0, x 3.0/7.2, y 0.9). Exposure/rtx settings untouched. | dressing.lighting_env |

## 3. Brief items not done / deviations

* C. "nose plate between tray lip line and incline belt start": a literal
  bridge plate at/above the belt plane would enter the B tray-sweep /
  airborne landing zone (surface strip y 3.26–3.45 must stay clear), so the
  continuity is provided as an under-mouth apron + skirt strictly below the
  belt surface plane and below the full-tilt tray plane (top z 0.352 vs
  sweep plane 0.374 at y 3.30) — the visual void is closed without any
  geometry where items fly or slide.
* A. REVIEW pen aperture verticals are skipped (its 0.62 m aperture spans
  the whole 0.66 m wall — the shell corner posts already frame it); the
  galvanized sill tube is present.
* F. Side-profiler conduit runs reuse the exact validated `camcable` route
  (y 2.55/3.45, outside the corridor band) — thickened to 10 mm box
  conduit as ordered, no new inward protrusion toward the belt.
