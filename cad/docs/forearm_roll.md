# The forearm roll drive — the joint added to the SolidWorks arm (`forearm_roll`, 4th in the chain)

**Purpose:** spec and attachment of the belt-driven forearm roll (`forearm_roll`, between `elbow_pitch` and
`wrist_pitch`), the way `cycloidal_drive.md` documents the shoulder. Code: `lib/forearm/` (`params.py
RollEndParams` + `RollDriveParams` + `ForearmConfig.elbow_offset`, `layout.py stack_positions`, `roll.py`),
`assemblies/forearm_roll_drive.py`, `parts/joints/forearm_roll_*.py`; tests `tests/forearm/`.
Numbers below name the constants; the values live in `lib/forearm/params.py`.

## 0. Specifications at a glance
| item | value | where |
|---|---|---|
| joint | `forearm_roll`, revolute, `elbow_link → forearm_link`; axis along the forearm across the `wrist_pitch` axis, `WEB_DROP` from the wrist centre (§1), `ELBOW_ROLL_OFFSET` (the drive's centre distance) across the elbow axis - **an elbow offset: the two axes do not cross** -, `FOREARM_ROLL_AXIS_Z` along N from `j2_link`'s origin | `robot/frames.py`, `lib/placements.py SHIFTS` |
| range | ±`FOREARM_ROLL_LIMIT_DEG`, in software only: no printed hard stop, the shaft turns free all the way round (`tests/test_sweeps.py`), so the limit alone keeps the cables through the bore from winding; home sensor behind the shaft (`FOREARM_ROLL_HOME_DEG`: the home sensor row below) | `lib/params.py` |
| drive | NEMA 17 × 40 mm + MKS SERVO42D (a CAN id of its own) **on the elbow axis**, GT2 20T on the motor (hub out: its teeth toward the motor, its set screws toward the wrist), the 90T ring **integral to the pulley - the output**, **4.5 : 1**, a stock 2GT × 6 mm belt (`forearm_roll_belt`, `FOREARM_ROLL_BELT_LENGTH`) at the drive's design `centre_distance` (= the offset), its **tensioner** taking up the rest | `RollDriveParams`, `lib/belts.py` |
| torque | ≈ 1.1–1.8 N·m at the roll vs ≈ 0.7 N·m worst-case static load | §2 |
| bearings | 2× 6806-2RS (30 × 42 × 7, the belt joints' bearing), back to back in the frame's tower, **a 12 mm spacer (`lip`) between their outer rings - 19 mm apart centre to centre** (the elbow pattern's 2 mm lip had them 9 apart: each bearing now takes the forearm's bending moment over twice the span), bearing 2 right under the ring (seats Ø42.15, journals Ø30.2 on the pulley, Ø30.3 on the shaft) | `RollDriveParams bearing_*`, `lib/bearings.py` |
| pulley (rotor, PETG - the output) | the integral flanged 90T ring on a Ø44 core (under the ring only: its rear flange 1.5 over the frame's front face, the belt's plane set low by the forearm wall's thickness, `wall_x`); under it the Ø33 shoulder on down to bearing 2's inner ring; its hub through bearing 2 and on as a neck (`neck_dia`, under the bearing's bore: bearing 2 slides over it onto its journal) through the spacer to bearing 1's front face, where the shaft meets it; its front face the ring's, flat (no spigot: the pulley prints on it) - the forearm wall's flat elbow face sits on it; the Ø12.5 cable bore straight through and the rotor clamp's 4 plain holes end to end | `forearm_roll_pulley` |
| shaft (rotor, PETG) | a collar that holds bearing 1: its Ø30.25 journal through bearing 1 to the bearing's front face (the pulley's neck meets it there), the Ø33 shoulder, the Ø40 flange with the rotor clamp's 4 nuts (closed hex pockets open to its rear face, a flat toward the bore, `clamp_nut_pocket`); the Ø12.5 cable bore through both rotor pieces and the forearm wall (`bore`, the elbow 90T's) | `forearm_roll_shaft` |
| tensioner | a GT2 20T toothless idler (`gt2_idler_20t`) on the belt's back, on the run back from the ring `along` from the motor, pushed in till the stock belt fits; on a printed **spacer** (`roll_idler_spacer`, `roll_idler_spacer_h` tall: its riser on the frame's front face, its land on the idler's bearing inner ring - its own part, so nothing stands proud of the face the frame prints on) on an M5 (`roll_idler_screw`) down through the idler, the spacer and the plate's slot into an M5 nut (`roll_idler_nut`) in a channel under the face, closed at both ends - the nut in from the motor's pocket before the motor (`entry`: a recess in the plate's inner face; the pocket's far face set back to the channel over its upper part, `setback_h`, so the way is open from below all the way in) -: along N over `slot` tensions the belt - its slack end backs the idler out of the loop to put the belt on, its tight end stops short of the belt's far run -, the nut sliding with the screw; nearer than ~73 (`centre_distance`'s note) the idler would meet the ring's flange and the belt's far run | `RollIdlerParams`, `tests/forearm/test_roll_idler.py` |
| rotor clamp | 4× M4 × `clamp_screw_len` on Ø23 (`clamp_*`, at 45°) from counterbores in the forearm wall's wrist face through the wall and both pieces into the shaft's nuts: the wall, the pulley, both inner rings and the shaft one block, the screws centring the wall (the bearings' presses hold the rotor until the wall is on); modelled on the rotor (`roll_clamp_screws`, the heads `clamp_head_clear` under the wall's wrist face, the tips at the shaft's rear face; `roll_clamp_nuts`, a corner radial in their pockets, which hold them from turning) | `RollDriveParams clamp_*`, `tests/forearm/test_roll_clamp.py` |
| frame (stator, PETG) | ONE printed part round the motor, the elbow's output flange bolted to it (next row): seen along the elbow axis an **octagon about it** (`end_sides`, a flat on each axis `end_r` 42 off it - 5 mm outside the motor pocket's top corners -, round `j1_link`'s narrower faceted end, `ArmParams.end_r`) hulled up to the **tower** round the roll axis (`tower_y`) - straight sides, sharp corners, both faces' outline chamfered (`docs/design_language.md`), the rear line where the round end's ran (`rear_line_r`) -; seen along the roll axis the tower is the **forearm flange's octagon** (`roll.py tower_section`: its top corners cut to the diagonal flats, so the flange's facets run straight back through the frame), and the front face's two long edges are **faceted** at 45°, straight end to end (`front_facet` 4.9: as far as leaves bearing 2's seat 4 mm of wall where it opens through the face), from the underside (`block_x[0]`, under it the **spigot** that keys the elbow flange - an octagon about the elbow axis, `spigot_af` across flats, `spigot_h` tall -, the elbow 90T's 4 nuts in hex channels in it, open up into the motor's pocket) to the side (+N) face (`block_x[1]`), **closed** (`side_t`, as thick as the floor); the motor's **pocket** between them, open down through the octagon's bottom - the motor slides up into it -, a **groove** inside each wall at the board's side plugs (`plug_slots`), its front wall the **plate** (`pad_t`: round holes - no slide, the idler tensions the belt -, the motor's 4× M3 from the front face, `roll_motor_screws`, their heads flush in counterbores; the pilot hole, the shaft's slit down to the bottom), the plate's face the frame's one **front face**, nothing proud of it (the frame prints flat on it; the tensioner's spacer is its own part); **bearing 2's seat** round the roll axis, open through the front face (bearing 2's front face on it, 4 mm of wall inside the facets' top edges: nothing stands out of the facets round it), centred on the frame - the underside, the side face and the tower's top all `ROLL_FRAME_R` from the roll axis, the forearm wall's radius (`wall_od` / 2): the frame flush with the round parts that turn about the roll axis, its underside lifted off `j1_link`'s flat top, the elbow flange's post reaching down across the gap over `j1_link`'s bore -; the shaft's **bay** behind bearing 1, bored on through the frame's back (the rear line) - the rear face stepped there for the back panel, the bay's opening grown down below its bore, the magnets' pockets in the step | `forearm_roll_block` |
| elbow flange (stator, PETG) | the elbow's output flange, **its own part** so its round faces print as circles (printed on the stub's end; on the frame, printed front face down for its 6806 seats, they lay on their side): under the frame's underside about the elbow axis the **post** (`post_dia`, the SolidWorks `j3_coupler`'s journal carried up, its foot chamfered), the **shoulder** on the upper elbow 6806's inner ring, the **stub** through it and the lip to the elbow 90T, which bolts flat onto its end; the pin bore, blind from the stub's end; the 90T's 4× M4 clearance through it; the octagonal **recess** in its top over the frame's spigot (`spigot_fit` a side: it centres the flange and keys it against turning). The 90T's 4× M4 × `pulley_screw_len` run through the 90T, the flange and the spigot into the frame's nuts: the 90T, the flange and the frame one clamp | `elbow_flange` |
| back panel (stator, PETG) | **removable**, so the shaft and its clamp nuts are reached from behind: a plate in a step across the frame's rear face, the face's full width (less `side_clear`, its top corners cut to the tower's octagon), `t` deep, from the tower's top down to `bottom` - its outer face ON the rear line, so the elbow motor's stack (its gearbox nearest) passes it as it would the frame's face -, over the bay's bore and the opening grown down below it (`opening_bottom`, its corners `opening_r`, `opening_depth` deep); relieved `run_gap` off the shaft's rear face, where it passes over it; located by a U rim (`rim_*`) on its inner face in the opening's lower part; held by 4 magnet pairs (`roll_back_magnets`, small discs - `magnet_dia` - so the upper pair fits beside the bay's bore under the tower's octagon; `magnet_points`: one disc in a pocket in the step's floor, one in a pocket in its inner face, attracting); a **grill** through it (`roll.py back_grill`, `docs/design_language.md`): straight slots up the face (`grill_w` at `grill_pitch`, one on the roll axis' line) over the bay and the opening, clipped to a field `grill_clear` inside the rim - its bottom corners the 45° chords of the rim's rounded ones - and, over the bay, the tower's octagon carried in to the field's half width, so the slots' ends follow the panel's own top corners; the turning shaft shows through it; nothing passes through it (the forearm's cables take another route: `docs/open_issues.md`) | `forearm_roll_back_panel`, `BackPanelParams` |
| home sensor (stator + rotor) | a KY-003 hall module (`ky003_hall_sensor`) behind the shaft, in the bay under the back panel - nothing shows -, its chip `air` behind the shaft's rear face (the board's header, the tallest part, 1.02 over the chip), `chip_r` off the axis `home_deg` round from the magnet; on a printed **carrier** (`roll_home_carrier`): the board's plate (2× M2 from under it into nuts on the board: `roll_home_board_screws`, `roll_home_board_nuts`), a leg back out of the bay's bore and a strip down the opening's floor to its bottom, where a **tube** plugs into the frame's **wire hole** - it locates the carrier, and the sensor's lead runs down through it into the motor's pocket behind the MKS board's cover - and a notch takes the panel's rim: the closed panel keeps it on the floor, no screw into the frame; the **magnet** (`roll_home_magnet`, Ø6 × 3 N52, south pole out) pressed flush into a pocket in the shaft's rear face at +X, between the clamp's nuts (the rotor). The magnet stands over the chip at `FOREARM_ROLL_HOME_DEG`, its switch's zone running on past the lower end (homing turns toward − until it switches on) | `RollHomeParams`, `roll.py home_*`, `roll_home.py`, `tests/forearm/test_roll_home.py` |
| forearm interface | `j2_link`'s wall at 48…56 mm along the roll axis (`wall_x` −56…−48): a faceted flange on the roll axis through its whole thickness (flats `wall_od` across, over the ring's Ø59.19 flanges; its corners inside the roll tensioner's screw head's path as the forearm rolls), the web's underside level with its bottom flat (`WEB_DROP`); the web, wider than the flange (`WebParams.half_w`: as wide as its belt cover's end, `docs/design_language.md`), grows out of it through the flange's two lower diagonal facets carried on along it into its edge chamfer (`facet_len`, `lib/forearm/link.py corner_facets`); two gussets beside the wrist motor (`rib_*`), standing on those facets' top edge under the flange's side flats; its elbow face flat on the pulley's flat front face (no recess), the rotor's Ø12.5 bore, the rotor clamp's 4× M4 × `clamp_screw_len` on Ø23 from counterbores in its wrist face (`RollDriveParams clamp_*`: the heads `clamp_head_clear` under it) through the wall, the pulley and the shaft into the nuts in the shaft's flange - modelled on the rotor (`roll_clamp_screws`, `roll_clamp_nuts`), the screws centring the wall; the forearm turned `FOREARM_ROLL_TURN_DEG` about the roll axis from the capture, the wall on the ring turned with it (§1) | `RollEndParams`, `ForearmConfig.roll_turn_deg` |
| clearances held by tests | the frame's underside FACE_GAP or more above `j1_link`'s flat top, level with the upper 6806's top (`FACE_GAP`: the least gap a face the elbow turns may keep), the elbow flange's post over its bore only - its chamfered foot the frame's nearest point to `j1_link`, off the bore's rim -; the rotor 1 mm off the frame (`run_gap`: the pulley's core over the front face, the shaft in the bay), the shaft's flange 2 mm or more inside the rear line at any roll, and clear of the frame all the way round (no hard stop); folded elbow (to `ELBOW_PITCH_LIMITS_DEG`) and rolled forearm (±`FOREARM_ROLL_LIMIT_DEG`) < 1 mm³ against every neighbour; at the elbow's limits the forearm, rolled anywhere, ≥ 2 mm off `j1_link`; over the elbow's range the drive ≥ 1 mm off the elbow motor and its board on the upper arm's same side (how near at the lower limit: `docs/open_issues.md`); the stator behind the forearm wall's plane but the motor shaft's tip and the tensioner's screw head, which pass it and which the rolling forearm clears round the roll axis; journals 0.9–1.0 × their presses (the shaft's 99 mm³, the pulley's 66); the back panel flush with the rear line (nothing of it past it), `run_gap` off the shaft, on the step's floor, the magnets on their pockets' bottoms | `tests/forearm/test_roll_drive.py`, `tests/forearm/test_roll_back_panel.py` |
| link masses (URDF) | `elbow_link` carries the stator (the frame and its back panel + magnets, both bearings, the motor + board, the 20T, the belt, the tensioner), `forearm_link` the rotor (the pulley + the shaft, the clamp's screws + nuts) + the forearm and what it carries (§5); the masses are the `<inertial>` blocks (`tools/robot/derive.py`), their sum checked by `test_link_masses_add_up` | `robot/arm.urdf` |
| purchased per drive | what to order, with quantities: `./cadtool python tools/bom.py --module forearm_roll_drive` (its BUY list; every fastener of the drive is a part, so it has no `tools/bom.py EXTRAS` row); the elbow 90T's M4 screws and nuts are the arm's (`elbow_pulley_screws`, `elbow_pulley_nuts`) | `tools/bom.py` |
| printed per drive | frame, elbow flange, shaft, pulley, back panel, the tensioner's spacer (`./cadtool python tools/export_printables.py --parts forearm_roll_block elbow_flange forearm_roll_shaft forearm_roll_pulley forearm_roll_back_panel roll_idler_spacer`) | `print/` |

## 1. Why a roll, and where
The SolidWorks arm had five revolute joints — `base_yaw` and three parallel pitches (`shoulder_pitch`,
`elbow_pitch`, `wrist_pitch`) plus `wrist_roll` — so the tool's approach axis could never leave the vertical
plane through the base axis. `wrist_pitch` and `wrist_roll` already met at one point, the **wrist centre**
(`robot/frames.py WRIST_CENTRE`: `WRIST_CENTRE_ALONG_N` = 17 mm back along the pitch axis from the wrist_pitch
origin, 5 µm off both axes). A roll about the forearm through that point would make the last three axes concurrent -
a spherical wrist; the wrist roll drive (`docs/wrist_roll.md`) puts the wrist roll axis `WRIST_ROLL_OFFSET` across the
pitch axis instead, so an IK must be numeric. The roll crosses the pitch axis `WEB_DROP` from the wrist centre: the
forearm's web and the wrist on it dropped that far along the pitch axis toward the belt side
(`lib/forearm/params.py`), the roll axis stayed. The axis runs
along `ELBOW_TO_WRIST_INPLANE`, `FOREARM_ROLL_AXIS_Z` = 42 − 17 = 25 mm along N from `j2_link`'s origin — in
`j2_link`'s frame: y 0, z 25, along −X.

**The elbow offset.** The roll motor sits ON the elbow axis and the roll axis above it, the drive's design centre
distance away (`ForearmConfig.elbow_offset` = `RollDriveParams.centre_distance`, `ELBOW_ROLL_OFFSET`; the stock belt runs round its tensioner, §0): so the
forearm, the wrist and the gripper sit that far across the elbow axis - along `j2_link`'s +Y, up in the arm's swing
plane -, moved rigidly from the capture (`lib/placements.py SHIFTS`: `j2_link#1` and every record beyond it; the
elbow's own records stay) - and `ELBOW_INSET` along −N, toward the upper arm (`ForearmConfig.elbow_inset`): the frame's
underside 6.0 over `j1_link`'s flat top, where the SolidWorks coupler's flange and boss left 11.0, the elbow flange's post
that much shorter; with the wrist flange's `PITCH_INSET` (`docs/wrist_roll.md`) the gripper's roll axis lies in the plane
of the base_yaw axis at home. The arm gains an elbow offset (the roll axis and the elbow axis do not cross); in
exchange the forearm lifts back to `ELBOW_PITCH_LIMITS_DEG`'s −47° (the frame, an octagon about the elbow axis, sweeps
nothing of `j1_link` as the elbow turns; it comes nearest the elbow motor's gearbox there: `docs/open_issues.md`). In
`j2_link`'s frame the elbow axis runs along Z through y = −`elbow_offset`.

**The forearm is turned.** `j2_link` and everything beyond it - the wrist-pitch motor and its board, the wrist, the
gripper - are turned half a turn about the roll axis from the capture (`ForearmConfig.roll_turn_deg`,
`lib/placements.py TURNS`): the wrist-pitch motor sits on the upper arm's side of the roll axis, its belt on the
other, and at zero the gripper is rolled with it (the URDF's capture pose; `robot/AGENTS.md`). The drive keeps its
pose - the rotor too, so ±`FOREARM_ROLL_LIMIT_DEG` stays where it was - and the wall's 4 bolts,
at 0 / 90 / 180 / 270° about the roll axis, take the pulley as before. The drive's frame and its tests stay in
`j2_link`'s frame BEFORE the turn (the elbow side's view of the forearm: `lib/mounts.py ModuleMount`,
`tests/forearm/helpers.py in_drive_frame()`); "`j2_link`'s frame" below is that one.

**The pulley is the output.** The 90T ring is the front end of the rotor, outside the bearings, and the forearm
bolts straight onto it: the belt drives the ring, the ring the forearm. With both bearings pressed onto printed
journals, each needs its own journal entered from its own end - so the rotor splits at bearing 1: the pulley's hub
runs through bearing 2 and on as a neck - under the bearing's bore, so bearing 2 slides over it - to bearing 1's front
face, where the shaft's journal through bearing 1 meets it; the shoulders space the inner rings as the spacer (`lip`)
spaces the outer ones, and the rotor clamp holds the two pieces and both inner rings together. The spacer sets the
pair 19 mm apart, so each bearing takes the forearm's bending moment over twice the elbow pattern's span; the shaft
behind bearing 1 is only the collar that holds it. No end cap: the frame's tower is one piece, the spacer between the
bearings.

## 2. Layout (module frame)
Module frame (`lib/forearm/layout.py module_frame_in_host`, `lib/mounts.py MODULE_MOUNTS` on `j2_link#1`, in its
frame before the forearm's turn):
origin on the roll axis at the elbow axis' station, **+Z along the roll axis toward the wrist** (host −X),
**+X = host +Z** (N, away from the upper arm), **+Y = host +Y = up in the arm's swing plane** (the tower above the
motor); the **elbow axis runs along X at y = `y_elbow`** (−`elbow_offset`). Every station: `stack_positions()`.

**The frame is one printed part round the motor; the elbow's output flange is its own part under it.** Its outline seen along the
elbow axis (`roll.py profile`): an octagon about the elbow axis (`end_sides`), a flat on each axis on the upper arm's
round end (`end_r`), hulled up to the tower round the roll axis (`tower_y`, from `z_tower_rear` to the front face), cut
off at the front face and behind the rear line (`roll.py rear_line`: tangent to the `end_r` circle and to a
`rear_line_r` circle in the tower's rear-top corner - the line the round end's hull ran, which the elbow motor's stack
passes nearest at the elbow's lower limit; the octagon's corner behind it cut off) - extruded
along N from the underside (`block_x[0]`, 6.0 over `j1_link`'s flat top) to the side face (`block_x[1]`); seen along the
roll axis the tower is the forearm flange's octagon (`roll.py tower_section`, flats `tower_y`), and the front face's two
long edges are faceted straight end to end (`roll.py front_facets`, `front_facet`), bearing 2's seat open
through the face inside them; closed: the
motor's pocket opens down through the octagon's bottom instead, the shaft's bay through the frame's back (under the
removable back panel, flush with the rear line: §0). The
measured SolidWorks coupler (`lib/coupler/params.py CouplerParams`: Ø78 flange host z −10…0, Ø62 boss
−14…−10, Ø40 journal −15.3…−14, Ø30 stub −22…−15.3, Ø12.5 bore, 4× M4 at r 11 from the pulley) gives the elbow flange
(`elbow_flange`) under the underside about the elbow axis its journal - carried up to the underside as one post (`post_dia`) -, stub, bore
and bolt circle; its flange and boss, which turned in the Ø80 recess `j1_link`'s top no longer has, are not repeated.
The elbow 90T bolts up through it (through the elbow's 6806 pair, §4) into nuts in the frame's spigot, and
`j3_coupler#1` is **retired** (`lib/placements.py RETIRED`: the record stays, no table claims it; the wrist's
`j3_coupler#2` likewise: the wrist's flange is its own part too, `wrist_flange` on `wrist_link`, `docs/wrist_roll.md`).

| station | what | fixed by |
|---|---|---|
| the octagonal end, its flats `end_r` about (y `y_elbow`, z 0), x `block_x` −30…30 | the frame's outline round the elbow axis; the motor and its board inside it (the pocket's top corners 36.75 from the axis, 5 mm or more inside the flats) | `j1_link`'s round end (the flats round it, the arm's `ArmParams.end_r` inside); `ROLL_FRAME_R` (the underside 6.0 over `j1_link`'s flat top, FACE_GAP or more: `ELBOW_INSET`) |
| the elbow flange (`elbow_flange`), x −45…−30 about (y `y_elbow`, z 0), and the frame's spigot, −32…−30 | `post_dia` Ø40 **post** −35.3…−30 (`post_x`: the coupler's journal carried up from 0.7 above the upper 6806 to the underside, over `j1_link`'s Ø42 bore and never over its flat top; its foot chamfered `post_chamfer` at 45°), `step_dia` Ø33 −36…−35.3 (the shoulder on the upper 6806's inner ring), `stub_dia` Ø30 −45…−36 (9 long, as the wrist flange's; in the upper of the elbow's 6806-2RS pair and on through the lip, `j1_link`'s bore: `lib/mounts.py bearing_6806#3` / `#4`), the Ø12.5 `pin_bore_dia` (from the stub's end, blind at −37); the octagonal recess −32…−30 in its top over the frame's **spigot** (`spigot_af` 32 across flats, `spigot_h` 2 under the underside, `spigot_fit` 0.15 a side); the elbow 90T's **4× M4 × 35 at `pulley_bolt_r` 11, turned `pulley_bolt_deg` 45°** off the axes: `flange_bolt_dia` Ø4.2 through the flange (printed on its stub's end: upright, `M4_CLEAR_UPRIGHT`) and `pulley_bolt_dia` Ø4.4 through the spigot (on their side), then each nut's **hex channel** (`nut_af`, `M4_NUT_SIDE_PRESS_AF`: a press printed on its side, a flat toward the elbow axis) from its seat `nut_seat_x` −31 (1 of spigot between the nut and the flange) up through the floor into the motor's pocket | the measured coupler, its stub `PULLEY_SEAT_SHIFT` (3) longer: its end (host −20, the lip's lower face) sits on the re-seated pulley's face; `ELBOW_INSET` (the frame that much nearer `j1_link` than the capture put it); the recess's corners 2.5 inside the post, the nuts' outer flats 1.6 inside the spigot's; the nuts (−31…−27.8) and the screw tips (−26.4, 2 pitches past) 3.4 or more under the pocket's floor |
| `motor_pocket` y `y_motor` ± 22.25 (cut on down out through the octagon's bottom), z −29.25 … `z_motor_face` 26.85, from `x_cradle` −23 to `x_side` 23 | the roll motor ON the elbow axis, centred on the roll axis in X: its mounting face `z_motor_face`, the body −12.65…26.85, the MKS board −26.75…−12.65, turned `motor_spin_deg` 0° so its connector and the board's motor header point down into the opening (their short wire there); the pocket round the board's 43 square + `pocket_clear` (no slide: the idler tensions the belt), `pocket_rear` behind the board - room for the motor and its board to slide up from the bottom `motor_drop` back (the pilot boss under the plate) and go forward into place; the floor (`web_t`) 2 mm under the motor, the side wall (`side_t`) as far over it | the 20T's teeth level with the ring (`t20`), the 20T hub out, its far end `pulley_gap` over the front face: the motor `pad_t` behind the front face, the plate thick enough for the screws' counterbores; the shaft on past the 20T's set screws (`t20_set_screws`), its tip and the 20T's hub past the forearm wall's plane, which the rolling forearm clears round the roll axis |
| `plug_slots`, at the board's side headers, inside the walls | a **groove** inside each wall: turned with the motor, the MKS board's two side headers face ±X - the side wall and the floor -, nearer than their plugs' `MKS_PLUG_ROOM`; each groove `plug_clear` round its header's outline, from the pocket out to `plug_skin` inside the frame's face, from the header down out through the bottom and `motor_drop` behind it; each cable bent straight down at its plug, the straight room left (`side_plug_room`) short of `MKS_PLUG_ROOM` | the board's plugs stay in as the arm moves (`tests/test_sweeps.py test_the_mks_boards_keep_their_plug_room`, these two at `side_plug_room`: `docs/open_issues.md`); the motor slides up with its cables plugged |
| `z_motor_face` 26.85 … `z_front` 33.85 | the **plate** (`pad_t`), the pocket's front wall: round holes on the motor's 31 mm square (`motor_holes`), each screw's head (`roll_motor_screws`, 4× M3 × `ROLL_MOTOR_SCREW_LEN`) flush in a Ø5.7 counterbore from the front face down to `z_motor_screws` 30.85, the Ø22.3 pilot hole, the shaft's **slit** (`shaft_slit`) from the pilot hole down out through the bottom; its face is the frame's front face, its two long edges faceted `front_facet` at 45°; the 20T's far end `pulley_gap` over the face | the motor's face; 4 of plate under the heads, the counterbores `head_h` deep, the screws' thread in the motor `motor_holes.thread`; the facets as deep as bearing 2's seat leaves 4 of wall (`front_facet`'s note) |
| `z_front` 33.85 … `z_idler` 35.55 | the tensioner's printed **spacer** on the front face (`roll_idler_spacer`: its riser `riser_dia`, a thin pad under the belt's low plane, its land `land_dia` on top under the idler's inner ring), sliding with the idler over its `slot` along N; the M5's slot (a stadium along N) through the plate from the face, on past the screw's tip; `roof` under the face, the nut's **channel**, closed at both ends, the nut in from the pocket (`entry`, under it the pocket's far face set back to the channel: `roll_idler_setback`) | the idler level with the belt (`z_ring_mid`); the channel `nut_fit` round the nut; the set-back's lower part a 45° slope, so the frame prints without support |
| `z_tower_rear` 0.75 … `z_front` | the **tower** round the roll axis, up to `tower_y`; seen along the roll axis the forearm flange's octagon (`tower_section`: its top corners cut to the diagonal flats) | `ROLL_FRAME_R`, as far as the underside and the side face (the bay + 4.4, to the diagonal flats too), its corners sharp, the rear line (`rear_line_r`) from the end up to its top; `tower_rear` the outline's (it anchors the rear line), the bay bored through it |
| … `z_bearing_1` 7.85 | the shaft's **bay** (`bay_r` 25.6 round the roll axis), bored on through the frame's back, under the back panel: the shaft, its nuts | the shaft, bearing 1 on it, goes in along the axis from behind; its flange sweeps the top, 2 mm or more inside the rear line |
| `z_bearing_1` 7.85 … `z_lip` 14.85 | **bearing 1**'s seat Ø42.15, open into the bay (the bearing goes in from it) | |
| `z_lip` 14.85 … 26.85 | the lip - a 12 mm spacer (ID `lip_id` 37.6) - between the outer rings | the pair 19 apart |
| `z_bearing_2` 26.85 … `z_boss` 33.85 | **bearing 2**'s seat Ø42.15 round the roll axis, open through the front face: 4.0 of wall at the front facets' top edges (`front_facet`), more inside the frame's three sides and the tower's octagon; bearing 2's front face is the front face (`z_boss` = `z_front`), so nothing stands proud of the face the frame prints on; the pulley's core turns `run_gap` over it | the front face |

The rotor, the forearm wall back (`stack_positions`):

| station | what | fixed by |
|---|---|---|
| `z_wall` 44.75 … `z_wall_back` 56 | the forearm wall (`wall_x` −56…−44.75, 11.25 thick: its wrist face sets the forearm's place, its elbow face the belt's plane), round (Ø60, over the ring), its elbow face flat on the pulley's **flat front face** (no spigot, no recess - both parts print on them); the rotor clamp's counterbores in its wrist face, their floor `z_clamp_head` 51.75, the heads `clamp_head_clear` under the face; the Ø12.5 cable bore | the rolling forearm clears `j1_link`'s r 45 end; the stock M4 × 50 from the shaft's rear face |
| `z_ring_flange_1` 35.35 … 44.75 | the **90T ring** (teeth `z_ring` 36.55…43.55, `z_ring_mid` 40.05) on the pulley's Ø44 core, its front flange on the wall | the forearm wall's station |
| `z_boss` 33.85 … `z_ring_flange_1` 35.35 | the pulley's Ø33 shoulder, on bearing 2's inner ring, up to the ring (its rear flange 1.5 over the frame's front face and bearing 2) | the front face |
| `z_bearing_2` 26.85 … 33.85 | the pulley's hub (Ø30.2 journal) through bearing 2 | |
| `z_meet` 14.85 … 26.85 | the pulley's **neck** (`neck_dia` 29.9: bearing 2 slides over it) through the spacer | bearing 1's front face |
| `z_bearing_1` 7.85 … `z_meet` 14.85 | the shaft's journal (Ø30.25) through bearing 1, its end on the pulley's neck | bearing 1 |
| `z_shoulder_1` 6.85 … 7.85 | the shaft's Ø33 shoulder on bearing 1's inner ring | `run_gap` |
| `z_shaft_end` 1.75 … 6.85 | the shaft's Ø40 flange: the rotor clamp's 4 M4 nuts 3.15…6.35 (closed hex pockets open to its rear face, `clamp_nut_pocket`; the shoulder and the journal on over them) | the clamp's tips (`clamp_screw_len` from `z_clamp_head`) |

Ratio 90 / 20 = 4.5 (`FOREARM_ROLL_RATIO`) → ≈ 1.1–1.8 N·m at the roll from the 40 mm kit motor, against a worst-case
static load of ≈ 0.5 N·m (forearm horizontal and rolled to hold the wrist's longest lever level: ≈ 0.67 kg wrist +
gripper - `wrist_pitch_link`, `wrist_roll_link` and the jaws in `robot/arm.urdf` - + 0.2 kg payload at `tool0`). The
shoulder (`cycloidal_drive.md` §7) carries the drive too (its masses: `elbow_link`'s and `forearm_link`'s `<inertial>`
blocks in `robot/arm.urdf`). The elbow's torque path is the elbow 90T → 4× M4 through the elbow flange into captive nuts in the frame's spigot (the flange keyed on the spigot's octagon) → the frame → the
tower → the 6806 pair → the rotor → the forearm wall: the frame carries the forearm's load `ELBOW_ROLL_OFFSET` across
from the elbow flange, and the pair, 19 mm apart centre to centre, the forearm's bending moment (`docs/open_issues.md`:
check both on the print).

## 3. Fits and printing
PETG, the drive's rules (`cycloidal_drive.md` §6): the seats are the bearing OD + `seat_add` 0.15 (a printed hole
comes out undersize — print the fit gauges first, `tools/export_gauges.py`); the shaft's journal `journal_add` - it prints
at `LAYER_H`, so its journal is the 0.20 press measured on `j1_coupler_plate`'s test plate (`lib/bearings.py BEARING_6806_JOURNAL`) -, the pulley's `pulley_journal_dia` - it prints at `LAYER_H_FINE`, so its
journal is the 0.16 press measured on the printed 120T (`BEARING_6806_JOURNAL_FINE`). Tests
(`tests/forearm/test_roll_drive.py`) hold each journal's press fit at 0.9–1.0 × its nominal volume, 0 in the seats,
every other pair in the module below 1 mm³, the rotor `run_gap` off the frame, the two hubs face to face in the lip,
the elbow flange's stub end ON the elbow pulley's face (`tests/test_mounts.py test_bearing_stacks` holds the elbow's 6806
stack) and the flange on the frame's underside, its recess round the spigot (`tests/forearm/test_elbow_flange.py`). Print the frame front face down, flat on the bed: nothing stands proud of the face (`z_boss` = `z_front`: the
bearing boss flush with it, bearing 2's seat open through it; the tensioner's spacer a part of its own) - the tower's
seats print as vertical bores, the pocket's rim as walls; supports only under the spigot standing `spigot_h` off the
underside, inside the pocket under its back wall and under the plug grooves' tops (the bay prints as an open bore, the
rear face's step, the opening and the magnets' pockets open upward); the octagonal end and the front face's facets rise
off the bed at 45°, FDM's self-supporting angle - a printability check (`parts/AGENTS.md`) before the first print,
`docs/open_issues.md`),
the tensioner's spacer riser down (its land up, like the other idlers' spacers),
the elbow flange on the stub's end (the stub, the shoulder and the post print as vertical circles, the holes vertical; the shoulder and the post's foot step out 1.5 and 2 over the stub as short overhangs, the recess open upward), the pulley front face down (flat on the bed; the ring's grooves print vertically, the clamp's plain holes and the bore open to the bed),
the shaft rear face down (its nut pockets open to the bed), the back panel outer face down (its rim and magnet pockets
up; `pocket_depth` leaves `t` − `pocket_depth` of it over each magnet; the grill's slots plain walls up from the bed).

## 4. Assembly sequence
1. Four M4 nuts (`elbow_pulley_nuts`) into the frame's hex channels from the motor's pocket, reached through its open
   bottom (before the motor), down onto their seats - the channel's `nut_af` across flats (`M4_NUT_SIDE_PRESS_AF`, printed on its side) holds each; the tensioner's M5 nut
   (`roll_idler_nut`) up into the recess in the plate's inner face and on along it into its channel.
2. The elbow's 6806-2RS pair into `j1_link`'s bore, one each side of the lip; the elbow flange's stub into the upper
   one (its Ø33 shoulder on the inner ring, its end through the lip), the frame onto the flange (its spigot in the
   flange's recess, the holes lined up), the elbow 90T's hub into the lower one from below (its Ø34.76 ring on that
   inner ring, its end on the stub's end): 4× M4 × 35 (`elbow_pulley_screws`, the heads in the pulley's counterbores,
   `GT2_PULLEY_90T_HEAD_SEAT` under its face) up through the pulley's hub (its holes opened to `M4_CLEAR`), the flange
   and the spigot into the nuts (the tips 1.4 mm past them) bolt the pulley flat onto the stub, the flange onto the
   frame, and clamp both inner rings - the stub's 2 mm in the lip spaces them as the lip spaces the outer rings.
   The hex holds each nut, so the screws re-tighten from the pulley side at any time (with the motor out, a screw
   taken right out can drop its nut into the pocket).
3. Slide bearing 2 over the pulley's neck and press it onto its journal against the shoulder; press bearing 1 onto
   the shaft's journal against its shoulder.
4. The pulley + bearing 2 into the tower from the front (the outer ring in its seat, on the spacer, its front face
   flush with the frame's; the pulley's core over it); four M4 nuts (`roll_clamp_nuts`) pressed into the shaft's flange's hex pockets from its rear face and the home magnet (`roll_home_magnet`) pressed flush into its pocket, south pole out (the KY-003's LED lights), then the shaft + bearing 1
   into the tower from behind (the back panel off), through the bay's bore in the frame's back and along the axis, its journal's end onto
   the pulley's neck.
5. Turn the pulley until its four holes line up with the shaft's (the bearings' presses hold both; the clamp goes in
   with the forearm wall, step 8).
6. The motor, its board on, the short wire from the motor's socket to the board's motor header plugged and the board's
   two side cables plugged - each bent straight down at its plug -, up into the pocket from the bottom `motor_drop`
   back - the pilot boss passes under the plate, the shaft up the slit, the side plugs and their cables up their
   grooves - then forward onto the plate, the boss into the pilot hole (4× M3 - `roll_motor_screws` - from the front
   face, the heads into its counterbores; the connector down, toward the opening); the 20T on its shaft **hub out**
   (its teeth toward the motor), its far end on a `pulley_gap` gauge laid on the front face (its teeth then level with
   the ring), its set screws tightened on the shaft's flat - they sit just inside the shaft's tip -, the gauge out.
   The roll belt (`forearm_roll_belt`) round the ring and the 20T; the idler on its spacer (riser down on the front
   face) against the belt's back, the M5 down through both into the nut; push the idler along its slot until the belt is tight, then tighten the screw.
7. The forearm's cables do not pass through the bay (their route: `docs/open_issues.md`); the roll motor board's cables
   out through the open bottom, tied flat along the frame (the elbow motor's gearbox and motor pass under it at the elbow's upper
   range: `docs/open_issues.md`). The back panel: its 4 magnets glued into the pockets in its inner face and the 4 into
   the step's floor (`roll_back_magnets`; mark the poles first, so each pair attracts: the frame's all one pole out),
   each pressed to its pocket's bottom. The home sensor: the KY-003 onto its carrier (2× M2 up through the plate into nuts on
   the board), its lead plugged on and threaded down through the carrier's tube; the carrier onto the opening's floor, the
   tube into the wire hole, the lead pushed on down behind the MKS board's cover to its plugs. Then the panel into the
   step, its rim into the opening and the carrier's notch - the magnets pull it flush. To reach the shaft again, lift it
   off by a side edge (the step is open on both side faces); the carrier stays on its tube.
8. The forearm wall flat onto the pulley's front face BEFORE the wrist-pitch motor goes on: 4× M4 × `clamp_screw_len`
   (the rotor clamp, `roll_clamp_screws`) from the counterbores in the wall's wrist face through the wall, the pulley
   and the shaft into the nuts in the shaft's flange - snug all four with the wall centred on the pulley (the screws
   centre it), then tighten: the wall, the pulley, both inner rings and the shaft one block, the rotor turns whole.
   Taking the wall off unclamps the rotor. The wrist-pitch motor's body stands in front of all four heads: take it
   off to reach them later. There is no hard stop:
   turned by hand, keep the forearm inside ±`FOREARM_ROLL_LIMIT_DEG`, or the cables through the bore wind.

## 5. Attachment to the arm (`robot/`)
`elbow_link` = `gt2_pulley_90t#3` (the re-seated elbow 90T) + its screws and nuts + `forearm_roll_drive#1:stator` (the
elbow's driven side: the frame and the elbow flange under it, its back panel and magnets, both bearings, the motor + board on the elbow axis and
its 4× M3, the 20T, the belt and its tensioner, the home sensor on its carrier); `forearm_link` = `forearm_roll_drive#1:rotor` (the pulley and the shaft, the home magnet in it) + `j2_link`
and what it carries (the wrist-pitch motor + board, its 20T, the wrist belt and its tensioner, the wrist's 6806 pair:
`robot/frames.py LINKS`). `Joint("forearm_roll")` has Z along the forearm and X = N (its child's long direction IS
the axis); its origin sits `ELBOW_ROLL_OFFSET` across `elbow_link`'s frame (`robot/frames.py FOREARM_ROLL_ORIGIN` from
`j2_link#1`'s shifted origin, on the turn's axis; `ELBOW_ORIGIN` from the retired `j3_coupler#1`'s). Beyond it the axes
are turned with the forearm (`WRIST_PITCH_AXIS` the capture's −N): the forearm's turn changes `forearm_link`'s mesh and
inertial and the `wrist_pitch` origin, nothing further out. The forearm side of the interface is
`lib/forearm/params.py RollEndParams`: the wall at `wall_x` (−56…−48), round about the roll axis (`wall_od`), its flat
elbow face, the rotor clamp's Ø23 bolt circle (`RollDriveParams clamp_*`, through the wall into the shaft's nuts) and
the rotor's Ø12.5 cable bore, on the end of the straight web
(`WebParams.half_w`, as wide as the wall) with two gussets — `j2_link`'s DEFAULT build
(`lib/forearm/link.py roll_wall`, `wall_outline`, `straight_web`, `wall_gussets`). In `arm.py` the module is kept whole under the `elbow_link` group. The wrist-pitch motor
sits on round holes `WRIST_MOTOR_CENTRES` from the wrist (`J2_MOTOR_X`, `MotorHoleParams`), held by 4x M3 up through
the web (`wrist_motor_screws`, `J2_MOTOR_SCREW_LEN` - the elbow motor's screws and seat), their heads flush in
counterbores in its underside - fit them before the belt's idler, whose flange passes under one head -, nearer than the stock wrist
belt's (`WRIST_BELT_LENGTH`, a multipack length) own centre distance, its body `motor_wall_clear` or more off the wall
screws' heads; its connector faces sideways - away from the elbow axis, the forearm turned - with `plug_clearance` of
room beside it (`tests/forearm/test_roll_end.py`). The belt's tensioner (`IdlerParams`): an idler under the web on the
run leaving the motor's 20T, on its M5 axle up through a rib and the web's slot into a nut in a channel from the web's
top face, slid across the forearm between `slot_y`'s ends (`tests/forearm/test_wrist_idler.py`).

## 6. Not modelled / to confirm
`docs/open_issues.md`: the home sensor's switch zone and its lead's way to the MKS board, the forearm's cable route (not
through the bay); the back panel's magnets (their hold, their stray field at the MKS encoders and the home sensor); the
6806 seats' PETG fit; the frame's print orientation and its flex under the forearm's load, the
6806 pair's tilt play at the roll, the rotor clamp's thin walls and its nut pockets' fit; `t20_hub`,
the limit and the printed stop's strength; the elbow offset in a future IK; the room to lift the forearm further back
than `ELBOW_PITCH_LIMITS_DEG`'s lower limit; the roll motor's **CAN id** (`software/control/src/robotic_arm/config.py`
gives it 4, unconfirmed on the board).
