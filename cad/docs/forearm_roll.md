# The forearm roll drive — the joint added to the SolidWorks arm (`forearm_roll`, 4th in the chain)

**Purpose:** spec and attachment of the belt-driven forearm roll (`forearm_roll`, between `elbow_pitch` and
`wrist_pitch`), the way `cycloidal_drive.md` documents the shoulder. Code: `lib/forearm/` (`params.py
RollEndParams` + `RollDriveParams` + `ForearmConfig.elbow_offset`, `layout.py stack_positions`, `roll.py`),
`assemblies/forearm_roll_drive.py`, `parts/joints/forearm_roll_*.py`; tests `tests/forearm/`.
Numbers below name the constants; the values live in `lib/forearm/params.py`.

## 0. Specifications at a glance
| item | value | where |
|---|---|---|
| joint | `forearm_roll`, revolute, `elbow_link → forearm_link`; axis along the forearm through the wrist centre (on the `wrist_pitch` axis), `ELBOW_ROLL_OFFSET` (the drive's centre distance) across the elbow axis - **an elbow offset: the two axes do not cross** -, `FOREARM_ROLL_AXIS_Z` along N from `j2_link`'s origin | `robot/frames.py`, `lib/placements.py SHIFTS` |
| range | ±`FOREARM_ROLL_LIMIT_DEG`, 2° short of the printed hard stop (a lug on the shaft's flange, a post on the frame's tower, in the bay behind bearing 1 - straight-sided, `stop_deg_width` wide at their outer radius, so they meet before 180° − `stop_deg_width`, where `stop_deg_width`'s note says; `tests/test_sweeps.py` holds the stop between that and the limit); home sensor modelled (`ky003_hall_sensor`), not placed | `lib/params.py`, `RollDriveParams stop_*` |
| drive | NEMA 17 × 40 mm + MKS SERVO42D (a CAN id of its own) **on the elbow axis**, GT2 20T on the motor, the 90T ring **integral to the pulley - the output**, **4.5 : 1**, a stock 2GT × 6 mm belt (`forearm_roll_belt`, `FOREARM_ROLL_BELT_LENGTH`) at the drive's design `centre_distance` (= the offset), its **tensioner** taking up the rest | `RollDriveParams`, `lib/belts.py` |
| torque | ≈ 1.1–1.8 N·m at the roll vs ≈ 0.7 N·m worst-case static load | §2 |
| bearings | 2× 6806-2RS (30 × 42 × 7, the belt joints' bearing), **back to back on a 2 mm lip** in the frame's tower, bearing 2 right under the ring (seats Ø42.15, journals Ø30.3) | `RollDriveParams bearing_*`, `lib/bearings.py` |
| pulley (rotor, PETG - the output) | the integral flanged 90T ring on a Ø44 core, its rear flange turning `run_gap` over the frame's bearing boss; its hub through bearing 2 to the lip's middle, the Ø33 shoulder on bearing 2's inner ring; the Ø39.7 spigot in the forearm wall's recess - the wall bolts onto the ring's face (4× M3 on Ø31 into nuts in the core); the rotor clamp's 4× M3 heads sunk in its front face | `forearm_roll_pulley` |
| shaft (rotor, PETG) | a collar behind bearing 1: its hub through bearing 1 to the lip's middle (it meets the pulley's there), the Ø33 shoulder, the Ø40 flange with the rotor clamp's 4 nuts (pockets open to its rear face) and the stop lug; the Ø18 cable bore through both rotor pieces | `forearm_roll_shaft` |
| tensioner | a GT2 20T toothless idler (`gt2_idler_20t`) on the belt's back, on the run back from the ring `along` from the motor, pushed in till the stock belt fits; on a **riser** on the frame's front face (its land on the idler's bearing inner ring) on an M5 (`roll_idler_screw`) down through the idler and the riser's slot into an M5 nut (`roll_idler_nut`) in a channel under the face, from the open +N face: ± `travel` along N tensions the belt, the nut sliding with the screw; nearer than ~73 (`centre_distance`'s note) the idler would meet the ring's flange and the belt's far run | `RollIdlerParams`, `tests/forearm/test_roll_idler.py` |
| rotor clamp | 4× M3 × 35 on Ø24.5 (`clamp_*`, turned 45° off the wall's screws) from the pulley's front face through both hubs into the shaft's nuts: the pulley, both inner rings and the shaft one block, the drive whole without the forearm | `RollDriveParams clamp_*` |
| frame (stator, PETG) | ONE printed part round the motor, also the elbow's output flange: seen along the elbow axis **round about it** (`end_r` 45, concentric with `j1_link`'s round end) and tangent up to the **tower** round the roll axis (`tower_y`), from the underside (`block_x[0]`, the elbow flange's lip / boss / journal / stub under it) to the open +N face (`block_x[1]`); the motor's **pocket** open on that face, its front wall the **plate** (`pad_t`: round holes - no slide, the idler tensions the belt -, the motor's 4× M3 from the front face, `roll_motor_screws`, their heads flush in counterbores; the pilot hole, the shaft's slit), the plate's face the frame's one **front face**, the tensioner's riser on it; the **bearing boss** on the front face round the roll axis (`bearing_boss_dia`, bearing 2's seat in it), centred on the frame - the underside, the +N face and the tower's top all `ROLL_FRAME_R` from the roll axis, the forearm wall's radius (`wall_od` / 2): the frame flush with the round parts that turn about the roll axis, its underside lifted off `j1_link`'s flat top, the lip bridging the gap -; the shaft's **bay** behind bearing 1, open on +N | `forearm_roll_block` |
| forearm interface | `j2_link`'s wall at 48…56 mm along the roll axis (`wall_x` −56…−48): a Ø60 round flange on the roll axis (`wall_od`, over the ring's Ø59.19 flanges) on a foot as wide as itself and the straight web (`WebParams.half_w`), two gussets beside the wrist motor (`rib_*`); Ø40 × 2 recess, Ø24 bore, 4× M3 × 16 on Ø31 from its wrist face (`screw`, `screw_len`; the bottom one's head in a channel under the web, `screw_channel`); the forearm turned `FOREARM_ROLL_TURN_DEG` about the roll axis from the capture, the wall on the ring turned with it (§1) | `RollEndParams`, `ForearmConfig.roll_turn_deg` |
| clearances held by tests | the frame's underside FACE_GAP or more above `j1_link`'s flat top, the boss 2.0 above its recess floor, level with the upper 6806's top (`FACE_GAP`: the least gap a face the elbow turns may keep); the rotor 1 mm off the frame (`run_gap`: the ring's flange over the bearing boss, the shaft in the bay); folded elbow (to `ELBOW_PITCH_LIMITS_DEG`, −47° / +90°) and rolled forearm (±`FOREARM_ROLL_LIMIT_DEG`) < 1 mm³ against every neighbour; at the elbow's limits the forearm, rolled anywhere, ≥ 2 mm off `j1_link`; over the elbow's range the drive ≥ 1 mm off the elbow motor and its board on the upper arm's same side (how near at the lower limit: `docs/open_issues.md`); the stator behind the forearm wall's plane but the motor shaft's tip and the tensioner's screw head, which pass it and which the rolling forearm clears round the roll axis; journals 0.9–1.0 × the 99 mm³ press | `tests/forearm/test_roll_drive.py` |
| link masses (URDF) | `elbow_link` carries the stator (the frame, both bearings, the motor + board, the 20T, the belt, the tensioner), `forearm_link` the rotor (the pulley + the shaft) + the forearm and what it carries (§5); the masses are the `<inertial>` blocks (`tools/robot/derive.py`), their sum checked by `test_link_masses_add_up` | `robot/arm.urdf` |
| purchased per drive | what to order, with quantities: `./cadtool python tools/bom.py --module forearm_roll_drive` (its BUY list, and BUY, NOT MODELLED from `tools/bom.py EXTRAS`); the elbow 90T's M4 screws and nuts are the arm's (`elbow_pulley_screws`, `elbow_pulley_nuts`) | `tools/bom.py` |
| printed per drive | frame, shaft, pulley (`./cadtool python tools/export_printables.py --parts forearm_roll_block forearm_roll_shaft forearm_roll_pulley`) | `print/` |

## 1. Why a roll, and where
The SolidWorks arm had five revolute joints — `base_yaw` and three parallel pitches (`shoulder_pitch`,
`elbow_pitch`, `wrist_pitch`) plus `wrist_roll` — so the tool's approach axis could never leave the vertical
plane through the base axis. `wrist_pitch` and `wrist_roll` already met at one point, the **wrist centre**
(`robot/frames.py WRIST_CENTRE`: `WRIST_CENTRE_ALONG_N` = 17 mm back along the pitch axis from the wrist_pitch
origin, 5 µm off both axes). A roll about the forearm through that point would make the last three axes concurrent -
a spherical wrist; the wrist roll drive (`docs/wrist_roll.md`) puts the wrist roll axis `WRIST_ROLL_OFFSET` across the
pitch axis instead, so an IK must be numeric. The roll runs through the wrist centre on the pitch axis. The axis runs
along `ELBOW_TO_WRIST_INPLANE`, `FOREARM_ROLL_AXIS_Z` = 42 − 17 = 25 mm along N from `j2_link`'s origin — in
`j2_link`'s frame: y 0, z 25, along −X.

**The elbow offset.** The roll motor sits ON the elbow axis and the roll axis above it, the drive's design centre
distance away (`ForearmConfig.elbow_offset` = `RollDriveParams.centre_distance`, `ELBOW_ROLL_OFFSET`; the stock belt runs round its tensioner, §0): so the
forearm, the wrist and the gripper sit that far across the elbow axis - along `j2_link`'s +Y, up in the arm's swing
plane -, moved rigidly from the capture (`lib/placements.py SHIFTS`: `j2_link#1` and every record beyond it; the
elbow's own records stay). The arm gains an elbow offset (the roll axis and the elbow axis do not cross); in
exchange the forearm lifts back to `ELBOW_PITCH_LIMITS_DEG`'s −47° (the frame, round about the elbow axis, sweeps
nothing of `j1_link` as the elbow turns; it comes nearest the elbow motor's board there: `docs/open_issues.md`). In
`j2_link`'s frame the elbow axis runs along Z through y = −`elbow_offset`.

**The forearm is turned.** `j2_link` and everything beyond it - the wrist-pitch motor and its board, the wrist, the
gripper - are turned half a turn about the roll axis from the capture (`ForearmConfig.roll_turn_deg`,
`lib/placements.py TURNS`): the wrist-pitch motor sits on the upper arm's side of the roll axis, its belt on the
other, and at zero the gripper is rolled with it (the URDF's capture pose; `robot/AGENTS.md`). The drive keeps its
pose - the rotor too, so the hard stop and ±`FOREARM_ROLL_LIMIT_DEG` stay where they were - and the wall's 4 bolts,
at 0 / 90 / 180 / 270° about the roll axis, take the pulley as before. The drive's frame and its tests stay in
`j2_link`'s frame BEFORE the turn (the elbow side's view of the forearm: `lib/mounts.py ModuleMount`,
`tests/forearm/helpers.py in_drive_frame()`); "`j2_link`'s frame" below is that one.

**The pulley is the output.** The 90T ring is the front end of the rotor, outside the bearings, and the forearm
bolts straight onto it: the belt drives the ring, the ring the forearm. With both bearings pressed onto printed
journals, each needs its own journal entered from its own end - so the rotor splits between them, the elbow's 6806
pattern: the pulley's hub through bearing 2 and the shaft's through bearing 1 meet inside the lip, the lip spacing
the outer rings as the meeting hubs space the inner ones, and the rotor clamp holds the two pieces and both inner
rings together. No end cap: the frame's tower is one piece, its lip between the bearings.

## 2. Layout (module frame)
Module frame (`lib/forearm/layout.py module_frame_in_host`, `lib/mounts.py MODULE_MOUNTS` on `j2_link#1`, in its
frame before the forearm's turn):
origin on the roll axis at the elbow axis' station, **+Z along the roll axis toward the wrist** (host −X),
**+X = host +Z** (N, away from the upper arm), **+Y = host +Y = up in the arm's swing plane** (the tower above the
motor); the **elbow axis runs along X at y = `y_elbow`** (−`elbow_offset`). Every station: `stack_positions()`.

**The frame is one printed part round the motor that is also the elbow's output flange.** Its outline seen along the
elbow axis (`roll.py profile`): a circle about the elbow axis at the upper arm's round end (`end_r`), tangent up to
the tower round the roll axis (`tower_y`, from `z_tower_rear` to the front face), cut off at the front face - extruded
along N from the underside (`block_x[0]`, 6.0 over `j1_link`'s flat top) to the open +N face (`block_x[1]`). The
measured SolidWorks coupler (`reference/solidworks/j3_coupler.step`: Ø78 flange host z −10…0, Ø62 boss
−14…−10, Ø40 journal −15.3…−14, Ø30 stub −22…−15.3, Ø12.5 bore, 4× M4 at r 11 from the pulley) is repeated under the
underside about the elbow axis, the elbow 90T bolts straight into it (through the elbow's 6806 pair, §4), and
`j3_coupler#1` is **retired** (`lib/placements.py RETIRED`: the record stays, no table claims it; the wrist's
`j3_coupler#2` is untouched).

| station | what | fixed by |
|---|---|---|
| the round end, `end_r` about (y `y_elbow`, z 0), x `block_x` −30…30 | the frame's outline round the elbow axis; the motor and its board inside it (the pocket's corners ≈ 40 from the axis) | `j1_link`'s round end (concentric); `ROLL_FRAME_R` (the underside 6.0 over `j1_link`'s flat top, FACE_GAP or more) |
| the elbow flange, x −50…−30 about (y `y_elbow`, z 0) | `lip_dia` Ø72 lip −35…−30 over `j1_link`'s Ø80 recess, from 1 above its flat top up to the underside, `boss_dia` Ø62 −39…−35, `journal_dia` Ø40 −40.3…−39, `step_dia` Ø33 −41…−40.3 (the shoulder on the upper 6806's inner ring), `stub_dia` Ø30 −50…−41 (in the upper of the elbow's 6806-2RS pair and on through the lip, `j1_link`'s bore: `lib/mounts.py bearing_6806#3` / `#4`), the Ø12.5 `pin_bore_dia` (from the stub's end, blind at −37: 1 mm under the nut seats); the elbow 90T's **4× M4 at `pulley_bolt_r` 11, turned `pulley_bolt_deg` 45°** off the axes: Ø4.4 clearance −50…−36, then each nut's **hex channel** (`nut_af` 6.85, a flat toward the elbow axis) from its seat `nut_seat_x` −36 up through the floor into the motor's pocket | the measured coupler, its stub `PULLEY_SEAT_SHIFT` (3) longer: its end (host −25, the lip's lower face) sits on the re-seated pulley's face; the nuts (−36…−32.8) and the screw tips (−31.4) ≥ 2 mm under the pocket's floor |
| `motor_pocket` y `y_motor` ± 22.25, z −29.25 … `z_motor_face` 26.85, from `x_cradle` −23 out through +N | the roll motor ON the elbow axis, centred on the roll axis in X: its mounting face `z_motor_face`, the body −12.65…26.85, the MKS board −26.75…−12.65, spun `motor_spin_deg` 90° so the connector points +X toward the open face (its end 2 inside it); the pocket round the board's 43 square + `pocket_clear` (no slide: the idler tensions the belt), `pocket_rear` behind the board - room for the motor and its board to go in from the +N face `motor_drop` low (the pilot boss under the plate) and lift into place; the floor (`web_t`) 2 mm under the motor | the 20T's teeth level with the ring (`t20`), its hub `pulley_seat` over the motor's pilot boss: the motor `pad_t` behind the front face, the plate thick enough for the screws' counterbores; the shaft's tip past the forearm wall's plane, which the rolling forearm clears round the roll axis |
| `plug_slots`, at the board's side headers, out through +N | a **plug slot** off each side of the pocket: spun with the motor, the MKS board's two side headers face ±Y, nearer the pocket's walls than their plugs' `MKS_PLUG_ROOM`; each slot `plug_clear` round its header's outline, `MKS_PLUG_ROOM` past the header's face and `plug_spare` more, open from the header up through the +N face - the −Y one inside the round end, the +Y one in the solid under the tower | the board's plugs stay in as the arm moves (`tests/test_sweeps.py test_the_mks_boards_keep_their_plug_room`); the motor goes in with its cables plugged |
| `z_motor_face` 26.85 … `z_front` 33.85 | the **plate** (`pad_t`), the pocket's front wall: round holes on the motor's 31 mm square (`motor_holes`), each screw's head (`roll_motor_screws`, 4× M3 × `ROLL_MOTOR_SCREW_LEN`) flush in a Ø5.7 counterbore from the front face down to `z_motor_screws` 30.85, the Ø22.3 pilot hole, the shaft's **slit** (`shaft_slit`) from the pilot hole out through the +N face; its face is the frame's front face; the 20T's hub 1.5 down in its pilot hole | the motor's face; 4 of plate under the heads, the counterbores `head_h` deep, the screws' thread in the motor `motor_holes.thread` |
| `z_front` … `z_idler` 38.8 | the tensioner's **riser** on the front face (`riser_w`, its land `land_w` on top under the idler's inner ring), a stadium along N over the idler's `travel`; the M5's slot through it and the plate, on past the screw's tip; `roof` under the face, the nut's **channel** from the +N face | the idler level with the belt (`z_ring_mid`); the channel `nut_fit` round the nut, off the motor's pocket |
| `z_tower_rear` 0.75 … `z_front` | the **tower** round the roll axis, up to `tower_y` | `ROLL_FRAME_R`, as far as the underside and the +N face (the bay + 4.4), its corners `tower_corner_r` (the bearing boss 1 inside them), `tower_wall` behind it |
| `z_bay` 2.75 … `z_bearing_1` 21.6 | the shaft's **bay** (`bay_r` 25.6 round the roll axis, open through the +N face): the shaft, its nuts, the stop, the cable exit; the **stop post** on the tower's rear face at −X (`stop_post_r`, `stop_post_t` back from it) | the shaft's hub length through bearing 1 + `bay_clear` behind its rear face (it goes in through the +N face and slides forward into the seat); round the lug's corners + 1 |
| `z_bearing_1` 21.6 … `z_lip` 28.6 | **bearing 1**'s seat Ø42.15, open into the bay (the bearing goes in from it) | |
| `z_lip` 28.6 … 30.6 | the lip (ID `lip_id` 37.6) between the outer rings | the elbow's |
| `z_bearing_2` 30.6 … `z_boss` 37.6 | **bearing 2**'s seat Ø42.15 in the **bearing boss** (Ø`bearing_boss_dia` 52 on the front face round the roll axis: 4.9 of wall, 4 inside the frame's three sides, on the flat of the front face), right under the ring, the front face (33.85) running through its seat; the ring's rear flange turns over the boss's top, 3.6 past its edge - no cup round it | the ring, `run_gap` above it; the belt's edge clear above it |

The rotor, the forearm wall back (`stack_positions`):

| station | what | fixed by |
|---|---|---|
| `z_wall` 48 … `z_wall_back` 56 | the forearm wall (`wall_x` −56…−48), round (Ø60, over the ring); 4× M3 × 16 on Ø31 from its wrist face through the wall and the spigot into the nuts in the pulley's core (tips `z_end_tip` 40, nuts 41…43.4, pockets from the bore, `end_nut_pocket`) | the rolling forearm clears `j1_link`'s r 45 end |
| 48 … `z_spigot_end` 50 | the pulley's **Ø39.7 spigot** (`RollEndParams.flange_dia`) in the wall's Ø40 recess; the rotor clamp's counterbores from its face, their floor `z_clamp_head` 46.75 | the wall's recess |
| `z_ring_flange_1` 38.6 … 48 | the **90T ring** (teeth `z_ring` 39.8…46.8, `z_ring_mid` 43.3) on the pulley's Ø44 core, its front flange on the wall | the forearm wall's station |
| `z_boss` 37.6 … 38.6 | the pulley's Ø33 shoulder on bearing 2's inner ring | `run_gap` |
| `z_meet` 29.6 … 37.6 | the pulley's hub (Ø30.3 journal) through bearing 2 | the lip's middle |
| `z_bearing_1` 21.6 … `z_meet` 29.6 | the shaft's hub (Ø30.3 journal) through bearing 1, its end on the pulley's | the lip's middle |
| `z_shoulder_1` 20.6 … 21.6 | the shaft's Ø33 shoulder on bearing 1's inner ring | `run_gap` |
| `z_shaft_end` 11.75 … 20.6 | the shaft's Ø40 flange: the rotor clamp's 4 nuts 12.75…15.15 (pockets open to its rear face and the bore, `clamp_nut_pocket`), the **stop lug** at +X (`stop_lug_r`, `stop_lug_t` from the rear face; where it meets the post: §0 range) | the clamp's tips (`clamp_screw_len` from `z_clamp_head`) |

Ratio 90 / 20 = 4.5 (`FOREARM_ROLL_RATIO`) → ≈ 1.1–1.8 N·m at the roll from the 40 mm kit motor, against a worst-case
static load of ≈ 0.7 N·m (forearm horizontal and rolled to hold the wrist's longest lever level: ≈ 0.78 kg wrist +
gripper - `wrist_pitch_link`, `wrist_roll_link` and the jaws in `robot/arm.urdf` - + 0.2 kg payload at `tool0`). The
shoulder (`cycloidal_drive.md` §7) carries the drive too (its masses: `elbow_link`'s and `forearm_link`'s `<inertial>`
blocks in `robot/arm.urdf`). The elbow's torque path is the elbow 90T → 4× M4 into captive nuts → the frame → the
tower → the 6806 pair → the rotor → the forearm wall: the frame carries the forearm's load `ELBOW_ROLL_OFFSET` across
from the elbow flange, and the pair, 9 mm apart centre to centre, the forearm's bending moment (`docs/open_issues.md`:
check both on the print).

## 3. Fits and printing
PETG, the drive's rules (`cycloidal_drive.md` §6): the seats are the bearing OD + `seat_add` 0.15 (a printed hole
comes out undersize — print a fit gauge first), the journals `journal_add` 0.3 interference. Tests
(`tests/forearm/test_roll_drive.py`) hold the journal press fit at 0.9–1.0 × 99 mm³ per bearing, 0 in the seats,
every other pair in the module below 1 mm³, the rotor `run_gap` off the frame, the two hubs face to face in the lip,
and the frame's stub end ON the elbow pulley's face (`tests/test_mounts.py test_bearing_stacks` holds the elbow's 6806
stack). Print the frame front face down (the tower's seats print as vertical bores, the pocket's rim as walls; the
tensioner's riser land stands on the bed, `z_idler` − `z_boss` proud of the bearing boss: supports under the boss and
the rest of the front face, under the round end where it widens off the bed, under the elbow flange standing off the
underside and across the pocket's and the bay's open sides - a `dfam-check` pass before the first print,
`docs/open_issues.md`),
the pulley spigot down (the ring's grooves print vertically; a support ring under its front flange, 2 mm over the bed),
the shaft rear face down (its nut pockets open to the bed).

## 4. Assembly sequence
1. Four M4 nuts (`elbow_pulley_nuts`) into the frame's hex channels from the motor's pocket (before the motor), down
   onto their seats - the channel's 6.85 across flats holds each.
2. The elbow's 6806-2RS pair into `j1_link`'s bore, one each side of the lip; the frame's stub into the upper one
   (its Ø33 shoulder on the inner ring, its end through the lip), the elbow 90T's hub into the lower one from below
   (its Ø34.76 ring on that inner ring, its end on the stub's end): 4× M4 × 35 (`elbow_pulley_screws`, the heads in
   the pulley's counterbores, `GT2_PULLEY_90T_HEAD_SEAT` under its face) up through the pulley's hub (its holes opened
   to `M4_CLEAR`), the stub, the journal and the boss into the nuts (the tips 1.4 mm past them) bolt the pulley flat
   onto the stub and clamp both inner rings - the stub's 2 mm in the lip spaces them as the lip spaces the outer rings.
   The hex holds each nut, so the screws re-tighten from the pulley side at any time (with the motor out, a screw
   taken right out can drop its nut into the pocket).
3. Press bearing 2 onto the pulley's hub against its shoulder, bearing 1 onto the shaft's hub against its shoulder.
4. The pulley + bearing 2 into the tower from the front (the outer ring in its seat, on the lip; the ring's flange
   over the bearing boss); the shaft + bearing 1 into it from the bay (through the +N opening, then along the axis), its hub's
   end onto the pulley's.
5. Four M3 nuts into the shaft's flange pockets from its rear face (in the bay); 4× M3 × 35 (the rotor clamp) from the
   pulley's front face, the heads into its counterbores, through both hubs into the nuts: the rotor turns, whole.
6. The motor, its board on and the board's two side cables plugged (their plugs ride down the plug slots), into the
   pocket from the +N face `motor_drop` low - the pilot boss passes under the plate,
   the shaft along the slit - then lifted onto the plate, the boss into the pilot hole (4× M3 - `roll_motor_screws` -
   from the front face, the heads into its counterbores; connector toward +N); the 20T on its shaft, its hub on a
   `pulley_seat` shim on the pilot boss (its teeth then level with the ring), its set screws tightened above the face. The tensioner's M5 nut into its
   channel from the +N face; the roll belt (`forearm_roll_belt`) round the ring and the 20T; the idler on the riser
   against the belt's back, the M5 down through it into the nut; push the idler along its slot until the belt is
   tight, then tighten the screw.
7. Cables from the forearm through the bore, out into the bay and on toward the upper arm.
8. Four M3 nuts into the pulley's core pockets from its cable bore (7–9 mm in from its front face, tweezers), each
   pushed out against the pocket's end. Then the forearm wall onto the spigot (2 mm into the recess) BEFORE the
   wrist-pitch motor goes on: 4× M3 × 16 from the wall's wrist face through the spigot into the nuts; the bottom one
   goes into the channel under the web from the belt side, its key through the channel or the motor slot. A screw taken
   right out can drop its nut into the bore. The wrist-pitch motor's body stands in front of three of the heads (it
   sits `motor_wall_clear` or more off them): take it off to reach them later. The stop lug on the
   shaft's flange meets the frame's post just past ±`FOREARM_ROLL_LIMIT_DEG`.

## 5. Attachment to the arm (`robot/`)
`elbow_link` = `gt2_pulley_90t#3` (the re-seated elbow 90T) + its screws and nuts + `forearm_roll_drive#1:stator` (the
elbow's driven side: the frame whose underside is the coupler, both bearings, the motor + board on the elbow axis and
its 4× M3, the 20T, the belt and its tensioner); `forearm_link` = `forearm_roll_drive#1:rotor` (the pulley and the shaft) + `j2_link`
and what it carries (the wrist-pitch motor + board, its 20T, the wrist belt and its tensioner, the wrist's 6806 pair:
`robot/frames.py LINKS`). `Joint("forearm_roll")` has Z along the forearm and X = N (its child's long direction IS
the axis); its origin sits `ELBOW_ROLL_OFFSET` across `elbow_link`'s frame (`robot/frames.py FOREARM_ROLL_ORIGIN` from
`j2_link#1`'s shifted origin, on the turn's axis; `ELBOW_ORIGIN` from the retired `j3_coupler#1`'s). Beyond it the axes
are turned with the forearm (`WRIST_PITCH_AXIS` the capture's −N): the forearm's turn changes `forearm_link`'s mesh and
inertial and the `wrist_pitch` origin, nothing further out. The forearm side of the interface is
`lib/forearm/params.py RollEndParams`: the wall at `wall_x` (−56…−48), round about the roll axis (`wall_od`), its Ø40
recess, the Ø31 bolt circle (the screws' nuts in the pulley) and the Ø24 cable bore, on the end of the straight web
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
`docs/open_issues.md`: the home sensor's mount (the modelled `ky003_hall_sensor`'s chip beside the stop
post on the tower, a magnet in the stop lug, to the MKS board's limit input), the cable route through the Ø18 bore; the
6806 seats' PETG fit; the frame's print orientation and its flex under the forearm's load, the
6806 pair's tilt play at the roll, the rotor clamp's thin walls and its nut pockets' fit; `t20_hub`,
the limit and the printed stop's strength; the elbow offset in a future IK; the room to lift the forearm further back
than `ELBOW_PITCH_LIMITS_DEG`'s lower limit; the roll motor's **CAN id** (`software/control/src/config.py` has no row
for it: it names fewer boards than the arm carries).
