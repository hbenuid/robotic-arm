# The forearm roll drive — the arm's 6th joint

**Purpose:** spec and attachment of the belt-driven forearm roll (`forearm_roll`, between `elbow_pitch` and
`wrist_pitch`), the way `cycloidal_drive.md` documents the shoulder. Code: `lib/forearm/` (`params.py
RollEndParams` + `RollDriveParams` + `ForearmConfig.elbow_offset`, `layout.py stack_positions`, `roll.py`),
`assemblies/forearm_roll_drive.py`, `parts/joints/forearm_roll_*.py`; tests `tests/forearm/`.
Numbers below name the constants; the values live in `lib/forearm/params.py`.

## 0. Specifications at a glance
| item | value | where |
|---|---|---|
| joint | `forearm_roll`, revolute, `elbow_link → forearm_link`; axis along the forearm through the wrist centre, `ELBOW_ROLL_OFFSET` (60.9, the roll belt's centre distance) across the elbow axis - **an elbow offset: the two axes do not cross** -, `FOREARM_ROLL_AXIS_Z` along N from `j2_link`'s origin | `robot/frames.py`, `lib/placements.py SHIFTS` |
| range | ±170° (`FOREARM_ROLL_LIMIT_DEG`), printed hard stop (a lug on the shaft's flange, a post on the frame's tower, in the bay behind bearing 1); home sensor modelled (`ky003_hall_sensor`), not placed | `lib/params.py`, `RollDriveParams stop_*` |
| drive | NEMA 17 × 40 mm + MKS SERVO42D (a 4th CAN id) **on the elbow axis**, GT2 20T on the motor, the 90T ring **integral to the pulley - the output**, **4.5 : 1**, 240-2GT × 6 mm belt (`roll_belt`, centre distance 60.9 = the offset) | `RollDriveParams`, `lib/belts.py` |
| torque | ≈ 1.1–1.8 N·m at the roll vs ≈ 0.9 N·m worst-case static load | §2 |
| bearings | 2× 6806-2RS (30 × 42 × 7, the arm's other joints' bearing), **back to back on a 2 mm lip** in the frame's tower, bearing 2 right under the ring (seats Ø42.15, journals Ø30.3) | `RollDriveParams bearing_*`, `lib/bearings.py` |
| pulley (rotor, PETG - the output) | the integral flanged 90T ring on a Ø44 core, its rear flange sunk in the frame's cup; its hub through bearing 2 to the lip's middle, the Ø33 shoulder on bearing 2's inner ring; the Ø39.7 spigot in the forearm wall's recess - the wall bolts onto the ring's face (4× M3 on Ø31 into nuts in the core); the rotor clamp's 4× M3 heads sunk in its front face | `forearm_roll_pulley` |
| shaft (rotor, PETG) | a collar behind bearing 1: its hub through bearing 1 to the lip's middle (it meets the pulley's there), the Ø33 shoulder, the Ø40 flange with the rotor clamp's 4 nuts (pockets open to its rear face) and the stop lug; the Ø18 cable bore through both rotor pieces | `forearm_roll_shaft` |
| rotor clamp | 4× M3 × 35 on Ø24.5 (`clamp_*`, turned 45° off the wall's screws) from the pulley's front face through both hubs into the shaft's nuts: the pulley, both inner rings and the shaft one block, the drive whole without the forearm | `RollDriveParams clamp_*` |
| frame (stator, PETG) | ONE printed part round the motor, also the elbow's output flange: seen along the elbow axis **round about it** (`end_r` 45, concentric with `j1_link`'s round end) and tangent up to the **tower** round the roll axis (`tower_y`), from the underside (`block_x[0]`, the elbow flange's lip / boss / journal / stub under it) to the open +N face (`block_x[1]`); the motor's **pocket** open on that face, its front wall the **plate** (the tension slots, the pilot slot), the plate's face the frame's one **front face**; the **cup** on the front face round the roll axis (`cup_od`); the shaft's **bay** behind bearing 1, open on +N | `forearm_roll_block` |
| forearm interface | `j2_link`'s wall at 48…56 mm along the roll axis (`wall_x` −56…−48): a Ø60 round flange on the roll axis (`wall_od`, over the ring's Ø59.19 flanges) on a foot as wide as itself, the web necked down to it from the wrist boss (`neck_half_w`), two gussets beside the wrist motor (`rib_*`); Ø40 × 2 recess, Ø24 bore, 4× M3 × 16 on Ø31 from its wrist face (`screw`, `screw_len`; the bottom one's head in a channel under the web, `screw_channel`) | `RollEndParams` |
| clearances held by tests | the frame's underside FACE_GAP or more above `j1_link`'s flat top, the boss 2.0 above its recess floor, level with the upper 6806's top (`FACE_GAP`: the least gap a face the elbow turns may keep); the rotor 1 mm off the frame (`run_gap`: the ring's flange in the cup, the shaft in the bay); folded elbow (to `ELBOW_PITCH_LIMITS_DEG`, −47° / +90°) and rolled forearm (±170°) < 1 mm³ against every neighbour; at the elbow's limits the forearm, rolled anywhere, ≥ 2 mm off `j1_link`; over the elbow's range the drive ≥ 1 mm off the elbow motor on the upper arm's same side (9.3 mm at −47°; 1 mm only at −60°); the motor shaft's tip, the 20T and the whole stator behind the forearm wall's plane; journals 0.9–1.0 × the 99 mm³ press | `tests/forearm/test_roll_drive.py` |
| link masses (URDF) | `elbow_link` carries the stator (the frame, both bearings, the motor + board, the 20T), `forearm_link` the rotor (the pulley + the shaft) + forearm + wrist motor; the masses are the `<inertial>` blocks (`tools/robot/derive.py`), their sum checked by `test_link_masses_add_up` | `robot/arm.urdf` |
| purchased per drive | the 6806-2RS pair, a NEMA 17 × 40 kit + MKS SERVO42D, a GT2 20T (5 mm bore), the roll belt, the motor's M3, the rotor clamp's and the forearm wall's M3 screws + nuts — with quantities: `./cadtool python tools/bom.py --module forearm_roll_drive`; the elbow 90T's M4 screws and nuts are the arm's (`elbow_pulley_screws`, `elbow_pulley_nuts`) | `tools/bom.py` |
| printed per drive | frame, shaft, pulley (`./cadtool python tools/export_printables.py --parts forearm_roll_block forearm_roll_shaft forearm_roll_pulley`) | `print/` |

## 1. Why a roll, and where
The SolidWorks arm had five revolute joints — `base_yaw` and three parallel pitches (`shoulder_pitch`,
`elbow_pitch`, `wrist_pitch`) plus `wrist_roll` — so the tool's approach axis could never leave the vertical
plane through the base axis. `wrist_pitch` and `wrist_roll` already met at one point, the **wrist centre**
(`robot/frames.py WRIST_CENTRE`: `WRIST_CENTRE_ALONG_N` = 17 mm back along the pitch axis from the wrist_pitch
origin, 5 µm off both axes). A roll about the forearm through that point makes the last three axes concurrent —
the standard 6R arm with a spherical wrist and a closed-form IK. The axis runs along `ELBOW_TO_WRIST_INPLANE`,
`FOREARM_ROLL_AXIS_Z` = 42 − 17 = 25 mm along N from `j2_link`'s origin — in `j2_link`'s frame: y 0, z 25, along −X.

**The elbow offset.** The roll motor sits ON the elbow axis and the roll axis above it, the roll belt's centre
distance away (`ForearmConfig.elbow_offset` = `RollDriveParams.centre_distance`, `ELBOW_ROLL_OFFSET`): so the
forearm, the wrist and the gripper sit that far across the elbow axis - along `j2_link`'s +Y, up in the arm's swing
plane -, moved rigidly from the capture (`lib/placements.py SHIFTS`: `j2_link#1` and every record beyond it; the
elbow's own records stay). The arm gains an elbow offset (the roll axis and the elbow axis no longer cross; the wrist
stays spherical); in exchange the forearm lifts back to `ELBOW_PITCH_LIMITS_DEG`'s −47° and further (the frame, round
about the elbow axis, sweeps nothing as the elbow turns). In `j2_link`'s frame the elbow axis runs along Z through
y = −`elbow_offset`.

**The pulley is the output.** The 90T ring is the front end of the rotor, outside the bearings, and the forearm
bolts straight onto it: the belt drives the ring, the ring the forearm. With both bearings pressed onto printed
journals, each needs its own journal entered from its own end - so the rotor splits between them, the elbow's 6806
pattern: the pulley's hub through bearing 2 and the shaft's through bearing 1 meet inside the lip, the lip spacing
the outer rings as the meeting hubs space the inner ones, and the rotor clamp holds the two pieces and both inner
rings together. No end cap: the frame's tower is one piece, its lip between the bearings.

## 2. Layout (module frame)
Module frame (`lib/forearm/layout.py module_frame_in_host`, `lib/mounts.py MODULE_MOUNTS` on `j2_link#1`):
origin on the roll axis at the elbow axis' station, **+Z along the roll axis toward the wrist** (host −X),
**+X = host +Z** (N, away from the upper arm), **+Y = host +Y = up in the arm's swing plane** (the tower above the
motor); the **elbow axis runs along X at y = `y_elbow`** (−`elbow_offset`). Every station: `stack_positions()`.

**The frame is one printed part round the motor that is also the elbow's output flange.** Its outline seen along the
elbow axis (`roll.py profile`): a circle about the elbow axis at the upper arm's round end (`end_r`), tangent up to
the tower round the roll axis (`tower_y`, from `z_tower_rear` to the front face), cut off at the front face - extruded
along N from the underside (`block_x[0]`, 3.0 over `j1_link`'s flat top) to the open +N face (`block_x[1]`). The
measured SolidWorks coupler (`reference/solidworks/j3_coupler.step`, 2026-09-23: Ø78 flange host z −10…0, Ø62 boss
−14…−10, Ø40 journal −15.3…−14, Ø30 stub −22…−15.3, Ø12.5 bore, 4× M4 at r 11 from the pulley) is repeated under the
underside about the elbow axis, the elbow 90T bolts straight into it (through the elbow's 6806 pair, §4), and
`j3_coupler#1` is **retired** (`lib/placements.py RETIRED`: the record stays, no table claims it; the wrist's
`j3_coupler#2` is untouched).

| station | what | fixed by |
|---|---|---|
| the round end, `end_r` about (y `y_elbow`, z 0), x `block_x` −33…26 | the frame's outline round the elbow axis; the motor and its board inside it (the pocket's corners ≈ 39 from the axis) | `j1_link`'s round end (concentric); the underside's FACE_GAP over its flat top |
| the elbow flange, x −50…−33 about (y `y_elbow`, z 0) | `lip_dia` Ø72 lip −35…−33 in `j1_link`'s Ø80 recess, `boss_dia` Ø62 −39…−35, `journal_dia` Ø40 −40.3…−39, `step_dia` Ø33 −41…−40.3 (the shoulder on the upper 6806's inner ring), `stub_dia` Ø30 −50…−41 (in the upper of the elbow's 6806-2RS pair and on through the lip, `j1_link`'s bore: `lib/mounts.py bearing_6806#3` / `#4`), the Ø12.5 `pin_bore_dia` (from the stub's end, blind at −37: 1 mm under the nut seats); the elbow 90T's **4× M4 at `pulley_bolt_r` 11, turned `pulley_bolt_deg` 45°** off the axes: Ø4.4 clearance −50…−36, then each nut's **hex channel** (`nut_af` 6.85, a flat toward the elbow axis) from its seat `nut_seat_x` −36 up through the floor into the motor's pocket | the measured coupler, its stub `PULLEY_SEAT_SHIFT` (3) longer: its end (host −25, the lip's lower face) sits on the re-seated pulley's face; the nuts (−36…−32.8) and the screw tips (−31.4) ≥ 2 mm under the pocket's floor |
| `motor_pocket` y `y_motor` ± 24.75, z −29.75 … `z_motor_face` 24.85, from `x_cradle` −23 out through +N | the roll motor ON the elbow axis, centred on the roll axis in X: its mounting face `z_motor_face`, the body −14.65…24.85, the MKS board −28.75…−14.65, spun `motor_spin_deg` 90° so the connector points +X out of the open face; the pocket round the board's 43 square + the tension travel (± `pad_slot_len` / 2) + `pocket_clear`, `pocket_rear` behind the board; the floor (`web_t`) 2 mm under the motor | the 20T's teeth level with the ring (`t20`); `pulley_lift` 3.5: the 20T's far end at the motor shaft's tip, the tip 1.15 behind the forearm wall's plane |
| `z_motor_face` 24.85 … `z_front` 28.85 | the **plate**, the pocket's front wall: the 4 tension slots ±2.5 along Y on the motor's 31 mm square, the Ø22.3 pilot slot; its face is the frame's front face; the 20T 3.5 in front of it | the motor's face |
| `z_tower_rear` 0.75 … `z_front` | the **tower** round the roll axis, up to `tower_y` | the bay + 3.4 over it, `tower_wall` behind it |
| `z_bay` 2.75 … `z_bearing_1` 21.6 | the shaft's **bay** (`bay_r` 25.6 round the roll axis, open through the +N face): the shaft, its nuts, the stop, the cable exit; the **stop post** on the tower's rear face at −X (`stop_post_r`, `stop_post_t` back from it) | the shaft's hub length through bearing 1 + `bay_clear` behind its rear face (it goes in through the +N face and slides forward into the seat); round the lug's corners + 1 |
| `z_bearing_1` 21.6 … `z_lip` 28.6 | **bearing 1**'s seat Ø42.15, open into the bay (the bearing goes in from it) | |
| `z_lip` 28.6 … 30.6 | the lip (ID `lip_id` 37.6) between the outer rings; the front face (28.85) runs through it | the elbow's |
| `z_bearing_2` 30.6 … `z_cup` 37.6 | **bearing 2**'s seat Ø42.15, right under the ring | the ring, `run_gap` above it |
| `z_cup` 37.6 … `z_rim` 39.5 | the **cup**: a boss Ø`cup_od` 66 on the front face round the roll axis (its −N side on the underside's plane), bored Ø61.2 round the ring's flange (`cup_id_add`); the ring's rear flange turns sunk in it | the rim `rim_under_teeth` under the teeth: the belt (its edge 0.5 above them) runs clear, no window |

The rotor, the forearm wall back (`stack_positions`):

| station | what | fixed by |
|---|---|---|
| `z_wall` 48 … `z_wall_back` 56 | the forearm wall (`wall_x` −56…−48), round (Ø60, over the ring); 4× M3 × 16 on Ø31 from its wrist face through the wall and the spigot into the nuts in the pulley's core (tips `z_end_tip` 40, nuts 41…43.4, pockets from the bore, `end_nut_pocket`) | the rolling forearm clears `j1_link`'s r 45 end |
| 48 … `z_spigot_end` 50 | the pulley's **Ø39.7 spigot** (`RollEndParams.flange_dia`) in the wall's Ø40 recess; the rotor clamp's counterbores from its face, their floor `z_clamp_head` 46.75 | the wall's recess |
| `z_ring_flange_1` 38.6 … 48 | the **90T ring** (teeth `z_ring` 39.8…46.8, `z_ring_mid` 43.3) on the pulley's Ø44 core, its front flange on the wall | the forearm wall's station |
| `z_cup` 37.6 … 38.6 | the pulley's Ø33 shoulder on bearing 2's inner ring | `run_gap` |
| `z_meet` 29.6 … 37.6 | the pulley's hub (Ø30.3 journal) through bearing 2 | the lip's middle |
| `z_bearing_1` 21.6 … `z_meet` 29.6 | the shaft's hub (Ø30.3 journal) through bearing 1, its end on the pulley's | the lip's middle |
| `z_shoulder_1` 20.6 … 21.6 | the shaft's Ø33 shoulder on bearing 1's inner ring | `run_gap` |
| `z_shaft_end` 11.75 … 20.6 | the shaft's Ø40 flange: the rotor clamp's 4 nuts 12.75…15.15 (pockets open to its rear face and the bore, `clamp_nut_pocket`), the **stop lug** at +X (`stop_lug_r`, `stop_lug_t` from the rear face; contact at ±`stop_deg` = 180 − 10) | the clamp's tips (`clamp_screw_len` from `z_clamp_head`) |

Ratio 90 / 20 = 4.5 (`FOREARM_ROLL_RATIO`) → ≈ 1.1–1.8 N·m at the roll from the 40 mm kit motor, against a worst-case
static load of ≈ 0.9 N·m (forearm horizontal, wrist 90°, roll 90°, ≈ 0.5 kg wrist + gripper + 0.2 kg payload). The
shoulder (`cycloidal_drive.md` §7) carries ≈ 0.35 kg more beyond the elbow. The elbow's torque path is the elbow 90T →
4× M4 into captive nuts → the frame → the tower → the 6806 pair → the rotor → the forearm wall: the frame carries the
forearm's load 60.9 mm across from the elbow flange, and the pair, 9 mm apart centre to centre, the forearm's bending
moment (`docs/open_issues.md`: check both on the print).

## 3. Fits and printing
PETG, the drive's rules (`cycloidal_drive.md` §6): the seats are the bearing OD + `seat_add` 0.15 (a printed hole
comes out undersize — print a fit gauge first), the journals `journal_add` 0.3 interference. Tests
(`tests/forearm/test_roll_drive.py`) hold the journal press fit at 0.9–1.0 × 99 mm³ per bearing, 0 in the seats,
every other pair in the module below 1 mm³, the rotor `run_gap` off the frame, the two hubs face to face in the lip,
and the frame's stub end ON the elbow pulley's face (`tests/test_mounts.py test_bearing_stacks` holds the elbow's 6806
stack). Print the frame front face down (the tower's seats and the cup print as vertical bores, the round end and the
pocket's rim as walls; supports under the front face round the cup, under the elbow flange standing off the underside
and across the pocket's and the bay's open sides - a `dfam-check` pass before the first print, `docs/open_issues.md`),
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
   into the cup); the shaft + bearing 1 into it from the bay (through the +N opening, then along the axis), its hub's
   end onto the pulley's.
5. Four M3 nuts into the shaft's flange pockets from its rear face (in the bay); 4× M3 × 35 (the rotor clamp) from the
   pulley's front face, the heads into its counterbores, through both hubs into the nuts: the rotor turns, whole.
6. The 240-2GT belt round the ring; the motor into the pocket from the +N face onto the plate (4× M3 through the
   tension slots from the front face, connector toward +N), the 20T on its shaft with its teeth level with the ring,
   in the belt; slide the motor down the slots to tension.
7. Cables from the forearm through the bore, out into the bay and on toward the upper arm.
8. Four M3 nuts into the pulley's core pockets from its cable bore (7–9 mm in from its front face, tweezers), each
   pushed out against the pocket's end. Then the forearm wall onto the spigot (2 mm into the recess) BEFORE the
   wrist-pitch motor goes on: 4× M3 × 16 from the wall's wrist face through the spigot into the nuts; the bottom one
   lays into the channel under the web from the belt side, its key through the channel or the motor slot. A screw taken
   right out can drop its nut into the bore. The stop lug on the shaft's flange meets the frame's post at
   ±`FOREARM_ROLL_LIMIT_DEG`.

## 5. Attachment to the arm (`robot/`)
`elbow_link` = `gt2_pulley_90t#3` (the re-seated elbow 90T) + its screws and nuts + `forearm_roll_drive#1:stator` (the
elbow's driven side: the frame whose underside is the coupler, both bearings, the motor + board on the elbow axis, the
20T); `forearm_link` = `forearm_roll_drive#1:rotor` (the pulley and the shaft) + `j2_link` + the wrist-pitch motor.
`Joint("forearm_roll")` has Z along the forearm and X = N (its child's long direction IS the axis); its origin sits
`ELBOW_ROLL_OFFSET` across `elbow_link`'s frame (`robot/frames.py FOREARM_ROLL_ORIGIN` from `j2_link#1`'s shifted
origin, `ELBOW_ORIGIN` from the retired `j3_coupler#1`'s). The forearm side of the interface is
`lib/forearm/params.py RollEndParams`: the wall at `wall_x` (−56…−48), round about the roll axis (`wall_od`), its Ø40
recess, the Ø31 bolt circle (the screws' nuts in the pulley) and the Ø24 cable bore, on the web's neck (`neck_half_w`,
tangent to the wrist boss) with two gussets — `j2_link`'s DEFAULT build (`lib/forearm/link.py roll_wall`,
`necked_web`, `wall_gussets`). In `arm.py` the module is kept whole under the `elbow_link` group. The wrist-pitch motor
sits where the stock wrist belt (`WRIST_BELT_LENGTH`) puts it (`J2_MOTOR_SLIDE_X`) so its plug clears the wall
(`plug_clearance`).

## 6. Not modelled / to confirm
`docs/open_issues.md`: the belts, the home sensor's mount (the modelled `ky003_hall_sensor`'s chip beside the stop
post on the tower, a magnet in the stop lug, to the MKS board's limit input), the cable route through the Ø18 bore; the
elbow belt's plane; the 6806 seats' PETG fit; the frame's print orientation and its flex under the forearm's load, the
6806 pair's tilt play at the roll, the rotor clamp's thin walls and its nut pockets' fit; `t20_hub`, the belt lengths,
the limit and the printed stop's strength; the elbow offset in a future IK; the room to lift the forearm further back
than −47°; the roll motor's **CAN id** (`software/control/src/config.py` has no row for it: it names fewer boards than
the arm carries).
