# The forearm roll drive — the arm's 6th joint

**Purpose:** spec and attachment of the belt-driven forearm roll (`forearm_roll`, between `elbow_pitch` and
`wrist_pitch`), the way `cycloidal_drive.md` documents the shoulder. Code: `lib/forearm/` (`params.py
RollEndParams` + `RollDriveParams` + `ForearmConfig.elbow_offset`, `layout.py stack_positions`, `roll.py`),
`assemblies/forearm_roll_drive.py`, `parts/joints/forearm_roll_*.py` + `bearing_6808.py`; tests `tests/forearm/`.
Numbers below name the constants; the values live in `lib/forearm/params.py`.

## 0. Specifications at a glance
| item | value | where |
|---|---|---|
| joint | `forearm_roll`, revolute, `elbow_link → forearm_link`; axis along the forearm through the wrist centre, `ELBOW_ROLL_OFFSET` (60.9, the roll belt's centre distance) across the elbow axis - **an elbow offset: the two axes do not cross** -, `FOREARM_ROLL_AXIS_Z` along N from `j2_link`'s origin | `robot/frames.py`, `lib/placements.py SHIFTS` |
| range | ±170° (`FOREARM_ROLL_LIMIT_DEG`), printed hard stop (lug on the shaft's neck, post on the cap); home sensor modelled (`ky003_hall_sensor`), not placed | `lib/params.py`, `RollDriveParams stop_*` |
| drive | NEMA 17 × 40 mm + MKS SERVO42D (a 4th CAN id) **on the elbow axis**, GT2 20T on the motor, integral printed 90T ring on the shaft, **4.5 : 1**, 240-2GT × 6 mm belt (`roll_belt`, centre distance 60.9 = the offset) | `RollDriveParams`, `lib/belts.py` |
| torque | ≈ 1.1–1.8 N·m at the roll vs ≈ 0.9 N·m worst-case static load | §2 |
| bearings | 2× 6808-2RS (40 × 52 × 7), 57 mm apart, straddling the elbow axis' station (seats Ø52.15, journals Ø40.3) | `RollDriveParams bearing_*` |
| shaft (rotor, PETG) | hollow, Ø24 cable bore end to end, Ø44 core, Ø38 neck, Ø39.7 end spigot; the forearm wall's 4× M3 on Ø31 through its end into M3 nuts in pockets behind bearing 2 (slots from the bore, `end_nut_pocket`); 72 mm long (z −22…50) | `forearm_roll_shaft` |
| frame (stator, PETG) | ONE printed part, **also the elbow's output flange**: the **housing** round the roll axis, a 66 × 72 × 62 mm rounded box (x −33…33, y ±36, z −26…36, r 8; Ø26 cable exit in its rear wall, the belt window in its bottom wall); the **web** under the motor on the upper arm's side (x −33…−23, `web_t`; from the housing down past the elbow axis, z −40…36, `web_z`) whose underside carries the coupler's Ø72 lip, Ø62 boss, Ø40 journal, Ø33 inner-ring shoulder and Ø30 stub about the elbow axis, down through `j1_link`'s bore to the elbow 90T (host z −25…−8), 4× M4 at r 11 (turned 45°) for the elbow 90T into captive nuts in hex channels that open up into the motor's cradle; the **plate** in front of the motor (`pad_t`, `plate_w` wide, the tension slots, the Ø22.3 pilot slot) | `forearm_roll_block` |
| end cap (PETG) | the housing's outline, 9 thick (seat + 2 lip), 4× M3 at the corners, the stop post | `forearm_roll_retainer` |
| forearm interface | `j2_link`'s wall at 48…56 mm along the roll axis (`wall_x` −56…−48): a Ø60 round flange on the roll axis (`wall_od`) on a foot as wide as itself, the web necked down to it from the wrist boss (`neck_half_w`), two gussets beside the wrist motor (`rib_*`); Ø40 × 2 recess, Ø24 bore, 4× M3 × 25 on Ø31 from its wrist face (`screw`, `screw_len`; the bottom one's head in a channel under the web, `screw_channel`) | `RollEndParams` |
| clearances held by tests | the frame's underside (the housing's and the web's, one plane) and the end cap 3.0 mm or more above `j1_link`'s flat top (the relief's floor, `relief_y`, end to end), the boss 2.0 above its recess floor, level with the upper 6806's top (`FACE_GAP`: the least gap a face the elbow turns may keep; minimum gaps, not overlap volumes); the roll motor 2 mm over the web and 1 mm or more under the housing at the slot's top end; folded elbow (to `ELBOW_PITCH_LIMITS_DEG`, −47° / +90°) and rolled forearm (±170°) < 1 mm³ against every neighbour; at the elbow's limits the forearm, rolled anywhere, ≥ 2 mm off `j1_link`; over the elbow's range the drive ≥ 1 mm off the elbow motor on the upper arm's same side (the lower limit is 3° short of where the housing's rear end comes that near; the roll motor, on the elbow axis, never does); journals 0.9–1.0 × the 132 mm³ press | `tests/forearm/test_roll_drive.py` |
| link masses (URDF) | `elbow_link` carries the pulley + stator, `forearm_link` the shaft + forearm + wrist motor; the masses are the `<inertial>` blocks (`tools/robot/derive.py`), their sum checked by `test_link_masses_add_up` | `robot/arm.urdf` |
| purchased per drive | the 6808-2RS pair, a NEMA 17 × 40 kit + MKS SERVO42D, a GT2 20T (5 mm bore), the roll belt, the motor's and the cap's M3 screws and the forearm wall's M3 screws + nuts — with quantities: `./cadtool python tools/bom.py --module forearm_roll_drive`; the elbow 90T's M4 screws and nuts are the arm's (`elbow_pulley_screws`, `elbow_pulley_nuts`) | `tools/bom.py` |
| printed per drive | frame, shaft, cap (`./cadtool python tools/export_printables.py --parts forearm_roll_block forearm_roll_shaft forearm_roll_retainer`) | `print/` |

## 1. Why a roll, and where
The SolidWorks arm had five revolute joints — `base_yaw` and three parallel pitches (`shoulder_pitch`,
`elbow_pitch`, `wrist_pitch`) plus `wrist_roll` — so the tool's approach axis could never leave the vertical
plane through the base axis. `wrist_pitch` and `wrist_roll` already met at one point, the **wrist centre**
(`robot/frames.py WRIST_CENTRE`: `WRIST_CENTRE_ALONG_N` = 17 mm back along the pitch axis from the wrist_pitch
origin, 5 µm off both axes). A roll about the forearm through that point makes the last three axes concurrent —
the standard 6R arm with a spherical wrist and a closed-form IK. The axis runs along `ELBOW_TO_WRIST_INPLANE`,
`FOREARM_ROLL_AXIS_Z` = 42 − 17 = 25 mm along N from `j2_link`'s origin — in `j2_link`'s frame: y 0, z 25, along −X.

**The elbow offset.** The roll motor sits ON the elbow axis and the roll housing above it, the roll belt's centre
distance away (`ForearmConfig.elbow_offset` = `RollDriveParams.centre_distance`, `ELBOW_ROLL_OFFSET`): so the
forearm, the wrist and the gripper sit that far across the elbow axis - along `j2_link`'s +Y, up in the arm's swing
plane -, moved rigidly from the capture (`lib/placements.py SHIFTS`: `j2_link#1` and every record beyond it; the
elbow's own records stay). The arm gains an elbow offset (the roll axis and the elbow axis no longer cross; the wrist
stays spherical); in exchange the forearm lifts back to `ELBOW_PITCH_LIMITS_DEG`'s −47° (it stopped at −21° when the
motor hung behind the elbow and its board met the elbow motor) and folds with more room to spare. In `j2_link`'s frame
the elbow axis runs along Z through y = −`elbow_offset`.

## 2. Layout (module frame)
Module frame (`lib/forearm/layout.py module_frame_in_host`, `lib/mounts.py MODULE_MOUNTS` on `j2_link#1`):
origin on the roll axis at the elbow axis' station, **+Z along the roll axis toward the wrist** (host −X),
**+X = host +Z** (N, away from the upper arm), **+Y = host +Y = up in the arm's swing plane** (the housing above the
motor); the **elbow axis runs along X at y = `y_elbow`** (−`elbow_offset`). Every station: `stack_positions()`.

**The frame is one printed part that is also the elbow's output flange.** The measured SolidWorks coupler
(`reference/solidworks/j3_coupler.step`, 2026-09-23: Ø78 flange host z −10…0, Ø62 boss −14…−10, Ø40 journal
−15.3…−14, Ø30 stub −22…−15.3, Ø12.5 bore, 4× M4 at r 11 from the pulley) is repeated under the frame's web about the
elbow axis, the elbow 90T bolts straight into it (through the elbow's 6806 pair, §4), and `j3_coupler#1` is **retired**
(`lib/placements.py RETIRED`: the record stays, no table claims it; the wrist's `j3_coupler#2` is untouched). The
motor sits in a cradle between the web (under it, on the upper arm's side), the plate (in front of it) and the
housing's bottom wall (over it); its 20T drives the ring straight up through the window in that wall.

| station | what | fixed by |
|---|---|---|
| `block_z` −26…36 | the housing: a rounded box (`block_x` −33…33, `block_y` ±36, r 8 edges along Z) round the roll axis; its underside (−X, host z −8) rides 3.0 mm above `j1_link`'s flat top | that top (host −11, `lib/upper_arm/params.py ElbowParams.relief_y`); 2 mm of wall under the cavity; the bearings 57 apart, the rear end what the elbow motor meets first as the forearm lifts back (`ELBOW_PITCH_LIMITS_DEG`) |
| the web, x −33…−23, y `y_web_low` −98.9 … into the housing, `web_z` −40…36 | under the motor on the upper arm's side, from the housing down past the elbow axis: its underside the housing's plane, its top (`x_cradle` −23) the cradle's floor | 2 mm under the motor's −N side; over the lip about the elbow axis (`lip_dia` / 2 either side, `web_margin` past it) |
| the elbow flange, x −50…−33 about (y `y_elbow`, z 0) | `lip_dia` Ø72 lip −35…−33 in `j1_link`'s Ø80 recess, `boss_dia` Ø62 −39…−35, `journal_dia` Ø40 −40.3…−39, `step_dia` Ø33 −41…−40.3 (the shoulder on the upper 6806's inner ring), `stub_dia` Ø30 −50…−41 (in the upper of the elbow's 6806-2RS pair and on through the lip, `j1_link`'s bore: `lib/mounts.py bearing_6806#3` / `#4`), the Ø12.5 `pin_bore_dia` (from the stub's end, blind at −37: 1 mm under the nut seats); the elbow 90T's **4× M4 at `pulley_bolt_r` 11, turned `pulley_bolt_deg` 45°** off the axes: Ø4.4 clearance −50…−36, then each nut's **hex channel** (`nut_af` 6.85, a flat toward the elbow axis) from its seat `nut_seat_x` −36 up through the web into the cradle | the measured coupler, its stub `PULLEY_SEAT_SHIFT` (3) longer: its end (host −25, the lip's lower face) sits on the re-seated pulley's face; the nuts (−36…−32.8) and the screw tips (−31.4) ≥ 2 mm under the cradle's floor |
| `y_motor` = `y_elbow` | the roll motor ON the elbow axis, centred on the roll axis in X: its mounting face `z_motor_face` 6.05, the body −33.45…6.05, the MKS board −47.55…−33.45, spun `motor_spin_deg` 90° so the connector points +X; the 20T's hub face `pulley_lift` in front of the plate, its teeth level with the ring (`t20`) | `centre_distance` = what a `roll_belt` 240-2GT sets; the body 3.9 mm under the housing at the nominal slot position, 1.4 at the slot's top end |
| `z_motor_face` 6.05 … `z_pad_top` 10.05 | the **plate** (x −33…23, from `y_plate_low` −86.4 up into the housing's bottom wall): the 4 tension slots ±2.5 along Y on the motor's 31 mm square, the Ø22.3 pilot slot | the motor's face; `plate_w` = the motor + 4 |
| `z_end` −26…−23 | the rear end wall, the Ø26 `cable_exit` on the axis | the shaft's Ø24 bore + 1 |
| `z_lip` −23…−21 | the lip (ID `lip_id` 46) bearing 1's outer race stops on | |
| `z_seat` −21…−14 | **bearing 1**'s seat Ø52.15; the shaft's rear journal from −22 (`shaft_end_clear`) | |
| −14…16 | the Ø52.6 `core_bore_dia`: bearing 1 (pressed on the shaft) rides through it to its seat, the Ø44 core turns in it | |
| `z_cavity` 16…36 | the **Ø62 cavity**, OPEN through the front face: the shaft's flanged 90T ring 16.8…26.2 (teeth 18…25, `z_ring_mid` 21.5) passes through it on assembly | `cavity_z0` 0.8 before the ring's first flange |
| 15.8…27.2 | the **belt window** through the bottom wall (`belt_window_half_x` ±22, from y −29 down) | the runs cross the wall at \|x\| 16…19 |
| `z_end_nut` 32…34.4 | in the Ø44 core, the forearm wall's four **M3 nuts** (`end_nut`), each in a pocket that runs out from the cable bore along its screw's angle (a flat either side, `end_nut_fit` past the nut's outer corner); the screws' tips at `z_end_tip` 31, 2 pitches past them | 3.3 mm of the core outside each nut; 1.6 mm before journal 2; the screws clamp the spigot, the neck and journal 2 (21.6 mm of PETG) between head and nut |
| `z_face` 36 | the front face = **bearing 2** = the **end cap** (`forearm_roll_retainer`, 36…45: seat 36…43, lip 43…45, the housing's outline, 4× M3 at (±27, ±30) self-tapped `cap_tap_depth` into the face, the **stop post** 45…47.5 at −X, r 24…30) | pull-out (+Z): core → bearing 2 → cap lip → 4× M3 |
| `z_neck` 43…48 | the shaft's Ø38 neck (bearing 2 slides over it), the **stop lug** on it 45…47.5 at +X, r 17…28 (contact at ±`stop_deg` = 180 − 10) | `stop_t` 0.5 mm short of the wall |
| `z_wall` 48…50 | the shaft's **Ø39.7 spigot** (`RollEndParams.flange_dia`) in the wall's Ø40 recess; the **forearm wall 48…56** (`wall_x` −56…−48), round (Ø60, over the stop lug and post); 4× M3 × 25 on Ø31 from its wrist face (`z_wall_back` 56) through the spigot, the neck and journal 2 to the nuts | the rolling forearm clears `j1_link`'s r 45 end; Ø31 centres each Ø3.4 hole in the neck's wall (1.8 mm either side) |

Shaft: Ø40.3 journals (`bearing_bore` + `journal_add`) at both ends of the Ø44 core (`shoulder_od` < the inner
race's edge, `inner_race_od` [ESTIMATE]), the bearings 57 mm apart straddling the elbow axis' station. Ratio 90 / 20 =
4.5 (`FOREARM_ROLL_RATIO`) → ≈ 1.1–1.8 N·m at the roll from the 40 mm kit motor, against a worst-case static load of
≈ 0.9 N·m (forearm horizontal, wrist 90°, roll 90°, ≈ 0.5 kg wrist + gripper + 0.2 kg payload). The shoulder
(`cycloidal_drive.md` §7) carries ≈ 0.35 kg more beyond the elbow. The elbow's torque path is the elbow 90T → 4× M4
into captive nuts → the frame's web → the housing → the bearings → the shaft → the forearm wall: the web carries the
forearm's load 60.9 mm across from the elbow flange (`docs/open_issues.md`: check it for flex on the print).

## 3. Fits and printing
PETG, the drive's rules (`cycloidal_drive.md` §6): the seats are the bearing OD + `seat_add` 0.15 (a printed hole
comes out undersize — print a fit gauge first), the journals `journal_add` 0.3 interference. Tests
(`tests/forearm/test_roll_drive.py`) hold the journal press fit at 0.9–1.0 × 132 mm³ per bearing, 0 in the seats,
every other pair in the module below 1 mm³, and the frame's stub end ON the elbow pulley's face
(`tests/test_mounts.py test_bearing_stacks` holds the elbow's 6806 stack). Print the frame front face down (the
housing's seat, core bore and cavity print as vertical bores, the web a vertical wall; the plate is a shelf across the
cradle and the coupler stub / journal / boss stand off the web's side - both need support: a `dfam-check` pass before
the first print, `docs/open_issues.md`), the shaft spigot down (the ring's grooves print vertically; the wall
screws' nut pockets are flat slots whose roofs bridge the nut's 5.5 across flats), the cap flat.

## 4. Assembly sequence
1. Four M4 nuts (`elbow_pulley_nuts`) into the web's hex channels from the motor's cradle (before the motor), down onto
   their seats - the channel's 6.85 across flats holds each.
2. The elbow's 6806-2RS pair into `j1_link`'s bore, one each side of the lip; the frame's stub into the upper one
   (its Ø33 shoulder on the inner ring, its end through the lip), the elbow 90T's hub into the lower one from below
   (its Ø34.76 ring on that inner ring, its end on the stub's end): 4× M4 × 35 (`elbow_pulley_screws`, the heads in
   the pulley's counterbores, `GT2_PULLEY_90T_HEAD_SEAT` under its face) up through the pulley's hub (its holes opened
   to `M4_CLEAR`), the stub, the journal and the boss into the nuts (the tips 1.4 mm past them) bolt the pulley flat
   onto the stub and clamp both inner rings - the stub's 2 mm in the lip spaces them as the lip spaces the outer rings.
   The hex holds each nut, so the screws re-tighten from the pulley side at any time (with the motor out, a screw
   taken right out can drop its nut into the cradle).
3. Press bearing 1 onto the shaft's rear journal against shoulder 1.
4. Slide the shaft + bearing 1 into the housing from the front, through the cavity and the core bore, the outer race to
   the lip; the ring ends in the cavity.
5. Slide bearing 2 over the spigot and the neck and press it onto journal 2 against the core; the cap over it (its
   seat on the outer race), 4× M3 into the front face.
6. The 240-2GT belt round the ring and down through the window; the motor into the cradle onto the plate (4× M3
   through the tension slots, connector toward +N), the 20T on its shaft with its teeth level with the ring, in the
   belt; slide the motor down the slots to tension.
7. Cables from the forearm through the bore, out of the rear end wall toward the upper arm.
8. Four M3 nuts into the shaft's pockets from its cable bore (16–18 mm in from its end, tweezers), each pushed out
   against the pocket's end. Then the forearm wall onto the spigot (2 mm into the recess) BEFORE the wrist-pitch motor
   goes on (a screw with its head is 28 mm long, the motor's body 22.6 mm from the wall): 4× M3 × 25 from the wall's
   wrist face through the shaft's end into the nuts; the bottom one lays into the channel under the web from the belt
   side, its key through the channel or the motor slot. A screw taken right out can drop its nut into the bore.
   The stop lug on the neck meets the cap's post at ±`FOREARM_ROLL_LIMIT_DEG`.

## 5. Attachment to the arm (`robot/`)
`elbow_link` = `gt2_pulley_90t#3` (the re-seated elbow 90T) + its screws and nuts + `forearm_roll_drive#1:stator` (the
elbow's driven side: the frame whose web is the coupler, both bearings, the end cap, the motor + board on the elbow
axis, the 20T); `forearm_link` = `forearm_roll_drive#1:rotor` (the shaft) + `j2_link` + the wrist-pitch motor.
`Joint("forearm_roll")` has Z along the forearm and X = N (its child's long direction IS the axis); its origin sits
`ELBOW_ROLL_OFFSET` across `elbow_link`'s frame (`robot/frames.py FOREARM_ROLL_ORIGIN` from `j2_link#1`'s shifted
origin, `ELBOW_ORIGIN` from the retired `j3_coupler#1`'s). The forearm side of the interface is
`lib/forearm/params.py RollEndParams`: the wall at `wall_x` (−56…−48), round about the roll axis (`wall_od`), its Ø40
recess, the Ø31 bolt circle (the screws' nuts in the shaft) and the Ø24 cable bore, on the web's neck (`neck_half_w`,
tangent to the wrist boss) with two gussets — `j2_link`'s DEFAULT build (`lib/forearm/link.py roll_wall`,
`necked_web`, `wall_gussets`). In `arm.py` the module is kept whole under the `elbow_link` group. The wrist-pitch motor
sits where the stock wrist belt (`WRIST_BELT_LENGTH`) puts it (`J2_MOTOR_SLIDE_X`) so its plug clears the wall
(`plug_clearance`).

## 6. Not modelled / to confirm
`docs/open_issues.md`: the belts, the home sensor's mount (the modelled `ky003_hall_sensor`'s chip on the cap's outer face, a magnet in the stop lug, to the MKS board's
limit input), the cable route; the elbow belt's plane; the 6806 seats' PETG fit; the frame's print orientation, its
web's flex under the forearm's load and its nut channels' fit; the 6808's mass and inner-race edge, `t20_hub`, the
belt lengths, the limit and the printed stop lugs' strength, the spigot's joint (its screws' nut pockets' fit, its
creep); the elbow offset in a future IK; the roll motor's **CAN id** (`software/control/src/config.py` has no row for
it: it names fewer boards than the arm carries).
