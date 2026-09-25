# The forearm roll drive — the arm's 6th joint

**Purpose:** spec and attachment of the belt-driven forearm roll (`forearm_roll`, between `elbow_pitch` and
`wrist_pitch`), the way `cycloidal_drive.md` documents the shoulder. Code: `lib/forearm/` (`params.py
RollEndParams` + `RollDriveParams`, `layout.py stack_positions`, `roll.py`, `pulley.py`),
`assemblies/forearm_roll_drive.py`, `parts/joints/forearm_roll_*.py` + `bearing_6808.py`; tests `tests/forearm/`.
Numbers below name the constants; the values live in `lib/forearm/params.py`.

## 0. Specifications at a glance
| item | value | where |
|---|---|---|
| joint | `forearm_roll`, revolute, `elbow_link → forearm_link`; axis along the forearm through the wrist centre, crossing the elbow axis 25 mm along N (`FOREARM_ROLL_AXIS_Z`) | `robot/frames.py` |
| range | ±170° (`FOREARM_ROLL_LIMIT_DEG`), printed hard stop (lug on the shaft's neck, post on the cap); home sensor not modelled | `lib/params.py`, `RollDriveParams stop_*` |
| drive | NEMA 17 × 40 mm + MKS SERVO42D (a 4th CAN id), GT2 20T on the motor, integral printed 90T ring on the shaft, **4.5 : 1**, 240-2GT × 6 mm belt (`roll_belt`, centre distance 60.9) | `RollDriveParams`, `lib/belts.py` |
| torque | ≈ 1.1–1.8 N·m at the roll vs ≈ 0.9 N·m worst-case static load | §2 |
| bearings | 2× 6808-2RS (40 × 52 × 7), 71 mm apart, straddling the elbow axis (seats Ø52.15, journals Ø40.3) | `RollDriveParams bearing_*` |
| shaft (rotor, PETG) | hollow, Ø24 cable bore end to end, Ø44 core, Ø38 neck, Ø39.7 end spigot + 4× M3 on Ø32 into the forearm wall; 86 mm long (z −36…50) | `forearm_roll_shaft` |
| block (stator, PETG) | 66 × 72 × 76 mm rounded box (x −33…33, y ±36, z −40…36, r 8), **also the elbow's output flange**: Ø72 lip, Ø62 boss, Ø40 journal, Ø30 stub down into `j1_link` (host z −22…−8), 4× M4 heat-set inserts at r 11 for the elbow 90T; Ø26 cable exit in the rear wall; the motor plate + cheeks on top | `forearm_roll_block` |
| end cap (PETG) | the block's outline, 9 thick (seat + 2 lip), 4× M3 at the corners, the stop post | `forearm_roll_retainer` |
| forearm interface | `j2_link`'s wall at 48…56 mm from the elbow axis (`wall_x` −56…−48): Ø40 × 2 recess, Ø24 bore, 4× M3 on Ø32 | `RollEndParams` |
| clearances held by tests | block 0.5 mm above the upper arm's slab; the rolling ±45 forearm wall 3 mm off `j1_link`'s r 45 end; folded elbow (±120°) and rolled forearm (±170°) < 1 mm³ against every neighbour; journals 0.9–1.0 × the 132 mm³ press | `tests/forearm/test_roll_drive.py` |
| link masses (URDF) | `elbow_link` 0.717 kg (pulley + stator), `forearm_link` 0.920 kg (shaft + forearm + wrist motor); arm total 5.93 kg | `robot/arm.urdf` |
| purchased per drive | 2× 6808-2RS, 1× NEMA 17 × 40 kit + MKS SERVO42D, 1× GT2 20T (5 mm bore), 1× 240-2GT belt, 4× M4 × 40 + 4× M4 inserts, 4× M3 × 16, 4× M3 × 8, 4× M3 × 20 | `tools/bom.py` |
| printed per drive | block, shaft, cap (`./cadtool python tools/export_printables.py --parts forearm_roll_block forearm_roll_shaft forearm_roll_retainer`) | `print/` |

## 1. Why a roll, and where
The SolidWorks arm had five revolute joints — `base_yaw` and three parallel pitches (`shoulder_pitch`,
`elbow_pitch`, `wrist_pitch`) plus `wrist_roll` — so the tool's approach axis could never leave the vertical
plane through the base axis. `wrist_pitch` and `wrist_roll` already met at one point, the **wrist centre**
(`robot/frames.py WRIST_CENTRE`: `WRIST_CENTRE_ALONG_N` = 17 mm back along the pitch axis from the wrist_pitch
origin, 5 µm off both axes). A roll about the forearm through that point makes the last three axes concurrent —
the standard 6R arm with a spherical wrist and a closed-form IK. The axis runs along `ELBOW_TO_WRIST_INPLANE`
and crosses the elbow axis `FOREARM_ROLL_AXIS_Z` = 42 − 17 = 25 mm along N from the elbow origin — in
`j2_link`'s frame: y 0, z 25, along −X.

## 2. Layout (module frame)
Module frame (`lib/forearm/layout.py module_frame_in_host`, `lib/mounts.py MODULE_MOUNTS` on `j2_link#1`):
origin on the roll axis at the elbow-axis crossing, **+Z along the roll axis toward the wrist** (host −X),
**+X = host +Z** (N, away from the upper arm), **+Y = host +Y = up in the arm's swing plane** (the motor side).
Every station: `stack_positions()`.

**The block is one printed part that is also the elbow's output flange.** The measured SolidWorks coupler
(`reference/solidworks/j3_coupler.step`, 2026-09-23: Ø78 flange host z −10…0, Ø62 boss −14…−10, Ø40 journal
−15.3…−14, Ø30 stub −22…−15.3, Ø12.5 bore, 4× M4 at r 11 from the pulley) has its flange top only 25 mm under the
roll axis — with a block bolted ON it, nothing on the shaft larger than Ø50 (the Ø59 ring, the Ø52 6808s) could
sit within 39 mm of the elbow axis, which is why the M5 housing started 40 mm out and the forearm 88. So the block's
underside now repeats the coupler's lip / boss / journal / stub, the elbow 90T bolts straight into it, and
`j3_coupler#1` is **retired** (`lib/placements.py RETIRED`: the record stays, no table claims it; the wrist's
`j3_coupler#2` is untouched). The shaft then crosses the elbow axis inside the block.

| station (module z) | what | fixed by |
|---|---|---|
| `block_z` −40…36 | the block: a rounded box (`block_x` −33…33, `block_y` ±36, r 8 edges along Z) round the roll axis; its underside (−X, host z −8) rides 0.5 mm above the upper arm's slab | the slab (host −8.5); 2 mm of wall under the cavity |
| underside, x −47…−33 | `lip_dia` Ø72 lip −35…−33 in `j1_link`'s Ø80 recess, `boss_dia` Ø62 −39…−35, `journal_dia` Ø40 −40.3…−39, `stub_dia` Ø30 −47…−40.3 (the elbow's own bearings, 6702-class in `j1_link`'s Ø42 bore — not modelled), the Ø12.5 `pin_bore` (blind at −29); the elbow 90T's **4× M4 at `pulley_bolt_r` 11** on the axes: Ø4.4 clearance −47…−39, **Ø5.6 heat-set inserts −39…−31** | the measured coupler; the stub's end (host −22) sits on the pulley's face |
| `z_end` −40…−37 | the rear end wall, the Ø26 `cable_exit` on the axis | the shaft's Ø24 bore + 1 |
| `z_lip` −37…−35 | the lip (ID `lip_id` 46) bearing 1's outer race stops on | |
| `z_seat` −35…−28 | **bearing 1**'s seat Ø52.15; the shaft's rear journal from −36 (`shaft_end_clear`) | |
| −28…16 | the Ø52.6 `core_bore`: bearing 1 (pressed on the shaft) rides through it to its seat, the Ø44 core turns in it | 4.7 mm over the inserts |
| `z_cavity` 16…36 | the **Ø62 cavity**, OPEN through the front face: the shaft's flanged 90T ring 16.8…26.2 (teeth 18…25, `z_ring_mid` 21.5) passes through it on assembly | `cavity_z0` 2.2 mm past the inserts (z 8.2…13.8) |
| 15.8…27.2 | the **belt window** through the top wall (`belt_window_half_x` ±22, from y 29 out) | the runs cross the wall at \|x\| 16…19 |
| `z_face` 36 | the front face = **bearing 2** = the **end cap** (`forearm_roll_retainer`, 36…45: seat 36…43, lip 43…45, the block's outline, 4× M3 at (±27, ±30) self-tapped `cap_tap_depth` into the face, the **stop post** 45…47.5 at −X, r 24…30) | pull-out (+Z): core → bearing 2 → cap lip → 4× M3 |
| `z_neck` 43…48 | the shaft's Ø38 neck (bearing 2 slides over it), the **stop lug** on it 45…47.5 at +X, r 17…28 (contact at ±`stop_deg` = 180 − 10) | `stop_t` 0.5 mm short of the wall |
| `z_wall` 48…50 | the shaft's **Ø39.7 spigot** (`RollEndParams.flange_dia`) in the wall's Ø40 recess, 4× M3 on Ø32 into its end wall; the **forearm wall 48…56** (`wall_x` −56…−48) | ≥ 45 + 3: the rolling ±45 wall (corners r 57) clears `j1_link`'s r 45 end |
| `z_motor_face` 7.05 | the roll motor's mounting face at (x 0, y `motor_y` 60.9): body −32.45…7.05 (behind the elbow axis, 7.5 before the block's rear), the MKS board −46.55…−32.45, spun `motor_spin_deg` 90° so the connector points +X; the 3 mm **plate** 7.05…10.05 standing on the block's top up to y 83.9 (tension slots ±2.5 along Y, the Ø22.3 pilot slot), two **cheeks** x ±(21.5…24.5), y 35…52; the 20T's hub face `pulley_lift` above the plate, its teeth level with the ring (`t20`) | `centre_distance` = what a `roll_belt` 240-2GT sets; the body 1.9 mm above the top at the nominal slot position |

Shaft: Ø40.3 journals (`bearing_bore` + `journal_add`) at both ends of the Ø44 core (`shoulder_od` < the inner
race's edge, `inner_race_od` [ESTIMATE]), the bearings 71 mm apart straddling the elbow axis. Ratio 90 / 20 = 4.5
(`FOREARM_ROLL_RATIO`) → ≈ 1.1–1.8 N·m at the roll from the 40 mm kit motor, against a worst-case static load of
≈ 0.9 N·m (forearm horizontal, wrist 90°, roll 90°, ≈ 0.5 kg wrist + gripper + 0.2 kg payload). The shoulder
(`cycloidal_drive.md` §7) carries ≈ 0.35 kg more beyond the elbow. The elbow's torque path is now the elbow 90T →
4× M4 in heat-set inserts → the block → the bearings → the shaft → the forearm wall.

## 3. Fits and printing
PETG, the drive's rules (`cycloidal_drive.md` §6): the seats are the bearing OD + `seat_add` 0.15 (a printed hole
comes out undersize — print a fit gauge first), the journals `journal_add` 0.3 interference. Tests
(`tests/forearm/test_roll_drive.py`) hold the journal press fit at 0.9–1.0 × 132 mm³ per bearing, 0 in the seats,
every other pair in the module below 1 mm³, and the block's stub end ON the elbow pulley's face. Print the block
front face down (the seat, the core bore and the cavity print as vertical bores; the coupler stub / journal / boss
on the side and the motor plate need support — a `dfam-check` pass before the first print, `docs/open_issues.md`),
the shaft spigot down (the ring's grooves print vertically), the cap flat.

## 4. Assembly sequence
1. Four M4 heat-set inserts into the block's underside (from the stub side, through the Ø4.4 holes' floor).
2. The block's stub into `j1_link`'s bore (with the elbow bearings), the elbow 90T under it: 4× M4 × 40 up through
   the pulley's hub, the stub, the journal and the boss into the inserts.
3. Press bearing 1 onto the shaft's rear journal against shoulder 1.
4. Slide the shaft + bearing 1 into the block from the front, through the cavity and the core bore, the outer race to
   the lip; the ring ends in the cavity.
5. Slide bearing 2 over the spigot and the neck and press it onto journal 2 against the core; the cap over it (its
   seat on the outer race), 4× M3 into the front face.
6. Motor onto the plate (4× M3 through the tension slots, connector toward +N), the 20T on its shaft with its teeth
   level with the ring, the 240-2GT belt through the window; slide the motor up the slots to tension.
7. Cables from the forearm through the bore, out of the rear end wall toward the upper arm.
8. Forearm wall onto the spigot (2 mm into the recess), 4× M3 × 20 from the wall's wrist face into the shaft's end.
   The stop lug on the neck meets the cap's post at ±`FOREARM_ROLL_LIMIT_DEG`.

## 5. Attachment to the arm (`robot/`)
`elbow_link` = `gt2_pulley_90t#1` + `forearm_roll_drive#1:stator` (the elbow's driven side: the block that is the
coupler, both bearings, the end cap, the motor + board, the 20T); `forearm_link` = `forearm_roll_drive#1:rotor` (the
shaft) + `j2_link` + the wrist-pitch motor. `Joint("forearm_roll")` has Z along the forearm and X = N
(its child's long direction IS the axis). The forearm side of the interface is `lib/forearm/params.py
RollEndParams`: the wall at `wall_x` (−56…−48), its Ø40 recess, the Ø32 bolt circle and the Ø24 cable bore —
`j2_link`'s DEFAULT build (`lib/forearm/link.py roll_wall`); the web gained 40 mm and its −75 socket column. In
`arm.py` the module is kept whole under the `elbow_link` group. The wrist-pitch motor sits where a 264-2GT belt
puts it (`J2_MOTOR_SLIDE_X`) so its plug clears the wall (`plug_clearance`).

## 6. Not modelled / to confirm
`docs/open_issues.md`: the belts, the home sensor (on the cap's outer face, a magnet in the stop lug, to the MKS board's
limit input), the cable route; the elbow's own bearings on the block's stub; the block's print orientation and its
M4 inserts; the 6808's mass and inner-race edge, `t20_hub`, the belt lengths, the limit and the printed stop lugs'
strength, the spigot's self-tapped M3s; a **4th CAN id** (`software/control/src/config.py` names three boards, the arm carries five).
