# The forearm roll drive — the arm's 6th joint

**Purpose:** spec and attachment of the belt-driven forearm roll (`forearm_roll`, between `elbow_pitch` and
`wrist_pitch`), the way `cycloidal_drive.md` documents the shoulder. Code: `lib/forearm/` (`params.py
RollEndParams` + `RollDriveParams`, `layout.py stack_positions`, `roll.py`, `pulley.py`),
`assemblies/forearm_roll_drive.py`, `parts/joints/forearm_roll_*.py` + `bearing_6808.py`; tests `tests/forearm/`.
**Last updated:** 2026-09-23 (M5: the layout of the diagram — motor up in the swing plane, the ring inside the housing, an end cap).
Numbers below name the constants; the values live in `lib/forearm/params.py`.

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
**+X = host +Z** (N, the top-face side), **+Y = host +Y = up in the arm's swing plane** (the motor side). Every
station: `stack_positions()`. The drive looks like the cutaway in the original diagram: the 90T ring inside the
housing between the two bearings, the motor on top of the block in the side view.

| station (module z) | what | fixed by |
|---|---|---|
| −45…45 (x −25…−6) | the elbow disc — the SolidWorks `j2_link` disc's `j3_coupler#1` interface (Ø54.89 bore, 4× M4 into hex nut pockets from the top), re-expressed in the module frame | the coupler; the block bolts on exactly as `j2_link` did |
| `z_end` 40…43 | the housing's closed elbow end (`end_wall`), with the cable window through its +X wall (`cable_window_w` × `cable_window_len`) | ≥ 1 mm past the disc's (−35, 0) nut pocket; clear of the coupler |
| `z_lip` 43…46 | the lip (ID `lip_id` 46) bearing 1's outer race stops on | PETG lip ≥ 3 |
| `z_seat` 46…53 | bearing 1's seat, Ø `bearing_od` + `seat_add` (52.15), reached through the cavity | the shaft goes in from the wrist end with bearing 1 already on it |
| `z_cavity` 53…70 | the **Ø62 cavity** (`cavity_dia`) the ring runs in: the shaft's Ø44 shoulder 1 53…56.8 (`ring_gap`), the **flanged 90T ring** 56.8…66.2 (teeth 58…65, `z_ring_mid` 61.5, two Ø59.19 flanges — the 20T is flanged on its hub side only), shoulder 2 66.2…70 | 1.4 mm round the flanges, 2 mm of wall under the flat |
| 55.8…67.2 | the **belt window** through the +Y wall (`belt_window_half_x` ±22, `belt_window_y` 22…36): the belt's two runs cross the wall at y 26…31 | the tangent points on the ring at ±26 |
| 58…70 | two **cap lugs** on the housing at (+`lug_y`, 0) and (0, −`lug_y`) — the top-face side and below in the swing plane, never −X (the upper arm's slab) — M3 self-tapped `cap_tap_depth` deep | `lug_w` × `lug_len` from inside the wall (`lug_root`) |
| whole length | the housing Ø `housing_od` 70 with a **flat** on its underside at `flat_x` = −33 (host z −8) | 0.5 mm above the upper arm's slab (host z −8.5), which swings under the forearm when the elbow folds |
| `z_face` 70 | the housing's wrist face = bearing 2 = the **end cap** (`forearm_roll_retainer`, 70…79: seat 2 Ø52.15 70…77, lip 2 ID 46 77…79, the same flat, two ears, the **stop post** on its outer face 79…82 at −X, r 24…30, 10° wide) | pull-out (+Z): shoulder 2 → bearing 2 → cap lip → 2× M3 |
| `z_neck` 77…88 | the shaft's Ø38 neck (`neck_od` < the 6808 bore: bearing 2 slides over it), the **stop lug** on it 81…84 at +X, r 17…28, 10° wide (1 mm of overlap with the post; contact at ±`stop_deg` = 180 − 10) | `stop_*` |
| `z_wall` 88…90 | the shaft's **Ø39.7 end spigot** (`RollEndParams.flange_dia`) in the wall's Ø40 recess; 4× M3 self-tapped `end_bolt_depth` into its end wall on Ø32 (`bolt_circle_dia`); the Ø24 bore (`bore` = the wall's `cable_bore`) end to end | no separate flange: everything beyond journal 2 must pass the bearing's Ø40 bore |
| `z_motor_face` 47.05 | the roll motor's mounting face at (**x `motor_x` 18, y `motor_y` ≈ 58.2**): body toward the elbow (7.55…47.05, host z 22…64 — 1 mm above the disc's top face), the MKS board −6.45…7.55; the 3 mm pad plate ABOVE the face (`pad_t`, tension slots ±2.5 along the axis→motor direction, the Ø22.3 pilot slot), the 20T's hub face `pulley_lift` above the plate, its teeth level with the ring (`t20` = `pad_t` + `pulley_lift` + `t20_hub`) | `centre_distance` = what a `roll_belt` 240-2GT sets (60.9); `motor_y` = √(C² − `motor_x`²) |

Shaft: Ø40.3 journals (`bearing_bore` + `journal_add`, +0.3 in the inner races like the drive's hub) either side
of the ring, Ø44 shoulders (`shoulder_od` < the inner race's edge, `inner_race_od` [ESTIMATE]). Ratio 90 / 20 =
4.5 (`FOREARM_ROLL_RATIO`) → ≈ 1.1–1.8 N·m at the roll from the 40 mm kit motor, against a worst-case static load
of ≈ 0.9 N·m (forearm horizontal, wrist 90°, roll 90°, ≈ 0.5 kg wrist + gripper + 0.2 kg payload). The shoulder
(`cycloidal_drive.md` §7) carries ≈ 0.35 kg more beyond the elbow.

## 3. Fits and printing
PETG, the drive's rules (`cycloidal_drive.md` §6): the seats are the bearing OD + `seat_add` 0.15 (a printed hole
comes out undersize — print a fit gauge first), the journals `journal_add` 0.3 interference. Tests
(`tests/forearm/test_roll_drive.py`) hold the journal press fit at 0.9–1.0 × 132 mm³ per bearing, 0 in the seats,
and every other pair in the module below 1 mm³. Print the housing elbow-face down (the disc flat on the bed, the
housing rising, the pad tower needs support), the shaft spigot down (the ring's grooves print vertically), the
cap flat.

## 4. Assembly sequence
1. Drop the 4 M4 nuts into the housing's disc pockets; bolt the housing to `j3_coupler#1` (the same bolts `j2_link`'s
   disc used).
2. Press bearing 1 onto the shaft's elbow-end journal up to shoulder 1.
3. Slide the shaft + bearing 1 into the housing from the wrist end, through the cavity, the outer race to the lip.
4. Slide bearing 2 over the spigot and the neck and press it onto journal 2 against shoulder 2; the cap over it
   (its seat on the outer race), 2× M3 into the lugs.
5. Motor onto the pad (4× M3 through the tension slots), the 20T on its shaft with its teeth level with the ring,
   the 240-2GT belt through the window; slide the motor along the slots to tension.
6. Cables from the forearm through the bore, out of the housing's +X window.
7. Forearm wall onto the spigot (2 mm into the recess), 4× M3 × 20 from the wall's wrist face into the shaft's end.
   The stop lug on the neck meets the cap's post at ±`FOREARM_ROLL_LIMIT_DEG`.

## 5. Attachment to the arm (`robot/`)
`elbow_link` = `gt2_pulley_90t#1` + `j3_coupler#1` + `forearm_roll_drive#1:stator` (the elbow's driven side
carrying the housing, both bearings, the end cap, the motor + board, the 20T); `forearm_link` =
`forearm_roll_drive#1:rotor` (the shaft) + `j2_link` + the caps + the wrist-pitch motor. `Joint("forearm_roll")`
has Z along the forearm and X = N (its child's long direction IS the axis). The forearm side of the interface is
`lib/forearm/params.py RollEndParams`: the wall at `wall_x` (−96…−88), its Ø40 recess, the Ø32 bolt circle and the Ø24 cable bore —
`j2_link`'s DEFAULT build (`lib/forearm/link.py roll_wall`). In `arm.py` the module is kept whole under the
`elbow_link` group. The wrist-pitch motor moved to where a 264-2GT belt puts it (`J2_MOTOR_SLIDE_X`) so its plug
clears the wall (`plug_clearance`).

## 6. Not modelled / to confirm
`docs/open_issues.md`: the belts, the home sensor (on the cap's outer face, a magnet in the stop lug, to the MKS board's
limit input), the cable route; the 6808's mass and inner-race edge, `t20_hub`, the belt lengths, the limit and the
printed stop lugs' strength, the spigot's self-tapped M3s; a **4th CAN id** (`src/config.py` names three boards, the
arm carries five).
