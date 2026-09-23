# The forearm roll drive — the arm's 6th joint

**Purpose:** spec and attachment of the belt-driven forearm roll (`forearm_roll`, between `elbow_pitch` and
`wrist_pitch`), the way `cycloidal_drive.md` documents the shoulder. Code: `lib/forearm/` (`params.py
RollEndParams` + `RollDriveParams`, `layout.py stack_positions`, `roll.py`, `pulley.py`),
`assemblies/forearm_roll_drive.py`, `parts/joints/forearm_roll_*.py` + `bearing_6808.py`; tests `tests/forearm/`.
**Last updated:** 2026-09-23. Numbers below name the constants; the values live in `lib/forearm/params.py`.

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
**+X = host +Z** (N, the motor side), +Y = host +Y. Every station: `stack_positions()`.

| station (module z) | what | fixed by |
|---|---|---|
| −45…45 (x −25…−6) | the elbow disc — the SolidWorks `j2_link` disc's `j3_coupler#1` interface (Ø54.89 bore, 4× M4 into hex nut pockets from the top), re-expressed in the module frame | the coupler; the block bolts on exactly as `j2_link` did |
| `z_end` 40…43 | the block's closed elbow end (`end_wall`), with the cable window through the tube top (+X, `cable_window_w` × `cable_window_len`) | ≥ 1 mm past the disc's (−35, 0) nut pocket; clear of the coupler |
| `z_lip` 43…46 | the lip (ID `lip_id` 46) bearing 1's outer race stops on | PETG lip ≥ 3 |
| `z_seat` 46…63 | ONE seat Ø `bearing_od` + `seat_add` (52.15) for both 6808s: bearing 1 at 46…53, `bearing_gap` 3 (the shaft's middle shoulder), bearing 2 at 56…63 | the shaft goes in from the wrist end with bearing 1 already on it — nothing passes over the ring |
| `z_face` 63 | the block's wrist face; the retainer 63…66 (`retainer_t`, ID 48, two ears at y ±31, the hard-stop post on +Y) | pull-out (+Z) goes shoulder 2 → bearing 2 → retainer |
| 66.8…76.2 | the shaft's integral **90T GT2 ring** (teeth 68…75, two Ø59.19 flanges — the 20T is flanged on its hub side only), `ring_teeth` / `ring_width` | `retainer_clear` 0.8 past the retainer; `z_ring_mid` 71.5 |
| 77…83 | the hard-stop pin boss (Ø6, radial +X, M3 pilot hole; the pin is EXTRAS) | 1 mm clear of the ring flange and the flange |
| `z_flange` 84…88 | the Ø60 flange (`flange_dia`), 4× M3 on Ø46 (`RollEndParams.bolt_circle_dia`), nuts captive from its elbow face | against the forearm wall's elbow face at host x −88 |
| 88…90 | the 2 mm spigot in the wall's Ø60.3 recess (`flange_recess_add`) | `RollEndParams.flange_recess_depth` |
| 43.5…90 | the Ø28 cable bore (`bore`) end to end | = the wall's `cable_bore` |
| `z_motor_face` 57.05 | the roll motor's mounting face (body toward the elbow, 17.55…57.05; the MKS board 3.45…17.55); the 3 mm pad plate ABOVE it (`pad_t`, radial tension slots ±2.5, the Ø22.3 pilot slot); the 20T's hub face `pulley_lift` 0.5 above the plate, its teeth level with the ring (`t20` = `pad_t` + `pulley_lift` + `t20_hub`) | `x_motor` = `motor_offset` 55.5: what a `roll_belt` 230-2GT sets (`lib/belts.py centre_distance`) |

Shaft: Ø40.3 journals (`bearing_bore` + `journal_add`, +0.3 in the inner races like the drive's hub), Ø44
shoulders (`shoulder_od` < the inner race's edge, `inner_race_od` [ESTIMATE]), the Ø44 core from bearing 2 to
the flange. Ratio 90 / 20 = 4.5 (`FOREARM_ROLL_RATIO`) → ≈ 1.1–1.8 N·m at the roll from the 40 mm kit motor,
against a worst-case static load of ≈ 0.9 N·m (forearm horizontal, wrist 90°, roll 90°, ≈ 0.5 kg wrist + gripper +
0.2 kg payload). The shoulder (`cycloidal_drive.md` §7) carries ≈ 0.35 kg more beyond the elbow.

## 3. Fits and printing
PETG, the drive's rules (`cycloidal_drive.md` §6): the seat is the bearing OD + `seat_add` 0.15 (a printed hole
comes out undersize — print a fit gauge first), the journals `journal_add` 0.3 interference. Tests
(`tests/forearm/test_roll_drive.py`) hold the journal press fit at 0.9–1.0 × 132 mm³ per bearing and every
other pair in the module below 1 mm³. Print the block elbow-face down (the disc flat on the bed, the tube
rising, the pad tower needs support), the shaft flange down (the ring's grooves print vertically), the
retainer flat.

## 4. Assembly sequence
1. Drop the 4 M4 nuts into the block's disc pockets; bolt the block to `j3_coupler#1` (the same bolts `j2_link`'s
   disc used).
2. Press bearing 1 onto the shaft's elbow-end journal up to the middle shoulder; drop the 4 M3 nuts into the
   flange's pockets.
3. Slide the shaft + bearing 1 into the block from the wrist end (the outer race to the lip); press bearing 2
   onto journal 2 and into the seat; bolt the retainer (2× M3 into the lugs).
4. Motor onto the pad (4× M3 through the tension slots), the 20T on its shaft with its teeth level with the
   ring, the 230-2GT belt; slide the motor out along the slots to tension.
5. Cables from the forearm through the bore, out of the block's top window behind the motor tower.
6. Forearm wall onto the flange (spigot in the recess), 4× M3 × 20 from the wall's wrist face into the flange's
   nuts. The hard-stop pin (M3) into the shaft's boss; it meets the retainer's post at ±`FOREARM_ROLL_LIMIT_DEG`.

## 5. Attachment to the arm (`robot/`)
`elbow_link` = `gt2_pulley_90t#1` + `j3_coupler#1` + `forearm_roll_drive#1:stator` (the elbow's driven side
carrying the block, both bearings, the retainer, the motor + board, the 20T); `forearm_link` =
`forearm_roll_drive#1:rotor` (the shaft) + `j2_link` + the caps + the wrist-pitch motor. `Joint("forearm_roll")`
has Z along the forearm and X = N (its child's long direction IS the axis). The forearm side of the interface is
`lib/forearm/params.py RollEndParams`: the wall at `wall_x` (−96…−88), its recess, bolt circle and cable bore —
`j2_link`'s DEFAULT build (`lib/forearm/link.py roll_wall`). In `arm.py` the module is kept whole under the
`elbow_link` group. The wrist-pitch motor moved to where a 264-2GT belt puts it (`J2_MOTOR_SLIDE_X`) so its plug
clears the wall (`plug_clearance`).

## 6. Not modelled / to confirm
`docs/open_issues.md`: the belts, the stop pin, the home sensor (on the retainer's post, magnet in the flange,
to the MKS board's limit input), a belt guard, the cable route; the 6808's mass and inner-race edge, `t20_hub`,
the belt lengths, the limit; a **4th CAN id** (`src/config.py` names three boards, the arm carries five).
