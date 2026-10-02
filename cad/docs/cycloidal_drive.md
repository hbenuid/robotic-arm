# Cycloidal drive — 21:1 shoulder-pitch actuator, its shell turning

**Purpose:** the specification of the drive (carried over from the `cycloidal_drive` repo, corrected
where the code disagreed with it), where it lives in `cad/`, what changed in the build123d port, the turning
shell it builds now (§5.5), and how it is attached to the arm.

## Provenance
- Designed in CadQuery in [`hbenuid/cycloidal_drive`](https://github.com/hbenuid/cycloidal_drive);
  imported at `2f1f67d` (117 commits) by a subtree merge into `cad/cycloidal_import/` (`6739509`),
  ported to build123d (`f670d20` printed parts, `7c4b9aa` purchased parts, `8b89e20` assembly,
  robot description in the following commit) and the import directory removed once the port was
  complete — `git log` keeps every original commit. The old repo is untouched.
- **Reference geometry** = the CadQuery builders' own STEP exports at that revision, produced by
  `tools/cycloidal/export_cadquery.py` (run inside the old repo's venv) and copied by
  `tools/cycloidal/import_cadquery.py` into `reference/cycloidal/<name>.step` (manifest kind `designed` for
  the 6 printed parts, `cots` for the 10 purchased ones, `origin: cycloidal_drive@2f1f67d`).
  `tests/cycloidal/test_port.py` proves the port reproduces them: identical face sets and
  tessellations, and the exact analytic volume for every part but the two spline discs (§11).

## Overview

| Parameter | Value |
|---|---|
| Application | The arm's `shoulder_pitch` joint (~340 mm reach to the wrist centre) — the `j1_coupler` fork holds its hub and motor, `j1_link` rises off its turning shell, printed with the shell's body (§12) |
| Type | Two-disc cycloidal drive, discs 180° apart; the carrier held, the housing shell the output (§5.5) |
| Gear ratio | 21:1 (21 ring pins, 20 lobes; the shell turns the same way as the motor) — `lib/params.py CYCLOIDAL_RATIO` (`DriveConfig.ratio`; the port's 20:1 held the ring) |
| Motor | NEMA 17, 48 mm body, 22 mm × Ø5 D-shaft (`parts/cycloidal/nema17_48mm.py`; the shown geometry is `vendor/nema17_48mm.step`, the real body from the user's kit export carrying this motor's own pilot + shaft, `lib/cycloidal/motor.py`) + its MKS SERVO42D board kit on the rear face (`parts/joints/mks_servo42d.py`, `z_mks_board` = −48) |
| Eccentricity | 1.5 mm |
| Housing OD | 129.2 mm at the pillars, 108 between them (`CYCLOIDAL_HOUSING_OD`; the port's 140 / 116, §5.4) |
| Print material | PETG, 100 % infill on the discs |

The drive's own dimensions live in **`lib/cycloidal/params.py`** (`DriveConfig`, ten frozen groups:
`gear, disc, shaft, bearings, housing, motor, output_hub, tolerances, profile, stack_up`, and `shell` - the turning
shell's `ShellParams`, `None` in the port's `LEGACY_CONFIG`); every number below names its field.

## 1. Drive geometry

### 1.1 Ring gear (`cfg.gear`)

| Parameter | Value | Notes |
|---|---|---|
| Ring pins | 21 (`num_ring_pins`) | N + 1, N = 20 lobes |
| Ring pin Ø | 4.00 (`ring_pin_dia`) | h6 ground steel dowel |
| Ring pin circle Ø | 100.00 (`ring_pin_circle_dia`) | centred in the housing bore; the port's 108 less 2 × `RING_INSET` (§5.4) |
| Ring pin length | 40 (`ring_pin_length`) | 5 in the shell ring's pin ring + 30 between the two plates + 5 in the body's pin ring (`ring_pin_engagement`; the port's 35: 3.5 in its motor plate + 28 + 3.5 in the bearing-zone wall) |
| Housing bore Ø | 108.00 (`housing.bore_dia`) | 2 behind the pins, 3 past the discs' swing; between the pillars it is the housing's outline (1.9 outside the pin holes) |

### 1.2 Cycloidal disc (`cfg.disc`, `parts/cycloidal/cycloidal_disc_1.py`, `_2.py`)

| Parameter | Value | Notes |
|---|---|---|
| Lobes | 20 (`gear.num_lobes`) | sets the ratio |
| Disc count | 2 | orbit centres 180° apart cancel vibration. Disc 2's epitrochoid is phase-rotated by `gear.disc2_phase_deg = -180°/N_lobes = -9°`; the output-pin holes are identical (disc-local 0/90/180/270 on the 60 mm circle). **The discs are distinct printed parts** — a 180° assembly rotation is a no-op on a 20-lobe disc. |
| OD | ~99 (98.0 / 99.0 bbox) | epitrochoid profile (§8): tips R − r + e = 49.5, valleys R − r − e = 46.5 |
| Centre bore Ø | 35.10 (`center_bore_dia`) | 35 mm 6003 OD + 0.10 clearance |
| Thickness | 10.00 (`thickness`) | = 6003 width |
| Lobe chamfer | 1.00 × 45° both faces (`lobe_chamfer`) | assembly lead-in, hides elephant foot; symmetric (no wrong side); costs ~20 % lobe contact length |
| Inter-disc spacer | 2.00 (`inter_disc_spacer`) | |
| Output pin holes | 4 × Ø7.4 on Ø60 | §1.3 |

### 1.3 Output stage (`cfg.disc`, `cfg.output_hub`)

| Parameter | Value | Notes |
|---|---|---|
| Output pins | 4 at 90° (`output_pin_count`) | 4 × 45 mm h6 dowels (`output_pin_dia`, `output_pin_length`) on Ø60 (`output_pin_circle_dia`) |
| Disc pin hole Ø | 7.40 (`output_pin_hole_dia`) | 4 + 2 × 1.5 ecc + 0.4 clearance |
| Hub pin holes | Ø4.20 × 18 blind (`ring_pin_hole_dia`, from the flange's inner face to `z_bearing_top` − `output_hub_pin_ceiling`; the port's 19) | greased sliding fit through the discs; captured between the closed hub ceiling and the motor plate - both held: the pins stop the discs turning, so the ring does (§5.5) |

Clearances: pin-hole inner edge 26.30 vs bore radius 17.55 → **8.75 mm wall**; pin-hole outer edge
33.70 vs lobe valley 46.5 (~45.5 past the lobe chamfer) → **~12 mm wall**.

**Backlash:** the ring-pin mesh is a zero-clearance theoretical epitrochoid, so the output-pin holes are
the dominant designed source: slack = hole_r − pin_r − e = 3.70 − 2.00 − 1.50 = **0.20 mm radial**, i.e.
**≈ ±0.38° at the 30 mm pin circle** (was ±0.95° at Ø8.0, ±0.57° at Ø7.6). **7.4 mm is the practical
floor:** below ~7.2 the four rigid pins over-constrain against printed-hole position error across two
prints and the disc binds after shrink. Going lower means attacking the secondary sources (4.20 hub
fit, 6003 radial clearance, the 35.10 bore) or bushings/rollers on the pins — a redesign. Guarded by
`test_output_hole_backlash_budget` (`tests/cycloidal/test_disc.py`).

### 1.4 Eccentric shaft (`cfg.shaft`, `parts/cycloidal/cycloidal_eccentric_shaft.py`)

| Parameter | Value | Notes |
|---|---|---|
| Bearing-seat OD | 17.10 (`bearing_seat_od`) | light press in the 6003 bore |
| Eccentricity | 1.50 | lobe 1 at (+e, 0), lobe 2 at (−e, 0) |
| Spine OD | 5.00 (`spine_od`) | |
| Input collar OD | 10.00 (`input_collar_od`) | wall around the D-bore |
| D-bore Ø | 5.13 (`d_bore_dia` + 2 × `tolerances.d_bore_clearance_add`) | receives the motor shaft directly, flat on +Y (`d_bore_flat` 4.5, cut like the shaft's: flat-to-round 4.75, `lib/cycloidal/motor.py`) |
| D-bore depth | 14.00 (`d_bore_depth`) | 13 mm engagement (22 shaft − 9 plate) + 1 mm clearance |
| Support-pin hole | Ø5.15 × 11 blind (`support_pin_dia` + 2 × `dowel_bore_clearance_add`, `support_pin_hole_depth`) | keeps a 1 mm wall to the D-bore |
| Support pin | 5 × 20 h6 dowel | 11 in the shaft + 4 gap (the hub's flange) + 5 in the 625 (the port's 2 gap + 5 + 2 proud) |
| Bridge / retention flange | Ø23.10 (`bridge_flange_od` = seat OD + `bridge_flange_add` 6) | ruled loft (+e → −e) across the spacer zone; keeps the 6003s apart |
| Material | PETG, 100 % infill, 0.16 mm layers | built at its stack position z 9..35 |

## 2. Bearings (`cfg.bearings`)

| Bearing | Qty | d × D × B | Part | Role |
|---|---|---|---|---|
| 6003-2RS | 2 | 17 × 35 × 10 | `parts/cycloidal/bearing_6003.py` | one per disc, on the shaft lobes (~5.4 kN dyn.) |
| 6814-2RS | 2 | 70 × 90 × 10 | `parts/cycloidal/bearing_6814.py` | the turning shell's bearings, 58 apart, one at each end: on the hub past its flange, in the body's seat (z 48), and on the motor plate's sleeve behind the plate, in the shell ring's seat (z −10); outer races turn (~7.0 kN dyn.) |
| 625-2RS | 1 | 5 × 16 × 5 | `parts/cycloidal/bearing_625.py` | in the hub's inner-face pocket, on the shaft support dowel (~1.0 kN dyn.) |

The port stacked both 6814s in the seat (z 37..57, centres 10 apart); the turning shell spreads them across the
discs (z 13..35), mirror images about their middle (z 24) - §5.5. The hub pin holes (r 27.9..32.1) sit well inside
the 70 mm inner-race bore.

## 3. Non-bearing purchased parts

| Part | Spec | Qty | Module |
|---|---|---|---|
| Motor | NEMA 17, 42.3² × 48 body, Ø5 shaft 22 long (4 round + 18 D-cut, `shaft_dcut_flat` 4.5 = flat-to-round 4.75, not the standard 4.5), Ø22 × 2 pilot, 4 × M3 on 31 mm square tapped 4.5, ~0.45 Nm | 1 | `nema17_48mm` |
| Ring pins | 4 × 40 h6 hardened dowel (buy 25) | 21 | `cycloidal_ring_pins` |
| Output pins | 4 × 45 h6 hardened dowel; light press in the hub's 4.20 blind holes (printed ~4.00–4.10), hub ceiling / motor plate as backup capture, free through the discs' 7.4 holes | 4 | `cycloidal_output_pins` |
| Shaft support pin | 5 × 20 h6 dowel | 1 | `cycloidal_shaft_support_pin` |
| Housing bolts | M4 × 65 SHCS (ISO 4762, `bolt_length`), Ø7 × 4 head; counterbored in the shell ring's end, through the shell end to end into nuts sunk in its body (the port's M4 × 55 ended in the ring gear body) | 6 | `cycloidal_housing_bolts` |
| Housing nuts | M4 hex, 7.0 AF × 3.2 (pocket 7.2 AF), in the body's pockets from its hub end (`lib/cycloidal/housing.py build_shell_body`) | 6 | `cycloidal_housing_nuts` |
| Motor bolts | M3 × 10 SHCS, Ø5.3 × 3 head (13 total): 6 through the plate + 4 engagement, head flush in the 3 mm inner-face pocket | 4 | `cycloidal_motor_bolts` |
| Hub bolts + nuts | 4 × M4 × 25 (`lib/yaw_coupler/params.py ForkParams.hub_screw_len`) from the `j1_coupler` fork's hub-side leg into captive M4 nuts in the hub's flange (its arm-mount pattern, `ShellParams.hub_nut_depth`: the screws' ends flush with the nuts) — install the nuts before pressing the hub through its 6814 | 4 + 4 | not modelled — on the buy list through `tools/bom.py EXTRAS` |

The purchased parts are modelled as the drive repo's simplified solids (annuli, cylinders, hex
prisms): each module's `_envelope()`, which is also its reference STEP. `vendor/bearing_625.step`
is a step.parts catalog model (`bearing_625_2rs_sealed_simple`); see `vendor/README.md` for why the
6003, 6814 and NEMA 17 catalog models were not adopted.

## 4. Axial stack-up (`cfg.stack_up`; z = 0 at the motor-plate outer face, +Z inward)

| Layer | Thickness | Running total |
|---|---|---|
| Motor plate outer wall (`motor_plate_wall`) | 5 | 5 |
| Motor plate inner wall (`motor_plate_inner_wall`) | 4 | 9 = `z_motor_plate_inner` |
| Input clearance / D-shaft collar (`input_clearance`) | 4 | 13 = `z_disc1` |
| Disc 1 + 6003 (`disc_thickness`) | 10 | 23 |
| Inter-disc spacer (`inter_disc_spacer`) | 2 | 25 = `z_disc2` |
| Disc 2 + 6003 | 10 | 35 |
| Clearance to the hub's flange + the flange (`output_clearance`: the input clearance + `hub_flange`, the motor plate's mirror; the port's 2) | 13 | 48 = `z_output_bearings` |
| The hub-end 6814 (`output_bearing_total`: one; the port's two, 20) | 10 | 58 = `z_bearing_top` |
| The lip (`output_wall` = `ShellParams.end_lip`, the port's 3) | 3 | 61 = `total_housing_depth` |

Behind z = 0, the mirror: the motor-end 6814 (−10..0) and the shell ring's lip (−13..−10) - the shell runs
**z −13..61** (`shell_ends`), symmetric about the middle of the discs (z 24); the two plates' inner faces bound the gear
at 9 and 39 (`arm_zone`, where `j1_link` rises off the shell). Derived: `disc_zone` 26, `bore_zone` 39,
`ring_gear_body_height` 52 (the body, 9..61).
**The ends:** the held hub is 23 tall (flange 9 + grip 10 + lip 3 + `proud_above_housing` 1 = `ShellParams.end_plate_gap`)
at z 39, its face at **z = 62** (`CYCLOIDAL_HUB_FACE_Z`) on the `j1_coupler` fork's hub-side leg; the motor plate's
sleeve ends at **z = −22** (`sleeve_end`, `CYCLOIDAL_SLEEVE_END_Z`) on the motor-side leg's outer face. **The housing
bolts run end to end:** M4 × 65 from the shell ring's 4.5 counterbores (heads' tops at −12.5) to their ends at 56.5,
flush with their nuts (stack `z_housing_nuts` 53.3, `test_full_nut_engagement`) in the body's pockets
(`test_bolt_ends_inside_the_body`).
**Total envelope with the motor:** the shell 74 (−13..61), the motor 48 behind the plate's face (the pilot recesses 2
mm into the plate), the MKS board kit behind the motor 14.1 (module z −62.1 … 62).

The module layout (`lib/cycloidal/layout.py stack_positions`, the drive repo's `assembly.py`
numbers for the gear stack) places: discs at (±1.5, 0, 13 / 25) with their 6003s, the 6814s at 48 (the hub's) and −10
(the motor plate's), hub and 625 at 39, ring pins at 4, output pins at 12, support pin at 24, motor bolts at −5,
housing bolts at −12.5, nuts at 53.3; shaft, motor, motor plate and shell ring at 0 (built in place).

## 5. Housing design notes (`cfg.housing`)

### 5.1 Envelope — OD 129.2, the shell z −13..61, bore Ø108, a 6814 seat Ø90.15 × 10 at each end, bolt circle Ø117 (`bolt_count` × M4).

### 5.2 Housing split — two printed parts, mirror images about the middle of the discs

1. **Shell ring** (`parts/cycloidal/cycloidal_shell_ring.py`, z −13..9): the shell's motor end - from its end in, the
   Ø86.15 lip (`end_lip`, `lip_radial` 2 over the 90 mm outer race), the motor-end 6814's Ø90.15 seat, and a pin ring
   (`ring_bore_dia`) round the held motor plate with the 21 ring pins' blind holes Ø4.20 from its face on the body; the
   M4 holes with Ø7.4 × 4.5 counterbores in the end face (z −13). The motor plate itself
   (`parts/cycloidal/cycloidal_motor_plate.py`) is held: §5.5.
2. **The body** (`lib/cycloidal/housing.py build_shell_body`, z 9..61), **printed as one with `j1_link`** (the upper
   arm rises off it, §12): the Ø108 bore round the discs (9..39), a pin ring round the hub's flange (39..48) with the
   ring pins' blind holes from its face on the bore, the hub-end 6814's seat (48..58), the lip (58..61); the M4
   through-holes, each backed by a full-height bolt pillar, and the housing nuts' **captive hex pockets** (7.2 AF)
   from the hub end down to the nuts' seats, turned `bolt_nut_turn_deg` (a flat outward, §5.4).

The port's two parts were its **motor plate** (z 0..9, the housing half) and its **ring gear body**
(`parts/cycloidal/cycloidal_ring_gear_body.py`, local z 0..51 at stack 9..60: 21 blind ring-pin holes Ø4.20 × 31.5
with 1 mm entry funnels, `ring_pin_entry_chamfer_*`, the stepped bore Ø116 / the Ø90.15 two-bearing seat / the lip,
the nut pockets in its output face); both are `LEGACY_CONFIG` builds now (`REFERENCE_BUILD`, `tests/cycloidal/test_port.py`),
the ring gear body modelled but not placed (`UNPLACED`).

**Shared outer profile** — both parts carry the same pillar / window silhouette around the bolt
circle (pillars `pillar_inner_w` 18 at the bore, `pillar_outer_w` 10 at the OD, one per bolt -
`lib/cycloidal/layout.py pillar_corners`) from end to end; the shell ring seats on the body's pillar faces, no
continuous rim. One shared cutter (`lib/cycloidal/housing.py reveal_window_cutter`) is subtracted from every housing
base solid. The turning shell's body keeps `ShellParams.arm_windows` (2) of its windows solid - the two either side of
its first pillar, where the upper arm rises off it (`arm_root`, §12); the other four, and all six of the shell ring's,
open on the discs and the ring pins.

**Bolt count** (`bolt_count`) — a departure from the port (the others: the gear size and the pillar tips, §5.4): `DEFAULT_CONFIG` builds 6 bolts where the port
(`LEGACY_CONFIG`) had 8, at 60°, the first at `bolt_start_deg` = `ARM_DEG` (47.584167°, the port's 0°): on the
centreline of the upper arm, which rises off the turning shell between the two windows either side of it (§12;
`tests/upper_arm/test_j1_link.py` checks the angle against `j1_link`'s frame). The bolt circle sits 8.5 mm outside the
ring pins', so any start clears them. The port's two housing parts and the housing bolts / nuts declare
`REFERENCE_BUILD` (their `LEGACY_CONFIG` build), which `tests/cycloidal/test_port.py` (the parts) and
`test_cots_envelope_tracks_reference_bbox` (the bolts / nuts) compare with the CadQuery exports.

**Outer-edge chamfer** (`edge_chamfer` 1.0; the port's 1.5, `chamfer_outer_silhouette`): the pillars' outer vertical
corners always, plus the *entire* outer-wire perimeter of each part's external end face (the shell's two ends: the
shell ring's z = −13, the body's z = 61); the faces where they meet (z = 9) stay sharp so the stack beds flush. Internal holes are never beveled. `edge_chamfer = 0` disables; keep it
under `LUG_WALL` - 1 to leave the external faces 1 mm or more past the nut pockets (§5.4).

### 5.3 Output hub (`parts/cycloidal/cycloidal_output_hub.py`, `cfg.output_hub`)

PETG (aluminium viable): Ø70.3 (`od` = 70 mm 6814 bore + 0.3 interference grip), 23 tall (the port's 28): a Ø90 × 9
flange (`hub_flange`, the motor plate's mirror) in the body's pin ring, its face on the hub-end 6814 cut back over
the outer race (`plate_relief_*`) - the flange stops the inner race -, then the grip, on past the lip to the
`j1_coupler` fork's hub-side leg; 4 blind Ø4.20 × 18 pin holes on Ø60 closed by a 1 mm ceiling; Ø6 shaft-clearance
bore through the flange; Ø16.2 × 5 625 pocket on the inner face; the **arm-mount pattern** 4 × Ø4.4 through-holes on
Ø50 at 45° from the pins, each into a captive M4 nut pocket (7.2 AF × 4.7, `hub_nut_depth`) in the flange's inner
face - the port bolted `j1_link` to it, the turning shell bolts the hub to the fork's leg (M4 × 25 from its
counterbores, their ends flush with the nuts); a Ø36 lightening recess in the face (`arm_mount_pocket_dia`, 0 =
sealed). The grip passes the lip bore with ~7.9 mm radial clearance. It is held: the carrier, with the output pins
(§5.5). Print output-face-down.

### 5.4 Gear size and pillar tips (`RING_INSET`, `LUG_WALL`)

`DEFAULT_CONFIG` moves the ring-pin circle and everything outside it — the bore, the bolt circle, the od — `RING_INSET`
in from the port's (`lib/cycloidal/params.py`), so the drive is 2 × `RING_INSET` smaller across and every wall outside
the pins keeps the port's thickness: between the pillars the housing's outline is its bore, 1.9 outside the pin holes,
so the bore cannot come in further than the pins (a Ø106 bore left 0.9 there, and the 1.5 edge chamfer broke into the
holes). The discs follow the pins (their epitrochoid is the pin circle's, §8: K1 = e·N / R goes 0.58 → 0.63, no
undercut); the 6814s, the hub, the shaft, the output pins, the stack-up and every fastener do not change. **The limit**
is the wall between the pins' far ends and the 6814 seat: the pins sit `ring_pin_engagement` into the wall round the
seat, 2.8 thick there (6.8 in the port; `test_pin_holes_dont_breach_the_seats` holds ≥ 2.5) — test-print the seat
(§6). The discs, the ring pins and the port's housing parts declare `REFERENCE_BUILD` for it (§5.2, §11).

**Pillar tips** (`LUG_WALL`): each housing nut and its pocket are turned a flat outward (`bolt_nut_turn_deg` 30), and
`LUG_WALL` of plastic stands past the pocket, so the od is the bolt circle + the pocket's AF + 2 × `LUG_WALL` (the port
kept 3.3 past a corner); the edge chamfer is 1.0, so the external faces keep 1.5 past the nut pockets and 1.4 past
the counterbores. The pillars keep the port's widths (`pillar_inner_w` / `pillar_outer_w`: they bridge the windows); beside the turned
nut 3.1 of wall is left. The bolt circle stays at the pins' distance.

### 5.5 The turning shell (`cfg.shell`, `ShellParams`)

`DEFAULT_CONFIG` turns the drive inside out: the **carrier** - the output hub and its pins, which stop the discs
turning - and the **motor** are held, the `j1_coupler` fork gripping the hub at one end and the motor plate's sleeve
at the other (`lib/yaw_coupler/params.py ForkParams`, §12); the **housing shell** - the shell ring, the body round the
discs and the ring pins - is the output, so the ratio is the ring pins' 21 (`DriveConfig.ratio`), the shell turning
the same way as the motor. **Why:** the port carried the whole arm on two 6814s stacked in one seat, centres 10 apart,
so their play and the PETG's give tipped the arm; the turning shell runs on one 6814 at each end of the gear stack,
centres **58** apart (`test_the_6814s_are_58_apart`). **The upper arm rises off the shell's middle** (PAROL6-like):
`j1_link` is printed as one with the shell's body, its arm leaving the shell over the gear (`arm_zone`, §12) - nothing
bolts the arm to the shell's end.

- **The two ends mirror each other** about the middle of the discs (`tests/cycloidal/test_shell_body.py`): from the
  outside in, an `end_lip`, the 6814 (its outer race in the shell's seat), a `plate_dia` plate inside the shell's
  `ring_bore_dia` pin ring - the motor plate at the motor end, the hub's flange at the other, each the port's motor
  plate thick, its face cut back `plate_relief_depth` from `plate_relief_dia` over the turning outer race - and the
  stack's 4 mm clearance to its disc. The ring pins stand in both pin rings (`ring_pin_engagement` 5 into each).
- **The motor plate is the carrier's motor end** (`parts/cycloidal/cycloidal_motor_plate.py`, its `LEGACY_CONFIG`
  build the port's plate): the plate - the motor's pilot, M3s and shaft bore as before - and a sleeve back over the
  motor: `sleeve_bore_dia` round the motor's and the MKS board's corners (61 across them), `sleeve_od`, and the hub's
  70.3 grip over the last `out_width`, where the motor-end 6814 presses on after sliding over the rest; it runs
  through the fork's motor-side leg to its outer face (`sleeve_end`).
- **The shell ring** (`parts/cycloidal/cycloidal_shell_ring.py`) and **the body** (`build_shell_body`, j1_link's): §5.2.
- **Held both ways along the axis:** at each end the shell's lip stops that 6814's outer race, the held plate's face
  its inner race (`test_the_shell_is_held_both_ways_along_the_axis`).
- **The fork's clearances:** the shell's pillars sweep r 64.6, `RING_DROP` above the yoke's lowered ring (2.68); the
  shoulder's whole range keeps the upper arm - `j1_link`, the elbow motor, the shell - its 1 mm running gap off the
  fork's two legs (`tests/test_sweeps.py`).
- **Assembly, from the motor end:** the hub's nuts into its flange; the hub-end 6814 into the body's seat from inside
  (`j1_link` face down), the hub pressed into it grip first to its flange; the gear stack (output pins in the hub,
  discs, shaft, the ring pins into the body's pin ring) as before; the motor plate with its 6814 on its seat, the
  shell ring over the sleeve and onto the ring pins' ends, bolted to the body with the M4 × 65s into its nuts; the
  motor into the sleeve; the whole lowered into the fork, the hub bolted to its leg, the cap bolted on.

## 6. PETG print tolerances (`cfg.tolerances`)

PETG holes print 0.10–0.20 undersized (shrink, elephant foot, arc overshoot); the positive offsets
are print compensation, so **everything ends up a press fit on a real print** — the offset sets how
tight. Never design a hole smaller than its steel part.

| Fit | Adjustment | Field | Application |
|---|---|---|---|
| Bearing outer race → housing | +0.15 / +0.20 on the bore | `output_bearing_seat_dia` 90.15, `bearing_seat_bore_add` 0.2 | 6814 seats - the body's, the shell ring's - (firm press), 625 pocket 16.2 (slip-to-light press) |
| 6814 inner race → hub / sleeve | +0.3 interference | `output_hub.od` 70.3; `ShellParams.sleeve_od` 70 | the hub's grip and the sleeve's last `out_width` (press); the sleeve behind it slides |
| Ring & output pin holes | +0.20 → Ø4.20 | `ring_pin_press_sub` −0.20 | slip-press as printed, through (plate) and blind (body, hub) |
| Disc centre bore | +0.10 → 35.10 | `center_bore_dia` | 6003 OD |
| Disc output-pin holes | nominal 7.4, no offset | `output_pin_hole_dia` | shrink *reduces* backlash (§1.3) |
| Motor-shaft D-bore | +0.065 on the radius → Ø5.13 | `d_bore_clearance_add` | slip-to-light press |
| Support-pin bore | +0.075 on the radius → Ø5.15 | `dowel_bore_clearance_add` | firm press |
| Mating surfaces | +0.15 | `mating_surface_add` | pilot recess 22.30 |
| Bolt / head clearance | +0.4 on the Ø | `bolt_clearance_add` | M3 3.4 / 5.7, M4 4.4 |

Print the discs flat at 100 % infill; ≤ 0.16 mm layers for bearing seats; test-print a bearing fit
gauge first; PETG shrinks 0.3–0.5 % over 129 mm (the housing may need ~129.5 — not compensated).

## 7. Performance estimates

Motor 0.45 Nm × 21 = 9.45 Nm theoretical; 55–65 % efficiency (printed, no pin bearings) →
**5.2–6.1 Nm practical**, ~1.3–1.5 kg at 400 mm including the arm; 200–500 rpm in → 9.5–24 rpm out, the shell
turning with the motor; not backdrivable; backlash ≈ ±0.38° (§1.3). The Ø100 ring-pin circle (§5.4) puts ~8 % more force on each ring pin
than the port's Ø108 for the same torque (54 / 50). **Torque budget warning:** short of this arm even at
its ~340 mm reach to the wrist centre (the arm stretched out horizontally weighs more on the shoulder than this) —
a lightweight demonstrator; a balance spring, NEMA 23 or a higher ratio for heavier payloads.

## 8. Disc profile (`lib/cycloidal/profiles.py`)

```
x(θ) =  R·cos θ − r·cos(θ + ψ) − e·cos(N·θ)
y(θ) = −R·sin θ + r·sin(θ + ψ) + e·sin(N·θ)
ψ    = atan2(sin((1 − N)·θ), R/(e·N) − cos((1 − N)·θ))
R = 54 (ring-pin circle radius), r = 2 (pin radius), N = 21 (pins), e = 1.5;  lobes = N − 1 = 20
```
2000 points per revolution (`profile.num_points`, `endpoint=False`) through a **periodic
interpolating** B-spline (`Edge.make_spline(..., periodic=True)` — never an approximating spline).

## 9. Shopping list

Generated, never retyped: `./cadtool python tools/bom.py --module cycloidal_drive` prints what to order
(each purchased module's `PURCHASE_SPEC` × pieces, from `assemblies/cycloidal_drive.py OCCURRENCES`) and,
under "not modelled", the items that have no geometry (`tools/bom.py EXTRAS`: the hub's bolts and nuts,
grease). The shell's body is on the print list as `j1_link` (it is printed with the arm). `cycloidal_drive.step` shows the same split: purchased parts grey, printed parts the drive's `TINT`.

Budget (2f1f67d estimate): NEMA 17 48 mm $10–15 (as the MKS SERVO42D closed-loop kit with the board: ~$30–40) · 6003-2RS ×2 $4–8 · 6814-2RS ×2 $16–40 · 625-2RS $1–2 ·
ring-pin dowels (pack of 25) $8–12 · output-pin dowels $2–4 · motor bolts $1–2 · housing bolts $2–4 · M4 nuts
(housing + hub) $2 · support dowel $0.5–1 · hub bolts $1–2 — **~$47–87**.

## 10. Where things live in `cad/`

| What | Where |
|---|---|
| Dimensions | `lib/cycloidal/params.py` (`DriveConfig`, `DEFAULT_CONFIG`); interface values re-exported as `lib/params.py CYCLOIDAL_*` (+ `STEEL_DENSITY`, bearing / motor / fastener masses) with locks in `tests/test_params_invariants.py` |
| Derived numbers | `lib/cycloidal/layout.py` — hole patterns, `hex_circumdiameter`, `ring_pin_engagement`, `motor_bolt_counterbore_depth`, `hub_flange`, `hub_height`, `shell_ends`, `arm_zone`, `sleeve_end`, `stack_positions` |
| Profile maths | `lib/cycloidal/profiles.py` (numpy) |
| Shared builders | `lib/cycloidal/housing.py` (reveal-window cutter, outer-silhouette chamfer, nut pockets, the turning shell's body `build_shell_body` - `lib/upper_arm/link.py` prints it with `j1_link`), `lib/cycloidal/disc.py` (`build_disc`); the arm-wide `lib/geom.py` (cylinders with `NUDGE` overshoot, `single_solid`, hex prisms) |
| Printed parts (designed, `CONVERTED = True`) | `parts/cycloidal/`: `cycloidal_disc_1.py`, `cycloidal_disc_2.py`, `cycloidal_eccentric_shaft.py`, `cycloidal_motor_plate.py`, `cycloidal_output_hub.py` — each exposes `build(cfg)` for tests and its `@step` model; the port's `cycloidal_ring_gear_body.py` (`UNPLACED`, for `test_port`); and the turning shell's `cycloidal_shell_ring.py`, designed here with no reference (`lib/reference.py NO_REFERENCE`) |
| Purchased parts (COTS) | `parts/cycloidal/`: `bearing_6003.py`, `bearing_6814.py`, `bearing_625.py`, `nema17_48mm.py` (+ the board kit `parts/joints/mks_servo42d.py`, shared with the belt joints' motors), `cycloidal_ring_pins.py` (21), `cycloidal_output_pins.py` (4), `cycloidal_shaft_support_pin.py`, `cycloidal_motor_bolts.py` (4), `cycloidal_housing_bolts.py` (`bolt_count`), `cycloidal_housing_nuts.py` (`bolt_count`) — shared body `lib/cots.py` (every COTS part's); the multi-body ones are registered in `MULTI_BODY`; each says what to order (`PURCHASE_SPEC` / `PURCHASE_QTY`, built from `DEFAULT_CONFIG`) |
| Assembly | `assemblies/cycloidal_drive.py` — one row `(part, role, position)` per piece from `stack_positions` (the MKS board at `z_mks_board`); `EXPECTED` locks the leaves / solids / volume, whole and per body (`EXPECTED["bodies"]`); `./cadtool python -c "from assemblies.cycloidal_drive import totals; print(totals())"` |
| Printed vs. bought | `./cadtool python tools/bom.py --module cycloidal_drive` (print list, buy list, the purchased items not modelled — `EXTRAS`); in `cycloidal_drive.step` (and in the arm) purchased parts are `_occurrences.BOUGHT_TINT` grey, printed parts `cycloidal_drive.TINT`; `./cadtool python tools/export_printables.py` → `print/<name>.stl` |
| References | `reference/cycloidal/<name>.step` (CadQuery exports, Git LFS), `reference/manifest.json` entries (`file` field); `tools/cycloidal/export_cadquery.py` + `tools/cycloidal/import_cadquery.py` |
| Tests | `tests/cycloidal/test_{disc,eccentric_shaft,motor_plate,shell_body,output_hub,shell_ring,housing,purchased,fitment,assembly,port}.py` + `tests/cycloidal/helpers.py` (the drive's config; the geometry helpers are `tests/helpers.py`; one module per part, geometry marked `slow`) |
| Viewer / export | `./cadtool gen assemblies/cycloidal_drive.py`, `./cadtool viewer` (`?file=assemblies/cycloidal_drive.step`), `./cadtool export parts/cycloidal/<name>.step stl` |

## Viewing the drive

```bash
cd cad
./cadtool gen assemblies/cycloidal_drive.py          # assemblies/cycloidal_drive.step (git-ignored) if missing
./cadtool viewer                                     # then open the printed URL with ?file=assemblies/cycloidal_drive.step
#   also ?file=assemblies/arm.step (the drive in the arm), ?file=parts/cycloidal/cycloidal_shell_ring.step (any part),
#   ?file=robot/arm.urdf (the robot with joint sliders - shoulder_pitch turns the drive's rotor with j1_link)
./cadtool snapshot assemblies/cycloidal_drive.step snapshots/cycloidal_drive.png --size-profile assembly --view-labels
./cadtool snapshot assemblies/cycloidal_drive.step snapshots/cycloidal_drive_x.png --display xray --camera "30:20"
#   (no turntable GIF: cadgen's `snapshot --video` renders a model's `@step(animation=…)` clip, and the
#    drive declares none - motion review is the CAD Viewer)
```

## 11. Port notes (CadQuery → build123d)

| CadQuery | build123d (here) |
|---|---|
| `cq.Workplane(...).circle().extrude()`, `.cut()`, `.union()` | algebra mode: `lib.geom.cylinder/through`, `-`, `+` |
| `.faces(">Z").edges().chamfer(d)` | `solid.chamfer(d, None, face.edges())` on the planar face picked by centre Z |
| `cq.Edge.makeSpline(pts, periodic=True)` → face → `extrudeLinear` | `Edge.make_spline(pts, periodic=True)` → `Face(Wire([edge]))` → `Solid.extrude` (same `GeomAPI_Interpolate`) |
| `.workplane().center(x, y)` (cumulative!) | explicit `Pos(x, y, z) *` |
| `.polygon(6, d).transformed(rotate=...)` | `RegularPolygon(r, 6, rotation=deg)` (vertex 0 on +X, like CadQuery) |
| 21 per-hole ruled lofts (funnels) | `Cone(r_entry, r_hole, depth)` |
| D-bore: cut a round bore, union a 0.25 mm key sliver back | round bore minus a half-space beyond the flat (identical geometry, no sliver) |
| `chamfer_outer_silhouette(result, cfg, external_face="<Z")` | `chamfer_outer_silhouette(solid, cfg, external_z=0.0)` — end face by height, edges by `GeomType.LINE` + radius |
| `.val().Volume()`, `.val().isInside()`, `.section(height=z).Area()`, `intersect().Volume()` | `lib.reference.solid_volume`, `Solid.is_inside`, thin-slab `section_area`, `interference` (`tests/helpers.py`) |
| `copy.deepcopy` + `object.__setattr__` on a frozen dataclass | `dataclasses.replace` |

- **Magic numbers promoted to tagged fields:** `housing.motor_plate_shaft_bore` 15, `lip_radial` 2,
  `ring_pin_entry_chamfer_depth/_add` 1/1, `pillar_inner_w/outer_w` 18/10, `bolt_nut_af` 7;
  `shaft.bridge_flange_add` 6; `motor.pilot_height` 2, `motor_bolt_thread_margin` 0.5,
  `motor_bolt_recess` 1; `tolerances.bolt_clearance_add` 0.4. Module constants: `PILLAR_OVERSHOOT`
  (`lib/cycloidal/layout.py`, with the pillar's outline `pillar_corners` / `pillar_half_width` the yoke's sockets
  share), `CUTTER_OVERSHOOT`, `BARREL_EDGE_MARGIN` (`lib/cycloidal/housing.py`).
- **Dropped (unused) fields:** `ProfileParams.spline_tolerance`, `PETGTolerances.bearing_inner_shaft_sub`
  / `sliding_clearance_add`, `HousingParams.wall_thickness` / `motor_plate_wall`,
  `BearingParams.ecc_qty` / `inp_qty`. `DriveConfig` itself is frozen now.
- **Geometry is identical** to the CadQuery builders in `LEGACY_CONFIG` (`DEFAULT_CONFIG` departs in the housing's
  bolt count, §5.2, the gear size and the pillar tips, §5.4; `tests/cycloidal/test_port.py` builds a part's `REFERENCE_BUILD` if it has one: same face sets, same
  tessellations to 1e-11, same analytic volume to 1e-13 for the analytic parts). Order matters on the
  disc: the lobe chamfer is applied while the end faces carry only the spline edge, *before* the holes.
- **OCCT volume caveat:** `BRepGProp` volume integration is ~0.3 % off on the 2000-knot spline face of
  the discs (differently on the CadQuery-exported and the native solids, both wrong; the adaptive
  integrator is worse). Face fingerprints and tessellations are the identity check; the discs' masses
  in the URDF therefore carry ~0.3 % error. `REF_VOL_TOL` stays at the 0.5 % default for the discs and
  is 1e-4 for the other designed parts.
- **Layout source:** the drive repo's `export.py` assembly had the ring pins at z 4.5 and the output
  pins at z 13; its `assembly.py` (and every test) uses 5.5 and 11. The port uses `stack_positions`
  (= `assembly.py`). The SolidWorks sub-assembly was imported from `export.py`'s STEP, so its pins are
  1–2 mm off ours (cosmetic, inside their holes); it is kept in `placements.json` as a cross-check.
- **Designed interference budget** (`test_module_interference_budget`, mm³): the 6814 inner-race press fits on
  the hub and on the motor plate's sleeve, 330.6 each; housing bolts through the solid nuts 241.3; motor-bolt heads
  in the plate 51.9 and shanks in the vendor motor's tapped holes (modelled at the M3 minor diameter) 46.4; 6003 /
  lobe press fits 26.8 × 2; the MKS kit's four M3x30 in the same holes from the rear 159.4. Everything else
  is < 1 mm³; the whole module vs the arm - the base, `j1_link` (with the shell's body), the `j1_coupler` fork and its
  cap - contact at most (≤ 1, `TestPoseInTheArm`).
- **Corrections to the drive repo's spec:** its §10 said "both discs are identical — the 180° offset is
  applied in the assembly": wrong (disc 2 carries the −9° phase, §1.2); §3.3's "7.6 mm disc holes"
  → 7.4; stale 67 / 134 / 120 mm comments (the port's OD is 140) and other rotted numbers in comments.
- **Two venvs:** `tools/cycloidal/export_cadquery.py` runs in the old repo's CadQuery venv
  (`cd ../cycloidal_drive && uv run python ../robotic-arm/cad/tools/cycloidal/export_cadquery.py`);
  everything else via `./cadtool`. The name maps of the exporter and `lib/reference.py` are kept equal
  by `test_exporter_name_map_matches_registry`.

## 12. Attachment to the arm

- `placements.json` record **`cycloidal_drive#1`** (`kind: module, designed: true`) = the SolidWorks
  node `New cyloidal assembly` (sic) at path 1.3: position (1.844381, 85.010435, 31.446506) mm,
  rotation XYZ (−180, −3.694455, 180)°, parent = the arm root. Its `solidworks` block keeps the node's
  leaves / solids / volume / world bbox as a cross-check; `assemblies/arm.py` locates
  `assemblies/cycloidal_drive.py` there (row after `j1_coupler#1`).
- **Frame:** module Z = motor axis, z = 0 the motor-plate outer face, motor body in −Z, the shell z −13..61, the
  hub's face at 62, the sleeve's end at −22. In the arm the axis is horizontal (module +Z → world −N, N = the J2/J3
  pitch direction; module +Y → world up). The `j1_coupler` **fork** (`lib/yaw_coupler/params.py ForkParams`, DEFAULT)
  holds the drive on its own axis - the SolidWorks pose, 0.21 off the old cradle's - with two alike thin legs
  (`ShellParams.yoke_leg`) 1 mm past the shell's ends, mirror images about the middle of the discs, which sits on the
  base_yaw axis (`ForkParams.face_x`; the capture had it 7.5 off, toward the motor - the drive and all it carries moved
  along N, `lib/placements.py SHIFTS`), each leg straddling a flat of the yoke's disc: the hub-side leg takes the hub's face and its 4 bolts, the motor-side one - its
  lower half and the cap `j1_coupler_cap` - grips the motor plate's sleeve; the shell turns between them, over the
  yoke's lowered ring. The SolidWorks yoke cradled the port's housing on its pillars at 225° / 270° / 315°, the
  6-pillar housing's yoke at 240° / 300° in sockets - `LEGACY` / git history. **The upper arm rises off the shell:**
  `j1_link` (parametric, `lib/upper_arm/`, `ArmParams`) is printed with the shell's body, its arm - a plain bar, the
  plate's width, 30 thick over the gear (the drive's z 9..39) - rising out of the body's two solid windows, the four
  others open on the discs; its elbow end slid along N onto the arm's outer face (`arm_slide`), and everything past
  the elbow with it (`lib/placements.py SHIFTS`): the end effector is no longer over the base_yaw axis. The elbow
  motor stands on the arm's motor side (+N, with the forearm), down a hole through the bar onto a plate under its
  outer face, its shaft back through the plate to the elbow belt. The drive's frame in `j1_link`'s
  is the capture's (`tests/upper_arm/`), the shell's body and the hub on the legs `TestPoseInTheArm`.
- **Kinematics:** the drive **is the `shoulder_pitch` joint** of `robot/frames.py` (axis `N` = the
  drive's −Z; origin `SHOULDER_ORIGIN` = `j1_link#1`'s origin, on the drive axis; limits
  `SHOULDER_PITCH_LIMITS_DEG`). `assemblies/cycloidal_drive.py BODIES` splits the rows into two rigid
  bodies and `LINKS` places them with a `:<body>` key suffix (`_occurrences.split_key` / `world_rows`):
  the **stator** (`cycloidal_drive#1:stator` — every row but the rotor's: the motor plate, the output hub, its
  output pins and the 625, the NEMA 17 + bolts + its MKS board, and the gear train: eccentric shaft, support pin,
  discs, 6003s) rides in `shoulder_link` with the yawing `j1_coupler`; the **rotor** (`cycloidal_drive#1:rotor` —
  what turns with the shell: the shell ring, ring pins, housing bolts + nuts, both 6814s) rides in `upper_arm_link`
  with `j1_link`, which holds the shell's body.
  `EXPECTED["bodies"]` locks the per-body totals and `TestPoseInTheArm` checks the joint origin sits on the drive
  axis. The arm STEP's viewer tree keeps
  the module whole under `shoulder_link` (one linked child); the per-link meshes split it. Which MKS
  motor (`software/control/src/config.py` J1..J3) drives which joint is unconfirmed; `config.py` still carries
  `gear_ratio` 1.0 where this joint needs `1 / CYCLOIDAL_RATIO`, the output turning with the motor (the direction:
  `robot/AGENTS.md`).

## 13. Change policy (carried over)

Every change to the drive must update (1) the tests — one `tests/cycloidal/test_<part>.py` per part
plus `tests/cycloidal/test_assembly.py` for the stack-up, (2) this document, and (3) run `./cadtool pytest`
green before it is done. Geometry changes also regenerate the STEPs (`./cadtool gen`), the
module totals lock (`totals()` / `totals(body)`), `robot/meshes/shoulder_link.stl` + `upper_arm_link.stl` and the URDF/SDF inertials
(`tools/robot/derive.py --urdf-draft` / `--check`).
