# reference/ — SolidWorks reference geometry

**Purpose:** the original design, as exported from SolidWorks, renamed to the clean part
names used everywhere in `cad/` (`solidworks/`) — plus, for the cycloidal drive, the CadQuery
exports of the `cycloidal_drive` repo the build123d parts were ported from (`cycloidal/`) — and, for the parts
designed here with no external origin (`lib/reference.py NATIVE` / `NATIVE_COTS`: the forearm roll drive's block,
shaft, retainer and its 6808 bearing envelope), their **accepted builds** (`native/`, written once by
`tools/reference/import_native.py`; `--force` accepts a changed design). These
files are **immutable inputs** (committed as Git LFS objects): each custom part's wrapper returns
them until it is converted, and `tests/test_reference_match.py` compares every converted part
against them. The rules for them — never edit, the regeneration order, the manifest, Recipe E for a new export —
are [`AGENTS.md`](AGENTS.md); this file is the record of what is here.

## Provenance
- Source tree: the SolidWorks 2026 STEP AP214 exports of 2026-08-27, **not in git, not needed on a machine and with
  no fixed place** — the committed copies here are the inputs, the raw tree only for re-running the derivation tools,
  which take its path (`--src` / `--src48` / `--monolith`).
- Full assembly: `final Arm Assembly Fully Movable.STEP` (13.5 MB, inch units, sha256 `67c39d5dc9ff1d7b…`) —
  not committed; `placements.json` captures its structure.
- NEMA 17 x 40 + MKS SERVO42D kit: `mks/nema17x40_with_mks.step` under the source tree (SolidWorks 2026 export of
  2026-09-21, mm, 15 solids, 521 236 bytes, sha256 `4e51a1591030…`; `lib.reference.MKS_EXPORT_NAME`) — not committed
  and no longer kept anywhere (re-export from SolidWorks if the split must be redone; the sha256 identifies it);
  `tools/reference/split_mks_motor.py` splits it into `vendor/nema17_40mm.step` (the body, with the drive's pilot + shaft
  from `lib/cycloidal/motor.py` in place of the export's) and `vendor/mks_servo42d.step` (board kit), which are committed
  and mirrored into `solidworks/`. The sibling
  `mks/nema17x48_with_mks.step` (cm units, 24 solids, 3 345 264 bytes, sha256 `c1958e60a14f…`; `MKS48_EXPORT_NAME`) is
  the same kit with the 48 mm motor - a 17HS19-2004S1 with the datasheet's 24 mm / 15 mm-D-cut shaft, not the drive's
  22 mm one - so `split_mks_motor.py --write drive` composes `vendor/nema17_48mm.step` from its body and the drive's
  own pilot + shaft; the drive motor's reference stays `cycloidal/nema17_48mm.step` (`import_cadquery.py --only nema17_48mm`
  records the vendor block).
- Measured, not committed: `GT2 Pulley - 20 - 60 teeth.STEP` (SolidWorks 2026 export of 2026-09-26,
  mm, 1 solid, 1 974 574 bytes, sha256 `55733855bdc5…`; `lib/reference.py MEASURED`) — the 20-60T compound pulley,
  converted by measuring it (`parts/AGENTS.md` Part states → *measured*): nothing of it is in this folder, its numbers
  are in `lib/pulley/params.py` and `tests/pulley/test_gt2_pulley_20_60t.py`.
- Regenerating these files, in order: [`AGENTS.md`](AGENTS.md).

## Naming map
Clean name ← SolidWorks product (source file under the source tree); every row lives in
`solidworks/<name>.step`. Sizes are the bounding box in mm after OCCT's import (inch-unit files
are converted automatically). How many of each the arm uses is not recorded here: `./cadtool python tools/bom.py`
counts them from the assembly tables.

| Part | Kind | SolidWorks product | Source export | Units | Solids | Bbox size (mm) |
|---|---|---|---|---|---|---|
| `base` | custom | `base of robot arm 62126` | `step/base of robot arm 62126.STEP` | mm | 1 | 168.178 × 95.807 × 106.679 |
| `j1_coupler` | custom | `Base couple updated 62126 _J1 coupler` | `step/Base couple updated 62126 _J1 coupler.STEP` | mm | 1 | 96 × 63.976 × 106.264 |
| `j1_link` | custom | `first joint edit 62126` | `step/first joint edit 62126.STEP` | mm | 1 | 300 × 34 × 90 |
| `j2_link` | custom | `Joint 2 change 8126` | `step/Joint 2 change 8126.STEP` | mm | 1 | 300 × 90 × 33.5 |
| `j3_coupler` | custom | `Joint 2 coupler 62226_J3 Coupler` | `step/Joint 2 coupler 62226_J3 Coupler.STEP` | mm | 1 | 78 × 22 × 78 |
| `gt2_pulley_90t` | custom | `GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric` | `step/GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric.STEP` | mm | 1 | 59.188 × 21.4 × 59.188 |
| `gripper_clamp_bracket` | custom | `brack for hand cmap` | `step/brack for hand cmap.STEP` | mm | 1 | 26.2 × 64 × 43.2 |
| `wrist_link` | custom | `final component arm qwrist movement` | `step/final component arm qwrist movement.STEP` | mm | 1 | 124.446 × 78 × 44 |
| `gripper_cover` | custom | `Gripper Cover_Gripper Cover` | `step/Gripper Cover_Gripper Cover.STEP` | mm | 1 | 44 × 10.5 × 70 |
| `gripper_end` | custom | `Gripper End_Gripper End` | `step/Gripper End_Gripper End.STEP` | mm | 1 | 27.123 × 30 × 25.123 |
| `gripper_finger_left` | custom | `Gripper Hand Left_Gripper Hand Left` | `step/Gripper Hand Left_Gripper Hand Left.STEP` | mm | 1 | 26 × 4 × 105 |
| `gripper_finger_right` | custom | `Gripper Hand Right_Gripper Hand Left` | `step/Gripper Hand Right_Gripper Hand Left.STEP` | mm | 1 | 26 × 4 × 105 |
| `gripper_link_1` | custom | `Gripper link 1_Gripper link 1` | `step/Gripper link 1_Gripper link 1.STEP` | mm | 1 | 34.2 × 3.5 × 7.2 |
| `gripper_link_2` | custom | `Gripper link 2_Gripper link 2` | `step/Gripper link 2_Gripper link 2.STEP` | mm | 1 | 34.2 × 5 × 7.2 |
| `gripper_slider` | custom | `Gripper Mechanism Slider_Gripper Mechanism Slider` | `step/Gripper Mechanism Slider_Gripper Mechanism Slider.STEP` | mm | 1 | 18 × 26 × 60 |
| `gripper_j3_connector` | custom | `Gripper to J3 connector 7726_Gripper to J3 connector` | `step/Gripper to J3 connector 7726_Gripper to J3 connector.STEP` | mm | 2 | 23 × 12.5 × 46 |
| `servo_holder` | custom | `Servo Holder_Servo Holder` | `step/Servo Holder_Servo Holder.STEP` | mm | 1 | 44 × 15 × 72.5 |
| `gt2_pulley_20t` | COTS | `GT2_20T_Конфигурация1` | `step/GT2_20T_Конфигурация1.STEP` | mm | 3 | 14.45 × 16 × 16 |
| `gripper_rail_6mm` | COTS | `Gripper rail 6mm_Gripper rail 6mm` | `step/Gripper rail 6mm_Gripper rail 6mm.STEP` | mm | 1 | 6 × 125 × 6 |
| `mg996r_servo` | COTS | `Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model` | `step/Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model.STEP` | mm | 4 | 55.8 × 45.2 × 20.5 |
| `mg996r_horn` | COTS | `Servo MG996R Horn_Servo MG996R Horn` | `step/Servo MG996R Horn_Servo MG996R Horn.STEP` | mm | 1 | 32 × 2.5 × 12 |
| `nema17_pancake` | COTS | `nema17_pancake` | `(extracted from the full assembly)` | mm | 11 | 41.5 × 47 × 43 |
| `nema17_40mm` | COTS | `nema17x40_with_mks` (body + shaft) | `(split from mks/nema17x40_with_mks.step)` | mm | 2 | 42 × 49 × 62.4 |
| `mks_servo42d` | COTS | `nema17x40_with_mks` (Servo42D_Assem + standoffs + M3x30) | `(split from mks/nema17x40_with_mks.step)` | mm | 13 | 43 × 43 × 33.7 |

Cycloidal drive (`cycloidal/<name>.step`: the named CadQuery builder's export at
`cycloidal_drive@2f1f67d`; `tools/bom.py --module cycloidal_drive` counts them):

| Part | Kind | CadQuery builder | Units | Solids | Bbox size (mm) |
|---|---|---|---|---|---|
| `cycloidal_disc_1` | designed | `src/cycloidal_disc.py:build_cycloidal_disc()` | mm | 1 | 105.885 × 105.885 × 10 |
| `cycloidal_disc_2` | designed | `src/cycloidal_disc.py:build_cycloidal_disc(phase_offset_deg=disc2_phase)` | mm | 1 | 107 × 107 × 10 |
| `cycloidal_eccentric_shaft` | designed | `src/eccentric_shaft.py:build_eccentric_shaft()` | mm | 1 | 26.1 × 23.1 × 26 |
| `cycloidal_motor_plate` | designed | `src/motor_plate.py:build_motor_plate()` | mm | 1 | 140 × 140 × 9 |
| `cycloidal_ring_gear_body` | designed | `src/ring_gear_body.py:build_ring_gear_body()` | mm | 1 | 140 × 140 × 51 |
| `cycloidal_output_hub` | designed | `src/output_hub.py:build_output_hub()` | mm | 1 | 70.3 × 70.3 × 28 |
| `bearing_6003` | COTS | `src/purchased_parts.py:build_bearing_6003()` | mm | 1 | 35 × 35 × 10 |
| `bearing_6814` | COTS | `src/purchased_parts.py:build_bearing_6814()` | mm | 1 | 90 × 90 × 10 |
| `bearing_625` | COTS | `src/purchased_parts.py:build_bearing_625()` | mm | 1 | 16 × 16 × 5 |
| `nema17_48mm` | COTS | `src/purchased_parts.py:build_nema17_motor()` | mm | 1 | 42.3 × 42.3 × 70 |
| `cycloidal_ring_pins` | COTS | `src/purchased_parts.py:build_ring_pins()` | mm | 21 | 111.397 × 111.698 × 35 |
| `cycloidal_output_pins` | COTS | `src/purchased_parts.py:build_output_pins()` | mm | 4 | 64 × 64 × 45 |
| `cycloidal_shaft_support_pin` | COTS | `src/purchased_parts.py:build_shaft_support_pin()` | mm | 1 | 5 × 5 × 20 |
| `cycloidal_motor_bolts` | COTS | `src/purchased_parts.py:build_motor_bolts()` | mm | 4 | 36.3 × 36.3 × 13 |
| `cycloidal_housing_bolts` | COTS | `src/purchased_parts.py:build_housing_bolts()` | mm | 8 | 132 × 132 × 59 |
| `cycloidal_housing_nuts` | COTS | `src/purchased_parts.py:build_housing_nuts()` | mm | 8 | 133.083 × 133.083 × 3.2 |

Notes:
- `gripper_finger_right` is the mirror configuration of `gripper_finger_left`; its SolidWorks
  export is still named `…_Gripper Hand Left` (stale configuration name, geometry is correct).
- The link caps `j1_cap` (`first joint cap 8726`, inch), `j2_cap_1` (`cap 1 joint 2 8726`) and `j2_cap_2`
  (`cap of joint 2 piece 2 8526`) were removed from the design 2026-09-25: their reference STEPs and manifest entries
  are gone, their products are `lib/reference.py SKIPPED_PRODUCTS` and `placements.json` keeps their poses under
  `skipped`. (They were modelled in assembly context, their geometry up to ~1 m from the part origin.)
- `gt2_pulley_20t`'s product name carries the Cyrillic configuration name `Конфигурация1`
  ("Configuration1"); it never enters our labels.
- `gt2_pulley_90t` is a printed parametric pulley (842 faces, ~1 s import); `gripper_j3_connector`
  has two bodies.

## Designed module: the cycloidal drive
- `New_cyloidal_assembly` (sic; path 1.3, 15 leaves / 38 solids in SolidWorks) is recorded in
  `placements.json` as the **designed module** `cycloidal_drive#1`: its pose — position
  (1.84, 85.01, 31.45) mm, rotation XYZ (-180.00, -3.69, 180.00)° — places
  `assemblies/cycloidal_drive.py`, whose contents come from code (`lib/cycloidal`), not from the
  SolidWorks node. The node's own totals / bbox stay in the record's `solidworks` block as a
  cross-check (`tests/cycloidal/test_assembly.py`); the walker does not descend into it.

## Skipped from the SolidWorks assembly
- The 6 zero-geometry assembly-skeleton STEPs, the `base/step/` re-exports and the pancake
  motor's internal parts (flattened into `vendor/nema17_pancake.step`).

## placements.json
`occurrences[]`: one record per placed part (`kind: part`) or sub-assembly (`kind: module`)
with `key` (`<part>#<n>`), `path` (SolidWorks tree path), `parent` (module key or null),
`rel` / `world` placements as `{position, rotation_xyz_deg, matrix_3x4}`
(`Location(position, rotation_xyz_deg)` reproduces them; intrinsic XYZ Euler, degrees),
world bounding boxes, solid counts and volumes; `expected` totals (every `kind: part` record, the mounted ones
included) for the assembly test; `designed_modules[]` (the keys of `kind: module, designed: true` records, which carry
`rel`/`world`, a `solidworks` cross-check block and `source` instead of solids/volume); `mounted[]` (the keys of the
part records with a `mount` block — the belt joints' motors and boards declared in `lib/mounts.py`, written by
`tools/reference/mount_placements.py`: `path` / `parent` / `label_in_monolith` null, `rel == world = host world *
mount.frame_in_host`, `mount.host` a SolidWorks key or the motor key for a board); `skipped[]` (one record per SolidWorks
node whose product the design dropped, `lib/reference.py SKIPPED_PRODUCTS` - the link caps: `path`, `label`, `reason`,
leaves / solids / volume and the `rel` / `world` pose, moved there by `tools/reference/mount_placements.py`).
The capture records stay as extracted when the design changes a link's length: `lib/placements.py SHIFTS` moves
the records beyond it when they are read (`assemblies/AGENTS.md` "Shifted occurrences").
