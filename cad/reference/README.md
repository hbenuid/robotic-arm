# reference/ — SolidWorks reference geometry

**Last updated:** 2026-08-28 — see the root `CHANGELOG.md` for dated changes.

**Purpose:** the original design, as exported from SolidWorks, renamed to the clean part
names used everywhere in `cad/`. These files are **immutable inputs**: each custom part's
wrapper returns them until it is converted, and `tests/test_reference_match.py` compares every
converted part against them (checksums locked in `manifest.json`). Regenerate — never edit.

## Provenance
- Source tree: `/home/hben09/Documents/arm_assembly_organized` (SolidWorks 2026, STEP AP214 exports of 2026-08-27; not in git).
- Full assembly: `final Arm Assembly Fully Movable.STEP` (13.5 MB, inch units, sha256 `67c39d5dc9ff1d7b…`) —
  not committed; `placements.json` captures its structure.
- Regenerate: `./cadtool python tools/import_reference.py` (copies + `manifest.json`), then
  `./cadtool python tools/extract_placements.py` (`placements.json` + `vendor/nema17_pancake.step`),
  then `import_reference.py` once more so the manifest describes the extracted pancake.

## Naming map
Clean name ← SolidWorks product (source file under the source tree). Sizes are the bounding
box in mm after OCCT's import (inch-unit files are converted automatically).

| Part | Kind | SolidWorks product | Source export | Units | Solids | Bbox size (mm) | In arm |
|---|---|---|---|---|---|---|---|
| `base` | custom | `base of robot arm 62126` | `step/base of robot arm 62126.STEP` | mm | 1 | 168.178 × 95.807 × 106.679 | ×1 |
| `j1_coupler` | custom | `Base couple updated 62126 _J1 coupler` | `step/Base couple updated 62126 _J1 coupler.STEP` | mm | 1 | 96 × 63.976 × 106.264 | ×1 |
| `j1_link` | custom | `first joint edit 62126` | `step/first joint edit 62126.STEP` | mm | 1 | 300 × 34 × 90 | ×1 |
| `j1_cap` | custom | `first joint cap 8726` | `step/first joint cap 8726.STEP` | inch | 1 | 300 × 90 × 27.228 | ×1 |
| `j2_link` | custom | `Joint 2 change 8126` | `step/Joint 2 change 8126.STEP` | mm | 1 | 300 × 90 × 33.5 | ×1 |
| `j2_cap_1` | custom | `cap 1 joint 2 8726` | `step/cap 1 joint 2 8726.STEP` | mm | 1 | 245.461 × 90 × 14.5 | ×1 |
| `j2_cap_2` | custom | `cap of joint 2 piece 2 8526` | `step/cap of joint 2 piece 2 8526.STEP` | mm | 1 | 223.377 × 90 × 13.5 | ×1 |
| `j3_coupler` | custom | `Joint 2 coupler 62226_J3 Coupler` | `step/Joint 2 coupler 62226_J3 Coupler.STEP` | mm | 1 | 78 × 22 × 78 | ×2 |
| `gt2_pulley_90t` | custom | `GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric` | `step/GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric.STEP` | mm | 1 | 59.188 × 21.4 × 59.188 | ×2 |
| `gripper_clamp_bracket` | custom | `brack for hand cmap` | `step/brack for hand cmap.STEP` | mm | 1 | 26.2 × 64 × 43.2 | ×1 |
| `wrist_link` | custom | `final component arm qwrist movement` | `step/final component arm qwrist movement.STEP` | mm | 1 | 124.446 × 78 × 44 | ×1 |
| `gripper_cover` | custom | `Gripper Cover_Gripper Cover` | `step/Gripper Cover_Gripper Cover.STEP` | mm | 1 | 44 × 10.5 × 70 | ×1 |
| `gripper_end` | custom | `Gripper End_Gripper End` | `step/Gripper End_Gripper End.STEP` | mm | 1 | 27.123 × 30 × 25.123 | ×2 |
| `gripper_finger_left` | custom | `Gripper Hand Left_Gripper Hand Left` | `step/Gripper Hand Left_Gripper Hand Left.STEP` | mm | 1 | 26 × 4 × 105 | ×2 |
| `gripper_finger_right` | custom | `Gripper Hand Right_Gripper Hand Left` | `step/Gripper Hand Right_Gripper Hand Left.STEP` | mm | 1 | 26 × 4 × 105 | ×2 |
| `gripper_link_1` | custom | `Gripper link 1_Gripper link 1` | `step/Gripper link 1_Gripper link 1.STEP` | mm | 1 | 34.2 × 3.5 × 7.2 | ×2 |
| `gripper_link_2` | custom | `Gripper link 2_Gripper link 2` | `step/Gripper link 2_Gripper link 2.STEP` | mm | 1 | 34.2 × 5 × 7.2 | ×2 |
| `gripper_slider` | custom | `Gripper Mechanism Slider_Gripper Mechanism Slider` | `step/Gripper Mechanism Slider_Gripper Mechanism Slider.STEP` | mm | 1 | 18 × 26 × 60 | ×2 |
| `gripper_j3_connector` | custom | `Gripper to J3 connector 7726_Gripper to J3 connector` | `step/Gripper to J3 connector 7726_Gripper to J3 connector.STEP` | mm | 2 | 23 × 12.5 × 46 | ×1 |
| `servo_holder` | custom | `Servo Holder_Servo Holder` | `step/Servo Holder_Servo Holder.STEP` | mm | 1 | 44 × 15 × 72.5 | ×1 |
| `gt2_pulley_20t` | COTS | `GT2_20T_Конфигурация1` | `step/GT2_20T_Конфигурация1.STEP` | mm | 3 | 14.45 × 16 × 16 | ×1 |
| `gripper_rail_6mm` | COTS | `Gripper rail 6mm_Gripper rail 6mm` | `step/Gripper rail 6mm_Gripper rail 6mm.STEP` | mm | 1 | 6 × 125 × 6 | ×2 |
| `mg996r_servo` | COTS | `Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model` | `step/Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model.STEP` | mm | 4 | 55.8 × 45.2 × 20.5 | ×1 |
| `mg996r_horn` | COTS | `Servo MG996R Horn_Servo MG996R Horn` | `step/Servo MG996R Horn_Servo MG996R Horn.STEP` | mm | 1 | 32 × 2.5 × 12 | ×1 |
| `nema17_pancake` | COTS | `nema17_pancake` | `(extracted from the full assembly)` | mm | 11 | 41.5 × 47 × 43 | ×1 |

Notes:
- `gripper_finger_right` is the mirror configuration of `gripper_finger_left`; its SolidWorks
  export is still named `…_Gripper Hand Left` (stale configuration name, geometry is correct).
- `j1_cap`, `j2_cap_1`, `j2_cap_2` were modelled in assembly context: their geometry sits far
  from the part origin (up to ~1 m). `placements.json` compensates; choose a sane origin with
  `LOCAL_FROM_REF` when converting them.
- `gt2_pulley_20t`'s product name carries the Cyrillic configuration name `Конфигурация1`
  ("Configuration1"); it never enters our labels.
- `gt2_pulley_90t` is a printed parametric pulley (842 faces, ~1 s import); `gripper_j3_connector`
  has two bodies.

## Skipped from the SolidWorks assembly
- `New_cyloidal_assembly` (path 1.3, 15 leaves / 38 solids): the cycloidal drive — its
  source lives in the `cycloidal_drive` repo. World pose recorded in `placements.json`
  (`skipped[0]`): position (1.84, 85.01, 31.45) mm, rotation XYZ (-180.00, -3.69, 180.00)°.
- The 6 zero-geometry assembly-skeleton STEPs, the `base/step/` re-exports and the pancake
  motor's internal parts (flattened into `vendor/nema17_pancake.step`).

## placements.json
`occurrences[]`: one record per placed part (`kind: part`) or sub-assembly (`kind: module`)
with `key` (`<part>#<n>`), `path` (SolidWorks tree path), `parent` (module key or null),
`rel` / `world` placements as `{position, rotation_xyz_deg, matrix_3x4}`
(`Location(position, rotation_xyz_deg)` reproduces them; intrinsic XYZ Euler, degrees),
world bounding boxes, solid counts and volumes; `expected` totals for the assembly test;
`skipped[]` as above.
