# vendor/ — purchased-part STEP files (raw geometry)

**Purpose:** the exact CAD of the off-the-shelf parts the arm uses; source of truth for the
`COTS = True` parts in `parts/`. Committed via the `!/cad/vendor/*.step` gitignore exception
because they cannot be regenerated from Python. Checksums are locked in
`reference/manifest.json` (`tests/test_reference_match.py`).

| File | Part module | Origin |
|---|---|---|
| `gt2_pulley_20t.step` | `parts/gt2_pulley_20t.py` | SolidWorks re-export of the downloaded `GT2_20T` model (3 bodies; Russian-locale config name) |
| `gripper_rail_6mm.step` | `parts/gripper_rail_6mm.py` | SolidWorks export, Ø6 × 125 mm rail |
| `mg996r_servo.step` | `parts/mg996r_servo.py` | SolidWorks re-export of a third-party MG996R model (4 bodies); original: `arm_assembly_organized/gripper/Servo Motor MG996R 3D Model.step` |
| `mg996r_horn.step` | `parts/mg996r_horn.py` | idem; original `…/gripper/Servo MG996R Horn.step` |
| `nema17_pancake.step` | `parts/nema17_pancake.py` | the 7-part pancake-motor sub-assembly of the SolidWorks arm, flattened into one 11-solid part by `tools/extract_placements.py` (in the sub-assembly's own frame) |

## How it's wired
Each COTS part's `gen_step()` returns `vendor/<name>.step` if it exists, else a parametric
`_envelope()` from `lib/params.py` — both in the SolidWorks frame that
`reference/placements.json` assumes. Replacing a file with a better model (e.g. from
`/cad:step-parts`) upgrades the part with no code change — but re-orient it inside
`gen_step()` if its frame differs, update `reference/manifest.json`
(`./cadtool python tools/import_reference.py`) and re-run the tests.

The bearings, NEMA 17 motor and pins of the cycloidal drive are *not* here: that
sub-assembly lives in the `cycloidal_drive` repo.
