# vendor/ — purchased-part STEP files (current best model per part)

**Purpose:** the geometry each `COTS = True` part in `parts/` imports. Committed via the
`!/cad/vendor/*.step` gitignore exception (not regenerable from Python). Unlike
`reference/<name>.step` — the immutable SolidWorks re-export that fixes each purchased part's
**frame and size** — a vendor file may be **replaced** by a better catalog model; the manifest
(`reference/manifest.json`, `vendor` sub-entry) records which file is current and
`tests/test_parts_convention.py::test_cots_vendor_matches_reference_frame` checks it still
occupies the reference's bounding box (±1.5 mm) after the part's `VENDOR_TO_REF` re-orientation.

| File | Part module | Current model | step.parts status (2026-08-28) |
|---|---|---|---|
| `gt2_pulley_20t.step` | `parts/gt2_pulley_20t.py` | SolidWorks re-export of the downloaded `GT2_20T` model (148 faces, real teeth, Ø5 bore, 6 mm belt, Ø16 flanges, 2 set screws) | `gt2_pulley_20t_bore5_w6` **tried and rejected**: analytic simplified (10 faces, Ø18.2 flanges, 10 mm long — >1.5 mm off) |
| `gripper_rail_6mm.step` | `parts/gripper_rail_6mm.py` | SolidWorks export, Ø6 × 125 mm rail | no Ø6 round rod in the catalog (only 8/10/12 mm supported rails) |
| `mg996r_servo.step` | `parts/mg996r_servo.py` | SolidWorks re-export of a third-party MG996R model (4 bodies); original: `arm_assembly_organized/gripper/Servo Motor MG996R 3D Model.step` | `MG996R`/`towerpro` not in the catalog; only generic servo envelopes — keep |
| `mg996r_horn.step` | `parts/mg996r_horn.py` | idem; original `…/gripper/Servo MG996R Horn.step` | not a catalog part |
| `nema17_pancake.step` | `parts/nema17_pancake.py` | the 7-part pancake-motor sub-assembly of the SolidWorks arm, flattened into one 11-solid part by `tools/extract_placements.py` | `stepper_motor_nema17_l0020_single_shaft` exists but is analytic simplified — keep |

## Swapping in a catalog model
```bash
./cadtool parts "GT2 20" --limit 20                         # search (ANDed tokens; facets --tag/--family/--standard)
./cadtool parts --id <id> --download --filename <name>.step --overwrite   # -> vendor/<name>.step (sha256 verified)
./cadtool inspect refs vendor/<name>.step --facts --planes --positioning   # frame of the new model
# set VENDOR_TO_REF in parts/<name>.py so the model lands in the SolidWorks frame (reference/<name>.step)
./cadtool pytest -k <name>                                  # vendor-frame + envelope + convention tests
./cadtool gen parts/<name>.py                               # regenerate the committed STEP
./cadtool python tools/import_reference.py                  # refresh manifest.json (vendor sha/bbox)
```
If the catalog model is worse than the SolidWorks re-export, restore it:
`cp reference/<name>.step vendor/<name>.step && ./cadtool python tools/import_reference.py`.

The bearings, NEMA 17 motor and pins of the cycloidal drive are *not* here: that
sub-assembly lives in the `cycloidal_drive` repo (`bearing_625_zz_shielded_simple`, 6003/6814
bearings and NEMA17 bodies are available on step.parts when it is re-attached).
