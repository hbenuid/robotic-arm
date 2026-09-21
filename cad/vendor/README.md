# vendor/ — purchased-part STEP files (current best model per part)

**Last updated:** 2026-09-21 — see the root `CHANGELOG.md` for dated changes.

**Purpose:** the geometry each `COTS = True` part in `parts/<group>/` imports. Committed via the
`!/cad/vendor/*.step` gitignore exception as Git LFS objects (not regenerable from Python). Unlike
`reference/solidworks/<name>.step` (or `reference/cycloidal/<name>.step` for the drive) — the
immutable re-export that fixes each purchased part's **frame and size** — a vendor file may be
**replaced** by a better catalog model; the manifest
(`reference/manifest.json`, `vendor` sub-entry) records which file is current and
`tests/test_parts_convention.py::test_cots_vendor_matches_reference_frame` checks it still
occupies the reference's bounding box (±1.5 mm) after the part's `VENDOR_TO_REF` re-orientation.

| File | Part module | Current model | step.parts status (2026-08-28) |
|---|---|---|---|
| `gt2_pulley_20t.step` | `parts/wrist/gt2_pulley_20t.py` | SolidWorks re-export of the downloaded `GT2_20T` model (148 faces, real teeth, Ø5 bore, 6 mm belt, Ø16 flanges, 2 set screws) | `gt2_pulley_20t_bore5_w6` **tried and rejected**: analytic simplified (10 faces, Ø18.2 flanges, 10 mm long — >1.5 mm off) |
| `gripper_rail_6mm.step` | `parts/gripper/gripper_rail_6mm.py` | SolidWorks export, Ø6 × 125 mm rail | no Ø6 round rod in the catalog (only 8/10/12 mm supported rails) |
| `mg996r_servo.step` | `parts/gripper/mg996r_servo.py` | SolidWorks re-export of a third-party MG996R model (4 bodies); original: `arm_assembly_organized/gripper/Servo Motor MG996R 3D Model.step` | `MG996R`/`towerpro` not in the catalog; only generic servo envelopes — keep |
| `mg996r_horn.step` | `parts/gripper/mg996r_horn.py` | idem; original `…/gripper/Servo MG996R Horn.step` | not a catalog part |
| `nema17_pancake.step` | `parts/wrist/nema17_pancake.py` | the 7-part pancake-motor sub-assembly of the SolidWorks arm, flattened into one 11-solid part by `tools/reference/extract_placements.py` | `stepper_motor_nema17_l0020_single_shaft` exists but is analytic simplified — keep |
| `bearing_625.step` | `parts/cycloidal/bearing_625.py` | step.parts `bearing_625_2rs_sealed_simple` (1 solid, 16 faces, Ø16 × 5, axis Z standing on z=0 — identity `VENDOR_TO_REF`) | adopted 2026-08-28; the reference is the drive repo's annulus |

Cycloidal-drive purchased parts **without** a vendor file (their `_envelope()` — the drive repo's
simplified model, also the reference STEP — is the geometry; `test_cots_vendor_matches_reference_frame`
skips them):

| Part module | Why no catalog model (2026-08-28) |
|---|---|
| `parts/cycloidal/bearing_6003.py` | `bearing_6003_2rs_sealed_simple` **tried and rejected**: the file is a Ø24 × 8 bearing (a 628 size), not 17 × 35 × 10 |
| `parts/cycloidal/bearing_6814.py` | no 6814 / 61814 entry in the catalog (search and direct ids 404) |
| `parts/cycloidal/nema17_48mm.py` | `stepper_motor_nema17_l0048_single_shaft` **tried and rejected**: 42.3² × 48 body but a 14.8 mm shaft — the drive needs the 22 mm D-shaft (13 mm engagement past the 9 mm plate) |
| `cycloidal_ring_pins`, `cycloidal_output_pins`, `cycloidal_shaft_support_pin`, `cycloidal_motor_bolts`, `cycloidal_housing_bolts`, `cycloidal_housing_nuts` | pattern parts (21 / 4 / 1 / 4 / 8 / 8 solids); the catalog has single fasteners only |

What to **order** for each purchased part is not here: it is `PURCHASE_SPEC` / `PURCHASE_QTY` /
`PURCHASE_NOTE` in the part module, printed as the buy list by `./cadtool python tools/bom.py` (which also
lists the purchased items that have no geometry at all).

## Swapping in a catalog model
```bash
./cadtool parts "GT2 20" --limit 20                         # search (ANDed tokens; facets --tag/--family/--standard)
./cadtool parts --id <id> --download --filename <name>.step --overwrite   # -> vendor/<name>.step (sha256 verified)
./cadtool inspect vendor/<name>.step --planes   # frame of the new model (leaf bbox + planar faces)
# set VENDOR_TO_REF in parts/<group>/<name>.py so the model lands in the reference frame (reference/<origin>/<name>.step)
./cadtool pytest -k <name>                                  # vendor-frame + envelope + convention tests
./cadtool gen parts/<group>/<name>.py                       # regenerate the STEP (git-ignored)
./cadtool python tools/reference/import_solidworks.py                 # refresh manifest.json (vendor sha/bbox)
```
If the catalog model is worse than the SolidWorks re-export, restore it:
`cp reference/solidworks/<name>.step vendor/<name>.step && ./cadtool python tools/reference/import_solidworks.py`.
For the cycloidal drive's parts the manifest is owned by `tools/cycloidal/import_cadquery.py`
(run that one for them), and a worse catalog model is simply deleted — the
envelope takes over and the manifest entry loses its `vendor` block.
