# vendor/ — purchased-part STEP files (current best model per part)

**Purpose:** the geometry each `COTS = True` part in `parts/<group>/` imports. Committed via the
`!/cad/vendor/*.step` gitignore exception as Git LFS objects (not regenerable from Python). Unlike
`reference/solidworks/<name>.step` (or `reference/cycloidal/<name>.step` for the drive) — the
immutable re-export that fixes each purchased part's **frame and size** — a vendor file may be
**replaced** by a better catalog model; the manifest
(`reference/manifest.json`, `vendor` sub-entry) records which file is current and
`tests/test_parts_convention.py::test_cots_vendor_matches_reference_frame` checks it still
occupies the reference's bounding box (±1.5 mm) after the part's `VENDOR_TO_REF` re-orientation. How to produce or
swap one (Recipe D) and the rules for these files: [`CLAUDE.md`](CLAUDE.md); this file is the record of what is here.

| File | Part module | Current model | step.parts status (2026-08-28) |
|---|---|---|---|
| `gt2_pulley_20t.step` | `parts/wrist/gt2_pulley_20t.py` | SolidWorks re-export of the downloaded `GT2_20T` model (148 faces, real teeth, Ø5 bore, 6 mm belt, Ø16 flanges, 2 set screws) | `gt2_pulley_20t_bore5_w6` **tried and rejected**: analytic simplified (10 faces, Ø18.2 flanges, 10 mm long — >1.5 mm off) |
| `gripper_rail_6mm.step` | `parts/gripper/gripper_rail_6mm.py` | SolidWorks export, Ø6 × 125 mm rail | no Ø6 round rod in the catalog (only 8/10/12 mm supported rails) |
| `mg996r_servo.step` | `parts/gripper/mg996r_servo.py` | SolidWorks re-export of a third-party MG996R model (4 bodies); original: `arm_assembly_organized/gripper/Servo Motor MG996R 3D Model.step` | `MG996R`/`towerpro` not in the catalog; only generic servo envelopes — keep |
| `mg996r_horn.step` | `parts/gripper/mg996r_horn.py` | idem; original `…/gripper/Servo MG996R Horn.step` | not a catalog part |
| `nema17_pancake.step` | `parts/wrist/nema17_pancake.py` | the 7-part pancake-motor sub-assembly of the SolidWorks arm, flattened into one 11-solid part by `tools/reference/extract_placements.py` | `stepper_motor_nema17_l0020_single_shaft` exists but is analytic simplified — keep |
| `bearing_625.step` | `parts/cycloidal/bearing_625.py` | step.parts `bearing_625_2rs_sealed_simple` (1 solid, 16 faces, Ø16 × 5, axis Z standing on z=0 — identity `VENDOR_TO_REF`) | adopted 2026-08-28; the reference is the drive repo's annulus |
| `nema17_40mm.step` | `parts/joints/nema17_40mm.py` | the motor body of the SolidWorks "nema17x40_with_mks" kit export, split off by `tools/reference/split_mks_motor.py` (2 solids; re-framed like the drive motor — face z=0, body −Z, shaft +Z, D-flat +Y — its own boss + 23 mm shaft cut off and the drive's `pilot()` + `shaft()` fused on; identity `VENDOR_TO_REF`) | added 2026-09-21; the split IS the reference (`same_as_reference`); the user's own export, not a catalog part |
| `nema17_48mm.step` | `parts/cycloidal/nema17_48mm.py` | composed by `tools/reference/split_mks_motor.py --write drive`: the x48 kit export's real 48 mm body (front plate, housing, back plate, its two Ø22 bearings, connector, rotor - 7 solids; tie rods left out, the kit's M3x30 replace them) with its own boss + shaft cut off and the drive's `lib/cycloidal/motor.py pilot()` + `shaft()` fused on (the envelope's exact interface); identity `VENDOR_TO_REF` | added 2026-09-21; the reference stays the drive repo's envelope (the vendor's bbox is within 1.5 mm: connector +0.85 on +Y). The datasheet 17HS19-2004S1 ships a 24 mm / 15 mm-D-cut shaft (the x48 export's own) - the user's motor is the 22 mm one |
| `mks_servo42d.step` | `parts/joints/mks_servo42d.py` | the Servo42D_Assem (PCB 4 solids + cover) + 4 standoffs + 4 M3x30 of the same export (13 solids; z=0 at the motor's rear face, stack −Z, screws to z +19.6) | added 2026-09-21; one board kit per MKS motor — every `nema17_40mm` and `nema17_48mm` (`tools/bom.py` counts them) |

Purchased parts **without** a vendor file (their `_envelope()` — the drive repo's simplified model or, for a
native COTS part, the envelope accepted by `tools/reference/import_native.py` - also the reference STEP — is the
geometry; `test_cots_vendor_matches_reference_frame` leaves them out):

| Part module | Why no catalog model (2026-08-28) |
|---|---|
| `parts/cycloidal/bearing_6003.py` | `bearing_6003_2rs_sealed_simple` **tried and rejected**: the file is a Ø24 × 8 bearing (a 628 size), not 17 × 35 × 10 |
| `parts/cycloidal/bearing_6814.py` | no 6814 / 61814 entry in the catalog (search and direct ids 404) |
| `parts/joints/bearing_6808.py` | no 6808 / 61808 / 6908 entry (2026-09-22: nothing above a 17 mm bore in the catalog) - a native COTS part (`lib/reference.py NATIVE_COTS`, reference `reference/native/`) |
| `parts/joints/bearing_6806.py` | no 6806 / 61806 / 30 x 42 x 7 entry (2026-09-25: the catalog's 132 bearings stop at a 20 mm bore) - a native COTS part like the 6808 |
| `parts/joints/gt2_idler_20t.py` | `gt2_smooth_idler_bore5_w6` / `gt2_flanged_smooth_idler_bore5_w6` **tried and rejected** (2026-09-29): one generic model under both ids - Ø18.2 flanges on a Ø15.2 seat, 10 mm long, three loose solids with no bore - where the seller's drawing is a Ø12.1 seat, 9 mm long; a native COTS part built from that drawing (`lib/belts.py GT2_IDLER_*`) |
| `cycloidal_ring_pins`, `cycloidal_output_pins`, `cycloidal_shaft_support_pin`, `cycloidal_motor_bolts`, `cycloidal_housing_bolts`, `cycloidal_housing_nuts` | pattern parts (21 / 4 / 1 / 4 / 8 / 8 solids); the catalog has single fasteners only |
| `parts/joints/elbow_pulley_screws.py`, `elbow_pulley_nuts.py`, `wrist_pulley_screws.py`, `wrist_pulley_nuts.py` | the 90T pulley bolts: pattern parts (4 solids each), native COTS (`lib/reference.py NATIVE_COTS`) - the catalog has single fasteners only |
| `parts/joints/ky003_hall_sensor.py` | no KY-003 module and no A3144 (2026-09-29: "KY-003", "KY003", "A3144", "hall sensor", "hall effect" find nothing; "sensor module" only other boards; the catalog's `to_92s` is a bare package, not the module) - a native COTS part, its dimensions `lib/sensors.py` |

What to **order** for each purchased part is not here: it is `PURCHASE_SPEC` / `PURCHASE_QTY` /
`PURCHASE_NOTE` in the part module, printed as the buy list by `./cadtool python tools/bom.py` (which also
lists the purchased items that have no geometry at all).

## Where the vendor files come from
Besides step.parts downloads (Recipe D, `CLAUDE.md`) and `tools/reference/extract_placements.py` (the flattened
`nema17_pancake.step`), the third producer of vendor files is `tools/reference/split_mks_motor.py`: it splits the
"NEMA 17 x 40 + MKS SERVO42D" kit export (`lib/reference.py MKS_EXPORT_NAME`, outside the repo next to the monolith)
by GEOMETRY into `vendor/nema17_40mm.step` and `vendor/mks_servo42d.step` (what each holds and its frame: the table
above); `import_solidworks.py` then mirrors both into `reference/solidworks/` (`rel=None`, the `nema17_pancake`
pattern). `--write drive` composes `vendor/nema17_48mm.step` for the drive motor from the x48 export's body
(`MKS48_EXPORT_NAME`) the same way - every motor carries the drive motor's interface (`CLAUDE.md`); the drive keeps
its parametric envelope as the reference (`tools/cycloidal/import_cadquery.py --only nema17_48mm` writes its `vendor`
block).
