# vendor/ — purchased-part STEP files (current best model per part)

**Purpose:** the geometry a `COTS = True` part in `parts/<group>/` imports when it has a vendor file (the others build
their `_envelope()`: below). Committed via the `!/cad/vendor/*.step` gitignore exception (not regenerable from
Python). Unlike the part's reference (`reference/<origin>/<name>.step`, the immutable re-export that fixes its
**frame and size**), a vendor file may be **replaced** by a better catalog model; the manifest
(`reference/manifest.json`, `vendor` sub-entry) records which file is current. How to produce or swap one (Recipe D),
the frame check and the rules for these files: [`AGENTS.md`](AGENTS.md); this file is the record of what is here.

| File | Part module | Current model (its solid count: `tests/test_parts_convention.py MULTI_BODY`, else one) | step.parts status (searched 2026-08-28) |
|---|---|---|---|
| `gt2_pulley_20t.step` | `parts/wrist/gt2_pulley_20t.py` | SolidWorks re-export of the downloaded `GT2_20T` model (148 faces, real teeth, Ø5 bore, 6 mm belt, Ø16 flanges, 2 set screws) | `gt2_pulley_20t_bore5_w6` **tried and rejected**: analytic simplified (10 faces, Ø18.2 flanges, 10 mm long — beyond `COTS_FRAME_TOL_MM`) |
| `gripper_rail_6mm.step` | `parts/gripper/gripper_rail_6mm.py` | SolidWorks export, Ø6 × 125 mm rail | no Ø6 round rod in the catalog (only 8/10/12 mm supported rails) |
| `mg996r_servo.step` | `parts/gripper/mg996r_servo.py` | SolidWorks re-export of a third-party MG996R model; original: `gripper/Servo Motor MG996R 3D Model.step` in the SolidWorks export tree | `MG996R`/`towerpro` not in the catalog; only generic servo envelopes — keep |
| `mg996r_horn.step` | `parts/gripper/mg996r_horn.py` | idem; original `…/gripper/Servo MG996R Horn.step` | not a catalog part |
| `nema17_pancake.step` | `parts/wrist/nema17_pancake.py` | the 7-part pancake-motor sub-assembly of the SolidWorks arm, flattened into one part by `tools/reference/extract_placements.py` | `stepper_motor_nema17_l0020_single_shaft` exists but is analytic simplified — keep |
| `bearing_625.step` | `parts/cycloidal/bearing_625.py` | step.parts `bearing_625_2rs_sealed_simple` (Ø16 × 5, axis Z standing on z=0 — identity `VENDOR_TO_REF`) | adopted; the reference is the drive repo's annulus |
| `nema17_40mm.step` | `parts/joints/nema17_40mm.py` | the motor body of the SolidWorks "nema17x40_with_mks" kit export, split off by `tools/reference/split_mks_motor.py` (re-framed like the drive motor — face z=0, body −Z, shaft +Z, D-flat +Y — its own boss + 23 mm shaft cut off and the drive's `pilot()` fused on and its `shaft()` as the second solid; identity `VENDOR_TO_REF`) | the split IS the reference (`same_as_reference`); the user's own export, not a catalog part |
| `nema17_48mm.step` | `parts/cycloidal/nema17_48mm.py` | composed by `tools/reference/split_mks_motor.py --write drive`: the x48 kit export's real 48 mm body (front plate, housing, back plate, its two Ø22 bearings, connector, rotor; tie rods left out, the kit's M3x30 replace them) with its own boss + shaft cut off and the drive's `lib/cycloidal/motor.py pilot()` + `shaft()` fused on (the envelope's exact interface); identity `VENDOR_TO_REF` | the reference stays the drive repo's envelope (the vendor's bbox is within `COTS_FRAME_TOL_MM`: connector +0.85 on +Y). The datasheet 17HS19-2004S1 ships a 24 mm / 15 mm-D-cut shaft (the x48 export's own) - the drive assumes the 22 mm one (unconfirmed: `docs/open_issues.md`) |
| `mks_servo42d.step` | `parts/joints/mks_servo42d.py` | the Servo42D_Assem (PCB + cover) + 4 standoffs + 4 M3x30 of the same export (z=0 at the motor's rear face, stack −Z, screws to z +19.6) | one board kit per MKS motor — every `nema17_40mm` and `nema17_48mm` (`tools/bom.py` counts them) |

Purchased parts **without** a vendor file build their `_envelope()` — the drive repo's simplified model or, for a
native COTS part, the envelope accepted by `tools/reference/import_native.py` (either also the reference STEP), or
for a part with no reference (`lib/reference.py NO_REFERENCE`) the design's own;
`test_cots_vendor_matches_reference_frame` leaves them out, and `./cadtool python tools/bom.py` marks each bought line
`vendor` or `envelope`. The pattern parts (pins, screw and nut sets) have no catalog model: the catalog has single
fasteners only. The catalog searches that found nothing usable:

| Part module | Why no catalog model (searched 2026-08-28 unless dated) |
|---|---|
| `parts/cycloidal/bearing_6003.py` | `bearing_6003_2rs_sealed_simple` **tried and rejected**: the file is a Ø24 × 8 bearing (a 628 size), not 17 × 35 × 10 |
| `parts/cycloidal/bearing_6814.py` | no 6814 / 61814 entry in the catalog (search and direct ids 404) |
| `parts/joints/bearing_6806.py` | no 6806 / 61806 / 30 x 42 x 7 entry (2026-09-25: the catalog's 132 bearings stop at a 20 mm bore) - a native COTS part (`lib/reference.py NATIVE_COTS`, reference `reference/native/`) |
| `parts/joints/gt2_idler_20t.py` | `gt2_smooth_idler_bore5_w6` / `gt2_flanged_smooth_idler_bore5_w6` **tried and rejected** (2026-09-29): one generic model under both ids - Ø18.2 flanges on a Ø15.2 seat, 10 mm long, three loose solids with no bore - where the seller's drawing is a Ø12.1 seat, 9 mm long; a native COTS part built from that drawing (`lib/belts.py GT2_IDLER_*`) |
| `parts/joints/ky003_hall_sensor.py` | no KY-003 module and no A3144 (2026-09-29: "KY-003", "KY003", "A3144", "hall sensor", "hall effect" find nothing; "sensor module" only other boards; the catalog's `to_92s` is a bare package, not the module) - a native COTS part, its dimensions `lib/sensors.py` |

What to **order** for each purchased part is not here: it is `PURCHASE_SPEC` / `PURCHASE_QTY` /
`PURCHASE_NOTE` in the part module, printed as the buy list by `./cadtool python tools/bom.py` (which also
lists the purchased items that have no geometry at all).

## Where the vendor files come from
Besides `tools/reference/import_solidworks.py` (it seeds a SolidWorks-origin purchased part's file with its
re-export), step.parts downloads (Recipe D, `AGENTS.md`) and `tools/reference/extract_placements.py` (the flattened
`nema17_pancake.step`), vendor files come from `tools/reference/split_mks_motor.py`: it splits the
"NEMA 17 x 40 + MKS SERVO42D" kit export (`lib/reference.py MKS_EXPORT_NAME`, a raw export not in the repo)
by GEOMETRY into `vendor/nema17_40mm.step` and `vendor/mks_servo42d.step` (what each holds and its frame: the table
above); `import_solidworks.py` then mirrors both into `reference/solidworks/` (export path `None`, the
`nema17_pancake` pattern). `--write drive` composes `vendor/nema17_48mm.step` for the drive motor from the x48
export's body (`MKS48_EXPORT_NAME`) the same way - every kit motor carries the drive motor's interface (`AGENTS.md`);
the drive keeps its parametric envelope as the reference (`tools/cycloidal/import_cadquery.py --only nema17_48mm`
writes its `vendor` block).
