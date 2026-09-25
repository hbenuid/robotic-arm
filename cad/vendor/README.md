# vendor/ — purchased-part STEP files (current best model per part)

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
| `nema17_40mm.step` | `parts/joints/nema17_40mm.py` | the motor body of the SolidWorks "nema17x40_with_mks" kit export, split off by `tools/reference/split_mks_motor.py` (2 solids; re-framed like the drive motor — face z=0, body −Z, shaft +Z, D-flat +Y — its own boss + 23 mm shaft cut off and the drive's `pilot()` + `shaft()` fused on; identity `VENDOR_TO_REF`) | added 2026-09-21; the split IS the reference (`same_as_reference`); the user's own export, not a catalog part |
| `nema17_48mm.step` | `parts/cycloidal/nema17_48mm.py` | composed by `tools/reference/split_mks_motor.py --write drive`: the x48 kit export's real 48 mm body (front plate, housing, back plate, its two Ø22 bearings, connector, rotor - 7 solids; tie rods left out, the kit's M3x30 replace them) with its own boss + shaft cut off and the drive's `lib/cycloidal/motor.py pilot()` + `shaft()` fused on (the envelope's exact interface); identity `VENDOR_TO_REF` | added 2026-09-21; the reference stays the drive repo's envelope (the vendor's bbox is within 1.5 mm: connector +0.85 on +Y). The datasheet 17HS19-2004S1 ships a 24 mm / 15 mm-D-cut shaft (the x48 export's own) - the user's motor is the 22 mm one |
| `mks_servo42d.step` | `parts/joints/mks_servo42d.py` | the Servo42D_Assem (PCB 4 solids + cover) + 4 standoffs + 4 M3x30 of the same export (13 solids; z=0 at the motor's rear face, stack −Z, screws to z +19.6) | added 2026-09-21; one board kit per MKS motor — the three 40 mm ones and the drive's 48 mm |

Purchased parts **without** a vendor file (their `_envelope()` — the drive repo's simplified model or, for a
native COTS part, the envelope accepted by `tools/reference/import_native.py` - also the reference STEP — is the
geometry; `test_cots_vendor_matches_reference_frame` skips them):

| Part module | Why no catalog model (2026-08-28) |
|---|---|
| `parts/cycloidal/bearing_6003.py` | `bearing_6003_2rs_sealed_simple` **tried and rejected**: the file is a Ø24 × 8 bearing (a 628 size), not 17 × 35 × 10 |
| `parts/cycloidal/bearing_6814.py` | no 6814 / 61814 entry in the catalog (search and direct ids 404) |
| `parts/joints/bearing_6808.py` | no 6808 / 61808 / 6908 entry (2026-09-22: nothing above a 17 mm bore in the catalog) - a native COTS part (`lib/reference.py NATIVE_COTS`, reference `reference/native/`) |
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

## Recipe D — produce or swap a vendor STEP
From an export → `./cadtool python tools/reference/split_mks_motor.py --write
kit|drive|all` (the motor kits); from the catalog → `./cadtool parts "<query>"` then `--id … --download` (below);
`./cadtool inspect vendor/<name>.step --planes` and set `VENDOR_TO_REF` if the frame differs → `import_solidworks.py`
(SolidWorks-origin parts) or `tools/cycloidal/import_cadquery.py --only <name>` (drive parts) for the manifest →
`./cadtool gen parts/<group>/<name>.py --force` → Recipe C (`cad/CLAUDE.md`). Vendor STEP bytes are written once, on one machine.

## Where the vendor files come from
Besides step.parts downloads (above) and `tools/reference/extract_placements.py` (the flattened
`nema17_pancake.step`), the third producer of vendor files is `tools/reference/split_mks_motor.py`: it splits the "NEMA 17 x 40 + MKS SERVO42D"
kit export (`lib/reference.py MKS_EXPORT_NAME`, outside the repo next to the monolith) by GEOMETRY into
`vendor/nema17_40mm.step` (the motor body, re-framed like the drive motor - face z=0, body −Z, shaft +Z, D-flat +Y -
with the drive's `lib/cycloidal/motor.py pilot()` + `shaft()` fused on in place of the export's own) and `vendor/mks_servo42d.step` (board + cover + standoffs + M3x30,
z=0 at the motor's REAR face, stack −Z); `import_solidworks.py` then mirrors both into `reference/solidworks/`
(`rel=None`, the `nema17_pancake` pattern). `--write drive` composes `vendor/nema17_48mm.step` for the drive motor from
the x48 export's real 48 mm body (`MKS48_EXPORT_NAME`; 7 solids, tie rods left out) with the same `pilot()` + `shaft()`
fused on - **every motor carries the drive motor's interface** (`MotorParams`: Ø22 × 2 pilot, Ø5 × 22 shaft, 18 mm D-cut,
the D-flat at `shaft_dcut_flat / 2` from the axis like the eccentric shaft's D-bore); the drive keeps its parametric envelope as the reference (`tools/cycloidal/import_cadquery.py --only nema17_48mm`
writes its `vendor` block). build123d's STEP writer stamps the time into the header: written once, committed as LFS -
never regenerated on the other machine, and a NEW vendor file needs `gen --force` on its part (the gate only tracks
inputs the last build read).

- build123d's `export_step` writes the time into the STEP header, so re-running a vendor-producing tool changes the
  file's bytes (and its manifest sha) with identical geometry — write vendor files once, on one machine, and use
  the tools' selectors (`split_mks_motor.py --write kit|drive`) to leave the others alone.
