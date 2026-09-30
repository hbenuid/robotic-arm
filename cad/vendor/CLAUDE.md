# vendor/ — the purchased-part STEP files

Loads when you work in `vendor/`. Which file is current for each part, what was tried in the catalog and where each
file came from is `README.md`; this file holds the rules.

## Rules
- A vendor file may be **replaced** by a better model; the part's reference (`reference/<origin>/<name>.step`) may
  not — it fixes the frame and size, and `tests/test_parts_convention.py::test_cots_vendor_matches_reference_frame`
  checks the vendor file still occupies the reference's bounding box (±1.5 mm) after the part's `VENDOR_TO_REF`.
- **Every motor carries the drive motor's interface**: `lib/cycloidal/motor.py pilot()` + `shaft()` (`MotorParams`:
  the pilot, the shaft, its D-cut with the flat at `shaft_dcut_flat / 2` from the axis like the eccentric shaft's
  D-bore) are fused onto every motor body `tools/reference/split_mks_motor.py` writes, in place of the export's own.
- **Written once, on one machine:** build123d's `export_step` writes the time into the STEP header, so re-running a
  vendor-producing tool changes the file's bytes (and its manifest sha) with identical geometry. Vendor files are
  committed as LFS and never regenerated on the other machine — the tools' selectors
  (`split_mks_motor.py --write kit|drive`) leave the others alone.
- A NEW vendor file needs `./cadtool gen parts/<group>/<name>.py --force` on its part once (why: `cad/CLAUDE.md`
  Gotchas).

## Recipe D — produce or swap a vendor STEP
From an export → `./cadtool python tools/reference/split_mks_motor.py --write kit|drive|all --src <x40 kit> --src48
<x48 kit>` (the motor kits; each path only where `--write` needs it). From
the catalog:
```bash
./cadtool parts "GT2 20" --limit 20                         # search (ANDed tokens; facets --tag/--family/--standard)
./cadtool parts --id <id> --download --filename <name>.step --overwrite   # -> vendor/<name>.step (sha256 verified)
./cadtool inspect vendor/<name>.step --planes   # frame of the new model (leaf bbox + planar faces)
# set VENDOR_TO_REF in parts/<group>/<name>.py so the model lands in the reference frame (reference/<origin>/<name>.step)
./cadtool pytest -k <name>                                  # vendor-frame + envelope + convention tests
```
Then the manifest: `./cadtool python tools/reference/import_solidworks.py --src <export tree>` for SolidWorks-origin parts,
`./cadtool python tools/cycloidal/import_cadquery.py --only <name>` for the drive's parts (their manifest is owned by
that tool) → `./cadtool gen parts/<group>/<name>.py --force` → Recipe C (`cad/CLAUDE.md`), on one machine only.

A catalog model worse than the SolidWorks re-export is restored:
`cp reference/solidworks/<name>.step vendor/<name>.step && ./cadtool python tools/reference/import_solidworks.py --src <export tree>`;
for the drive's parts a worse catalog model is simply deleted — the envelope takes over and the manifest entry loses
its `vendor` block. Record what was tried in `README.md`.
