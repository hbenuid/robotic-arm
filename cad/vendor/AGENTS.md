# vendor/ — the purchased-part STEP files

Loads when you work in `vendor/`. Which file is current for each part, what was tried in the catalog and where each
file came from is `README.md`; this file holds the rules.

## Rules
- A vendor file may be **replaced** by a better model; the part's reference (`reference/<origin>/<name>.step`) may
  not — it fixes the frame and size, and `tests/test_parts_convention.py::test_cots_vendor_matches_reference_frame`
  checks the vendor file still occupies the reference's bounding box (`COTS_FRAME_TOL_MM`) after the part's
  `VENDOR_TO_REF` - collected only for a vendor file it can tell from the reference (`_vendor_differs`: the
  manifest's `same_as_reference` false, or a `VENDOR_TO_REF` that is not `IDENTITY`): a vendor file that IS its
  reference (export path `None`, mirrored into `reference/solidworks/`) gets no frame check, so measure a swap of
  it by hand (`./cadtool inspect --planes`).
- **Every kit motor carries the drive motor's interface** (the pancake keeps its export's):
  `lib/cycloidal/motor.py pilot()` + `shaft()` (`MotorParams`: the pilot, the shaft, its D-cut with the flat at
  `shaft_dcut_flat / 2` from the axis like the eccentric shaft's D-bore) replace the export's own on every motor body
  `tools/reference/split_mks_motor.py` writes - the pilot fused onto the front plate, the shaft fused onto the rotor
  (x48) or added as the body's second solid (x40).
- **Written once, on one machine:** build123d's `export_step` writes the time into the STEP header, so re-running a
  vendor-producing tool changes the file's bytes (and its manifest sha) with identical geometry. Vendor files are
  committed and never regenerated on the other machine — the tools' selectors
  (`split_mks_motor.py --write kit|drive`) leave the others alone.
- A NEW vendor file needs `./cadtool gen parts/<group>/<name>.py --force` on its part once (why: `cad/AGENTS.md`
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
Then the manifest: `./cadtool python tools/reference/import_solidworks.py --src <export tree>` for SolidWorks-origin
parts and a catalog-only one (export path `None`, Recipe A in `parts/AGENTS.md`; it needs the whole tree:
`reference/AGENTS.md` "Regenerating, in this order"),
`./cadtool python tools/cycloidal/import_cadquery.py --only <name>` for the drive's parts (their manifest is owned by
that tool; it reads the drive's CadQuery export - run its exporter first, the same section - or pass `--src`) →
`./cadtool gen parts/<group>/<name>.py --force` → Recipe C (`cad/AGENTS.md`), on one machine only.

A catalog model worse than the SolidWorks re-export is restored:
`cp reference/solidworks/<name>.step vendor/<name>.step && ./cadtool python tools/reference/import_solidworks.py --src <export tree>`;
for the drive's parts a worse catalog model is simply deleted — the envelope takes over, and
`import_cadquery.py --only <name>` drops the manifest entry's `vendor` block. Record what was tried in `README.md`.
