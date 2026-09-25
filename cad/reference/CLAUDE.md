# reference/ — the immutable reference inputs

Loads when you work in `reference/`. What is here and where each file came from — the provenance, the naming map,
the designed module, the skipped products, the `placements.json` schema — is `README.md`; this file holds the rules.

## Rules
- Every file here is an **immutable input**, committed as a Git LFS object: its checksum is locked in `manifest.json`
  (whose `file` field names its origin directory; `lib.reference.path_of(name)` resolves it) and
  `tests/test_reference_match.py` compares every converted part against it. Regenerate with its tool — never edit.
- Each import tool owns its own `manifest.json` entries and keeps the others'; all build them with `lib/manifest.py`
  (`read()` / `write()` / `entry()` — no part imports it, so editing it never makes a part stale). The cycloidal
  export runs in the OLD repo's CadQuery venv — never ours.
- `lib.reference.step_units()` only tells inch from mm: a centimetre file (the x48 kit export) is reported as mm.
  OCCT converts every unit correctly on import; the manifest's `units` field is the one that would lie.

## Regenerating, in this order
- `./cadtool python tools/reference/split_mks_motor.py` (once, on one machine — STEP bytes are per machine),
  `./cadtool python tools/reference/import_solidworks.py` (copies + `manifest.json`), then
  `./cadtool python tools/reference/extract_placements.py --no-pancake` (`placements.json`, the mounted records of
  `lib/mounts.py` appended; without the flag also `vendor/nema17_pancake.step`, whose bytes would then change),
  then `import_solidworks.py` once more so the manifest describes the extracted pancake. A changed mount alone:
  `./cadtool python tools/reference/mount_placements.py` (merge mode, no monolith needed).
- Cycloidal drive (manifest `origin: cycloidal_drive@2f1f67d`, kind `designed` / `cots`): in the old
  repo `cd ../cycloidal_drive && uv run python ../robotic-arm/cad/tools/cycloidal/export_cadquery.py`
  (CadQuery venv, writes its git-ignored `export/step/house/`), then here
  `./cadtool python tools/cycloidal/import_cadquery.py` (copies + merges its manifest entries;
  `import_solidworks.py` leaves them alone).
- Native parts (manifest `origin: native`, kind `native` / `cots`): `./cadtool python tools/reference/import_native.py
  [--only NAME] [--force]` builds the part in-process and exports it to `native/<name>.step` (build123d's STEP header
  carries the time: written once, on one machine, committed as LFS; `--force` accepts a changed design); both other
  tools keep the entries.

## Recipe E — a new SolidWorks / vendor export arrives
Keep it OUTSIDE the tree with a lowercase `.step` name (the raw
exports are never committed; the tools take `--src` / `--monolith`, default `lib/reference.py DEFAULT_SOURCE_DIR`
or `ARM_REFERENCE_SRC` where `README.md` Provenance says); record file, size, sha256 and what it is under
`README.md` Provenance; name it in `lib/reference.py` (`MONOLITH_NAME`, `MKS_EXPORT_NAME`, … or a `CUSTOM` / `COTS`
row); measure before trusting it (`./cadtool inspect <file> --planes` — units, frame, shaft / pilot / bolt pattern);
then Recipe A (`parts/CLAUDE.md`) or D (`vendor/CLAUDE.md`). What the CAD keeps is the derived, committed copy
(`reference/`, `vendor/`) — the raw file can be discarded afterwards.
