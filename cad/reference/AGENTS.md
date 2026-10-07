# reference/ — the immutable reference inputs

Loads when you work in `reference/`. What is here and where each file came from — the provenance, the naming map,
the designed module, the skipped products, the `placements.json` schema — is `README.md`; this file holds the rules.

## Rules
- **These references were a bootstrap, on their way out** — the user's direction, not a current task. The CAD is to
  stand on its own code (parametric builds locked by numeric tests, poses declared in code like `lib/mounts.py`'s)
  with nothing here needed: the SolidWorks and CadQuery exports, `placements.json`, `native/` (`vendor/` is bought-part
  data, not a reference). So: add no new dependency on a file here in a test, a lock or a tool; a new part or a new
  export gets no reference file (`parts/AGENTS.md` Part states: *no reference*, *measured*); when a task touches a
  check measured against a reference, say what code or test could replace it; start no migration of the existing
  references unasked.
- Every file here is an **immutable input**, committed as a Git LFS object: its checksum is locked in `manifest.json`
  (whose `file` field names its origin directory; `lib.reference.path_of(name)` resolves it) and
  `tests/test_reference_match.py` compares every converted part against it. Regenerate with its tool — never edit.
- Each import tool owns its own `manifest.json` entries and keeps the others'; all build them with `lib/manifest.py`
  (`read()` / `write()` / `entry()` — no part imports it, so editing it never makes a part stale). The cycloidal
  export runs in the OLD repo's CadQuery venv — never ours.
- `lib.reference.step_units()` only tells inch from mm: a centimetre file (the x48 kit export) is reported as mm.
  OCCT converts every unit correctly on import; the manifest's `units` field is the one that would lie.

## Regenerating, in this order
- Each of these reads raw exports, so each takes their path (`README.md` Provenance names the files):
  `./cadtool python tools/reference/split_mks_motor.py --src <x40 kit> --src48 <x48 kit>` (once, on one machine — STEP
  bytes are per machine), `./cadtool python tools/reference/import_solidworks.py --src <export tree>` (copies +
  `manifest.json`), then
  `./cadtool python tools/reference/extract_placements.py --monolith <full assembly> --no-pancake` (`placements.json`, the mounted records of
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
Read it where the user put it, by the path they give: never move, rename or copy it elsewhere on the machine (the
committed copies its tool derives in `reference/` / `vendor/` excepted), and never make a folder for it outside the
repo (the raw exports are never committed and have no fixed place — every tool that reads one takes its
path: `--src` / `--src48` / `--monolith`). Record its file name as handed over, size, sha256 and what it is under
`README.md` Provenance; name it in `lib/reference.py` (`MONOLITH_NAME`, `MKS_EXPORT_NAME`, … or a `CUSTOM` / `COTS`
row); measure before trusting it (`./cadtool inspect <file> --planes` — units, frame, shaft / pilot / bolt pattern);
then Recipe A (`parts/AGENTS.md`) or D (`vendor/AGENTS.md`). What the CAD keeps is the derived, committed copy
(`reference/`, `vendor/`) — the raw file is the user's, to keep or discard. A printed part's export is not copied here
at all: a *measured* conversion (`parts/AGENTS.md` Part states, `lib/reference.py MEASURED`) keeps only its numbers, in
the part's params and tests - the raw file is needed until the tests are committed (its sha256 in `MEASURED`).
