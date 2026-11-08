# reference/ — SolidWorks reference geometry

**Purpose:** the original design, as exported from SolidWorks, renamed to the clean part names used everywhere in `cad/`
(`solidworks/`) — plus, for the cycloidal drive, the CadQuery exports of the `cycloidal_drive` repo the build123d parts
were ported from (`cycloidal/`) — and, for the parts designed here with no external origin (`lib/reference.py NATIVE` /
`NATIVE_COTS`), their **accepted builds** (`native/`, written once by `tools/reference/import_native.py`). These files
are **immutable inputs** (committed): `tests/test_reference_match.py` compares the converted parts and the native ones
against them (the drive's ported parts: `tests/cycloidal/test_port.py`). The rules for them — never edit, the
regeneration order, the manifest, Recipe E for a new export — are [`AGENTS.md`](AGENTS.md); this file is the record of
where they came from and of `placements.json`'s schema.

**What is here, per file:** `manifest.json` — each reference's product, source export (or CadQuery builder), units,
solids, bounding box, bytes and sha256, and its `vendor` block. **Which part maps to which product:** `lib/reference.py`
(`CUSTOM`, `COTS`, `DESIGNED` / `CYCLOIDAL_COTS`, `NATIVE` / `NATIVE_COTS`; the products dropped from the design and
why: `SKIPPED_PRODUCTS`; the retired ones whose records stay: `RETIRED_PRODUCTS`). How many of each the arm uses:
`./cadtool python tools/bom.py`.

## Provenance
- Source tree: the SolidWorks 2026 STEP AP214 exports of 2026-08-27, **not in git, not needed on a machine and with
  no fixed place** — the committed copies here are the inputs, the raw tree only for re-running the derivation tools,
  which take its path (`--src` / `--src48` / `--monolith`).
- Full assembly: `final Arm Assembly Fully Movable.STEP` (inch units; its bytes and sha256: `manifest.json` `monolith`)
  — not committed; `placements.json` captures its structure.
- The NEMA 17 + MKS SERVO42D kit exports, `mks/nema17x40_with_mks.step` and `mks/nema17x48_with_mks.step` under the
  source tree (SolidWorks 2026, 2026-09-21; `lib/reference.py MKS_EXPORT_NAME` / `MKS48_EXPORT_NAME` with their units
  and sha256) — not committed and kept nowhere (re-export from SolidWorks if the split must be redone; the sha256
  identifies each). What `tools/reference/split_mks_motor.py` derives from them: its docstring.
- The cycloidal drive's references: the CadQuery builders' exports at `cycloidal_drive@2f1f67d`
  (`docs/cycloidal_drive.md` Provenance).
- Measured exports (`lib/reference.py MEASURED`: product, file name, sha256): nothing of them is in this folder — their
  numbers are in the parts' params and tests (`parts/AGENTS.md` Part states → *measured*).
- `gt2_pulley_20t`'s SolidWorks product name carries the Cyrillic configuration name `Конфигурация1`
  ("Configuration1"); it never enters our labels.

## The SolidWorks assembly's nodes
- **Designed module:** `New_cyloidal_assembly` (sic; path 1.3) is recorded in `placements.json` as the designed module
  `cycloidal_drive#1`: its pose (the record's `world`) places `assemblies/cycloidal_drive.py`, whose contents come from
  code (`lib/cycloidal`), not from the SolidWorks node. The node's own totals / bbox stay in the record's `solidworks`
  block as a cross-check (`tests/cycloidal/test_assembly.py`); the walker does not descend into it.
- **Skipped:** the zero-geometry assembly-skeleton STEPs, the `base/step/` re-exports and the pancake motor's internal
  parts (flattened into `vendor/nema17_pancake.step`); the products the design dropped (`lib/reference.py
  SKIPPED_PRODUCTS`: the link caps, `gripper_clamp_bracket` and the SolidWorks gripper) have no reference STEP or
  manifest entry, and `placements.json` keeps their poses under `skipped`. (The caps were modelled in assembly context,
  their geometry up to ~1 m from the part origin.)
- **Retired:** `j3_coupler#1` / `#2` (`lib/reference.py RETIRED_PRODUCTS`) — the part is gone, its records stay among
  the occurrences: `lib/placements.py RETIRED`.

## placements.json
`occurrences[]`: one record per placed part (`kind: part`) or sub-assembly (`kind: module`)
with `key` (`<part>#<n>`), `path` (SolidWorks tree path), `parent` (module key or null),
`rel` / `world` placements as `{position, rotation_xyz_deg, matrix_3x4}`
(`Location(position, rotation_xyz_deg)` reproduces them; intrinsic XYZ Euler, degrees),
world bounding boxes, solid counts and volumes; `expected` totals (every `kind: part` record, the mounted and the
retired ones included; `tests/test_placements.py` checks them); `designed_modules[]` (the keys of `kind: module,
designed: true` records, which carry `rel`/`world` and `source` instead of solids/volume, plus a `solidworks`
cross-check block when a SolidWorks node places the module, or a `mount` block when `lib/mounts.py MODULE_MOUNTS`
does - those are listed under `mounted[]` too); `mounted[]` (the keys of the records with a `mount` block — what
`lib/mounts.py MOUNTS` / `MODULE_MOUNTS` declare, written by `tools/reference/mount_placements.py`: `path` / `parent` /
`label_in_monolith` null, `rel == world = host world * mount.frame_in_host`, `mount.host` the record it is declared
on - a SolidWorks one (a retired one for a re-seated pulley) or another mounted one: a board's motor, a bolt set's
pulley, a nut set's screws); `skipped[]` (one record per SolidWorks node whose product the design dropped,
`lib/reference.py SKIPPED_PRODUCTS` (a module's parts are its leaves): `path`, `label`, `reason`, leaves / solids /
volume and the `rel` / `world` pose, moved there by `tools/reference/mount_placements.py`).
The capture records stay as extracted when the design moves or turns what lies beyond a link (e.g. its length, a
module in between): `lib/placements.py SHIFTS` / `TURNS` move the records beyond it when they are read
(`assemblies/AGENTS.md` "Shifted occurrences", "Turned occurrences").
