# CAD — build123d models (robotic arm)

**Purpose:** CAD-scoped agent guide — the `gen_step()` part convention, the wrapper → parametric
conversion workflow, shared-dimension rules, assembly placements, purchased parts, tests, tooling.
**Audience:** agent. Human docs: `README.md`. Reference provenance: `reference/README.md`. The
cycloidal drive (spec, port notes, attachment): `docs/cycloidal_drive.md`.
**Last updated:** 2026-08-28. Every commit that changes behaviour, layout or tooling gets a dated entry in the
root `CHANGELOG.md` and bumps the `Last updated` line of the docs it touches.

`cad/` is a **separate uv project** (Python 3.12, build123d 0.10, cadgen 0.4.x) inside the
robotic-arm repo; the root motor-control project never depends on it.

## Running things (always via `./cadtool` or `uv run`, from `cad/`)
- **Never call bare `python`** — the system Python is 3.14 without build123d. Python is pinned
  to **3.12** (vtk wheels) — don't bump it. Never run CAD code with the root repo's venv.
- Plugin: `cad@text-to-cad` **v0.4.x** (`~/.claude/plugins/cache/text-to-cad/cad/<ver>/skills/`);
  `cadgen==<same version>` is a locked dependency — bump both together. Plugin updates need
  `git-lfs` on `PATH`.
- `./cadtool gen parts/<group>/<name>.py` (alias `step`) — build and write the **committed**
  `parts/<group>/<name>.step` (+ `parts/<group>/__cadgen__/` viewer package, git-ignored;
  `./cadtool clean [--all]` deletes every such cache).
- `./cadtool gen assemblies/arm.py` — `assemblies/arm.step` (git-ignored).
- `./cadtool export parts/<group>/<name>.py --stl [path]` — mesh sidecars; paths resolve **beside
  the source** (`--stl ../../exports/<name>.stl`). Bare `--stl` → sibling `<name>.stl`.
- `./cadtool inspect refs <file.step> --facts --planes --positioning` (+ `measure|align|frame|diff|interfere`).
- `./cadtool snapshot --input assemblies/arm.step --output snapshots/arm.png --size-profile assembly --view-labels`
  (or `--job -` with a JSON job on stdin). Snapshot review is mandatory after visible geometry changes.
- `./cadtool viewer` — CAD Viewer for this folder: `http://127.0.0.1:3245/<abs cad>?file=<rel path>`
  (Python backend in our venv; 12 h auto-stop). Hand every created/updated STEP to it.
- `./cadtool validate <file.urdf|.srdf|.sdf>`, `./cadtool parts "<query>"`, `./cadtool skill <skill> <tool> …`.
- `./cadtool pytest [-m "not slow"]`, `./cadtool python -m assemblies.arm`,
  `./cadtool python -m assemblies.cycloidal_drive [--totals]`.
- `uv add <pkg>` for deps (commit `pyproject.toml` + `uv.lock`); never `pip install`.
- Every committed STEP/STL under `cad/` is a **Git LFS** object (`/.gitattributes`): a checkout
  showing ~130-byte pointer files (checksum tests, `import_step` and the plugin fail on them)
  needs `git lfs pull`; every regenerated STEP is a new LFS object.
- Cycloidal references: `cd ../cycloidal_drive && uv run python ../robotic-arm/cad/tools/cycloidal/export_cadquery.py`
  (the OLD repo's CadQuery venv — never ours), then `./cadtool python tools/cycloidal/import_reference.py`.

## Authoring a part (`parts/<group>/<name>.py`)
Parts live in subsystem groups (`base`, `joints`, `wrist`, `gripper`, `cycloidal`; templates in
`parts/_templates/`). The group is only a directory: the part's NAME is its module stem, unique
across groups (`import parts` raises on a duplicate), and it keys the manifest,
`reference/<origin>/<name>.step`, `placements.json` and the URDF links. `parts.names()` /
`parts.load(name)` (a stdlib-only directory scan in `parts/__init__.py`) are the only way code
reaches a part — never `from parts import <name>`; note `parts.base` is the *group package*, the
part is `parts.load("base")`. Every part MUST (enforced by `tests/test_parts_convention.py`):
- define a module-level **`gen_step()`** that **returns** a valid, labelled Part/Compound at its
  **local origin** — the assembly owns placement; label == module name;
- have **no import side effects** (`show()` only under `if __name__ == "__main__":`);
- keep the 2-line path shim (`sys.path.insert(0, parents[2])` — two levels below `cad/`) above the build123d imports —
  the plugin's generator runner restores `sys.path` after import and does not seed the cwd, so
  anything imported lazily inside `gen_step()` must re-assert the path (see
  `assemblies/_occurrences.py`);
- pull shared dims from `lib/params.py`.

**Part states.** Custom parts declare `REFERENCE = NAME`, `CONVERTED` and `LOCAL_FROM_REF`:
- *wrapper* (`CONVERTED = False`, from `_templates/wrapper.py`): `gen_step()` returns
  `reference/solidworks/<name>.step` in the SolidWorks part-file frame — the day-one state of all 20 custom parts;
- *parametric* (`CONVERTED = True`, from `_templates/designed.py`): real build123d. To convert: rewrite
  `gen_step()`, set `CONVERTED = True`, optionally set `LOCAL_FROM_REF` (reference frame → new
  local frame; the assemblies compose `placement * LOCAL_FROM_REF⁻¹`, so `placements.json` never
  changes), then `./cadtool pytest tests/test_reference_match.py -k <name>` (volume ±0.5 %,
  bbox ±0.2 mm; per-part `REF_VOL_TOL` / `REF_BBOX_TOL`) and `./cadtool gen parts/<group>/<name>.py`.
- *designed* (`CONVERTED = True`, `REFERENCE = NAME`, registered in `lib/reference.py DESIGNED`):
  the cycloidal drive's printed parts — the reference is the CadQuery export they were ported
  from; `tests/cycloidal/test_port.py` additionally demands identical face sets / tessellations.
  Their geometry helpers live in `lib/cycloidal/` and each module exposes `build(cfg)` for tests.
- Multi-body parts are registered in `MULTI_BODY` in `tests/test_parts_convention.py`.
- Keep the importable `parts/<group>/<name>.py` naming (tests/assemblies import them through
  `parts.load`); the plugin's `<name>.step.py` entry naming is deliberately not used. A part and its
  STEP stay siblings (the viewer pairs them into one entry).

## Purchased (COTS) parts (`parts/_templates/cots.py`)
`COTS = True`, `MASS_G` (datasheet grams), `VENDOR_STEP = vendor/<name>.step`, `VENDOR_TO_REF`
(vendor-file frame → SolidWorks frame; identity for the SolidWorks re-exports); `gen_step()` is
hybrid (vendor STEP if present, else `_envelope()` from `lib.params`, both in the SolidWorks
frame `placements.json` assumes). `reference/solidworks/<name>.step` keeps the SolidWorks re-export of
every COTS part as the frame/size reference (for the drive's purchased parts: the CadQuery export of
their simplified model in `reference/cycloidal/`, `lib/reference.py CYCLOIDAL_COTS`; `path_of()`
resolves the origin); `test_cots_vendor_matches_reference_frame`
(bbox within 1.5 mm, skipped when there is no vendor file - the envelope is then the geometry) and
`test_cots_envelope_tracks_reference_bbox` guard vendor swaps. Swap procedure and what has been
tried: `vendor/README.md` (`./cadtool parts …`, step.parts).

## Shared dimensions (DRY)
`lib/params.py` is the single source of truth: mm and grams, every constant tagged
`[MEASURE] / [DATASHEET] / [DESIGN] / [REFERENCE] / [ESTIMATE]` with a derivation comment.
`lib/` never imports `parts/`. Docs name constants, never numbers. Datum: the SolidWorks capture
frame is **Y up** (J1 axis); the URDF base frame (REP-103) lives in `robot/frames.py`.
The cycloidal drive's own dimensions are `lib/cycloidal/params.py` (`DriveConfig`, frozen
dataclasses, variants via `dataclasses.replace`); `lib/params.py` re-exports the interface values
(`CYCLOIDAL_*`, masses) from it - never retype a drive number.

Changing a shared dimension — touchpoints in order:

| # | Edit | What |
|---|---|---|
| 1 | `lib/params.py` | the value (keep tag + derivation) |
| 2 | `tests/test_params_invariants.py` | the lock; `./cadtool pytest -m "not slow"` |
| 3 | `./cadtool gen parts/<group>/<affected>.py` | regenerate the committed STEP(s) |
| 4 | `./cadtool pytest` + `./cadtool gen assemblies/arm.py` + snapshot | verify geometry and fit |

## Assembly & placements
- `reference/placements.json` (from `tools/reference/extract_placements.py`) holds every occurrence:
  `rel` (to its parent node) and `world`, as `Location(position, rotation_xyz_deg)`; keys
  `"<part>#<n>"`, module `"gripper#1"`. Treat it as an immutable input.
- `assemblies/arm.py` / `gripper.py`: `OCCURRENCES = [(part, role|None, key), …]` in SolidWorks
  document order; `assemblies/_occurrences.py` places a **fresh** `gen_step()` copy per occurrence
  with `.moved(rel * LOCAL_FROM_REF⁻¹)` and locates the modules **in place** (`.locate`).
  Roles (`j2`/`j3`, `1`/`2`) only disambiguate duplicates; rename when joint semantics arrive.
- `assemblies/cycloidal_drive.py` is **code-driven**: rows are `(part, role, Location)` from
  `lib/cycloidal stack_positions` (`add_located`); its placement key `cycloidal_drive#1` is a
  `designed` module record in `placements.json` (pose from the SolidWorks node, no leaf records,
  `solidworks` cross-check block; `tools/reference/extract_placements.py` never descends into
  `DESIGNED_MODULES`). `world_rows(key)` expands a designed-module key into world-placed parts for
  links and inertials. Keep `EXPECTED` in step with the geometry (`--totals`).
- When adding source-level joints, use `cadgen.assembly.AssemblyHelper` frames/mates, keep
  placements parameter-driven, and validate with `inspect align/measure/frame`.

## Robot description (`robot/`)
- `robot/arm.urdf` is the **source of truth** (hand-authored XML, ledger comment on top);
  `arm.srdf` pairs by colocation + robot name; `arm.sdf` is derived from the URDF. Never build a
  Python generator for them - `tools/robot/frames.py --urdf-draft/--sdf-draft` only prints scaffolding
  to copy numbers from, and `--check` (run by `tests/test_robot.py`) catches drift.
- `robot/frames.py` is the kinematic SSOT: `LINKS` (occurrence keys per rigid link) and `JOINTS`
  (axis point/direction in the SolidWorks capture frame, limits from `lib/params.py`). Joint frame:
  Z on the axis, X along the child link; child link frame = joint frame at capture, so **all joints
  are 0 at the capture pose** and mesh origins are identity. Moving an occurrence between links or
  changing an axis = edit `frames.py`, re-export meshes, re-derive the affected numbers, re-check.
- Meshes: `tools/robot/export_link_meshes.py` (mm STL per link, `scale 0.001` in the XML); never hand-edit.
  Inertials come from OCP `BRepGProp` (printed parts at `PETG_DENSITY`, COTS at `MASS_G`).
- Validate with `./cadtool validate <file> --strict` and snapshot with
  `./cadtool skill urdf snapshot --input robot/arm.urdf --output snapshots/x.png` after every edit;
  hand `.urdf` files to the viewer (`?file=robot/arm.urdf`).
- Placeholders to confirm before real use: joint limits/effort/velocity (`lib/params.py`), axis signs,
  jaw travel, the link-membership assumptions listed in the URDF ledger. The cycloidal drive is
  physically the shoulder-pitch joint between `j1_coupler` and `j1_link` but is NOT a joint yet: its
  module key rides in `LINKS["link1"]`; which MKS motor drives which joint is unconfirmed;
  wrist_roll and the jaws are not driven by `src/config.py`.

## Tests (`./cadtool pytest`)
`test_parts_convention.py` (contract + geometry for every part, COTS envelopes + vendor frames),
`test_reference_match.py` (manifest checksums; converted parts vs reference), `test_placements.py`
(JSON integrity, tables cover every key once, the designed module record), `test_assembly.py`
(34 + 18 leaves / 50 + 58 solids / volume / bbox vs SolidWorks + the module lock),
`test_params_invariants.py` (locks), `test_robot.py` (link partition, frames, FK at zero = capture,
meshes, inertials, URDF/SRDF/SDF consistency + plugin validators), `tests/cycloidal/` (the
drive: one module per part + housing / purchased / fitment / assembly / port, ~230 tests,
`from tests.cycloidal.helpers import …`). Geometry tests are `slow`. Run pytest only through
`./cadtool pytest` (rootdir `cad/`; the repo-root `tests/` is the unrelated, broken motor-control suite).

## Gotchas (all verified)
- `Compound.volume` skips nested sub-assemblies in build123d 0.10 — use `lib.reference.solid_volume()`.
- `is_valid` is a **property**; `Location.to_tuple()` is deprecated (`tuple(loc)` → two Vectors).
- `.moved()`/`.located()` deep-copy the shape **and its parent chain**; never call them on a child
  of a big imported assembly (copy its `solids()` instead). A shape added to two Compounds is
  silently re-parented — build a fresh `gen_step()` per occurrence.
- `import_step()` converts inch-unit files to mm and keeps the assembly hierarchy (labels mangle
  ` .()` → `_`; `lib.reference.clean_label` mirrors it). `j1_cap`, `j2_cap_1`, `j2_cap_2` have
  geometry far from their part origin — `placements.json` compensates; use `LOCAL_FROM_REF` when converting.
- `uv sync` prunes anything `uv pip install`-ed; `uv run` doesn't — that's why `cadgen` is a
  pyproject dependency, not a manual install.
- Don't compare large STEP/GLB artifacts with `git diff`; compare source, `inspect` output and snapshots.
- New STEPs in a group directory are committed thanks to the recursive `!/cad/parts/**/*.step`
  re-admit in the root `.gitignore` (`git check-ignore -v --no-index cad/parts/<group>/<name>.step`
  from the repo root must print that line); the plugin writes `__cadgen__/` beside every entry.
- Cycloidal discs: chamfer the lobe edges BEFORE cutting holes (the end face must carry only the
  spline edge); the profile is a periodic *interpolating* spline (`Edge.make_spline(periodic=True)`,
  never `make_spline_approx`); OCCT's analytic volume is ~0.3 % off on that face (both ours and the
  reference) - compare tessellations/face sets, not `.volume`. A boolean between the two discs takes
  minutes: probe points instead.
