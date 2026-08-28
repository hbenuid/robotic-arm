# CAD — build123d models (robotic arm)

**Purpose:** CAD-scoped agent guide — the `gen_step()` part convention, the wrapper → parametric
conversion workflow, shared-dimension rules, assembly placements, purchased parts, tests, tooling.
**Audience:** agent. Human docs: `README.md`. Reference provenance: `reference/README.md`.

`cad/` is a **separate uv project** (Python 3.12, build123d 0.10, cadgen 0.4.x) inside the
robotic-arm repo; the root motor-control project never depends on it.

## Running things (always via `./cadtool` or `uv run`, from `cad/`)
- **Never call bare `python`** — the system Python is 3.14 without build123d. Python is pinned
  to **3.12** (vtk wheels) — don't bump it. Never run CAD code with the root repo's venv.
- Plugin: `cad@text-to-cad` **v0.4.x** (`~/.claude/plugins/cache/text-to-cad/cad/<ver>/skills/`);
  `cadgen==<same version>` is a locked dependency — bump both together. Plugin updates need
  `git-lfs` on `PATH`.
- `./cadtool gen parts/<name>.py` (alias `step`) — build and write the **committed**
  `parts/<name>.step` (+ `parts/__cadgen__/` viewer package, git-ignored).
- `./cadtool gen assemblies/arm.py` — `assemblies/arm.step` (git-ignored).
- `./cadtool export parts/<name>.py --stl [path]` — mesh sidecars; paths resolve **beside the
  source** (`--stl ../exports/<name>.stl`). Bare `--stl` → sibling `<name>.stl`.
- `./cadtool inspect refs <file.step> --facts --planes --positioning` (+ `measure|align|frame|diff|interfere`).
- `./cadtool snapshot --input assemblies/arm.step --output snapshots/arm.png --size-profile assembly --view-labels`
  (or `--job -` with a JSON job on stdin). Snapshot review is mandatory after visible geometry changes.
- `./cadtool viewer` — CAD Viewer for this folder: `http://127.0.0.1:3245/<abs cad>?file=<rel path>`
  (Python backend in our venv; 12 h auto-stop). Hand every created/updated STEP to it.
- `./cadtool validate <file.urdf|.srdf|.sdf>`, `./cadtool parts "<query>"`, `./cadtool skill <skill> <tool> …`.
- `./cadtool pytest [-m "not slow"]`, `./cadtool python -m assemblies.arm`.
- `uv add <pkg>` for deps (commit `pyproject.toml` + `uv.lock`); never `pip install`.

## Authoring a part (`parts/<name>.py`)
Every part MUST (enforced by `tests/test_parts_convention.py`):
- define a module-level **`gen_step()`** that **returns** a valid, labelled Part/Compound at its
  **local origin** — the assembly owns placement; label == module name;
- have **no import side effects** (`show()` only under `if __name__ == "__main__":`);
- keep the 2-line path shim (`sys.path.insert(0, parent.parent)`) above the build123d imports —
  the plugin's generator runner restores `sys.path` after import and does not seed the cwd, so
  anything imported lazily inside `gen_step()` must re-assert the path (see
  `assemblies/_occurrences.py`);
- pull shared dims from `lib/params.py`.

**Part states.** Custom parts declare `REFERENCE = NAME`, `CONVERTED` and `LOCAL_FROM_REF`:
- *wrapper* (`CONVERTED = False`, from `_wrapper_template.py`): `gen_step()` returns
  `reference/<name>.step` in the SolidWorks part-file frame — the day-one state of all 20 custom parts;
- *parametric* (`CONVERTED = True`, from `_template.py`): real build123d. To convert: rewrite
  `gen_step()`, set `CONVERTED = True`, optionally set `LOCAL_FROM_REF` (reference frame → new
  local frame; the assemblies compose `placement * LOCAL_FROM_REF⁻¹`, so `placements.json` never
  changes), then `./cadtool pytest tests/test_reference_match.py -k <name>` (volume ±0.5 %,
  bbox ±0.2 mm; per-part `REF_VOL_TOL` / `REF_BBOX_TOL`) and `./cadtool gen parts/<name>.py`.
- Multi-body parts are registered in `MULTI_BODY` in `tests/test_parts_convention.py`.
- Keep the importable `parts/<name>.py` naming (tests/assemblies import them); the plugin's
  `<name>.step.py` entry naming is deliberately not used.

## Purchased (COTS) parts (`parts/_cots_template.py`)
`COTS = True`, `MASS_G` (datasheet grams), `VENDOR_STEP = vendor/<name>.step`, `VENDOR_TO_REF`
(vendor-file frame → SolidWorks frame; identity for the SolidWorks re-exports); `gen_step()` is
hybrid (vendor STEP if present, else `_envelope()` from `lib.params`, both in the SolidWorks
frame `placements.json` assumes). `reference/<name>.step` keeps the SolidWorks re-export of every
COTS part as the frame/size reference; `test_cots_vendor_matches_reference_frame` (bbox within
1.5 mm) and `test_cots_envelope_tracks_reference_bbox` guard vendor swaps. Swap procedure and
what has been tried: `vendor/README.md` (`./cadtool parts …`, step.parts).

## Shared dimensions (DRY)
`lib/params.py` is the single source of truth: mm and grams, every constant tagged
`[MEASURE] / [DATASHEET] / [DESIGN] / [REFERENCE] / [ESTIMATE]` with a derivation comment.
`lib/` never imports `parts/`. Docs name constants, never numbers. Datum: the SolidWorks capture
frame is **Y up** (J1 axis); the URDF base frame (REP-103) lives in `robot/frames.py`.

Changing a shared dimension — touchpoints in order:

| # | Edit | What |
|---|---|---|
| 1 | `lib/params.py` | the value (keep tag + derivation) |
| 2 | `tests/test_params_invariants.py` | the lock; `./cadtool pytest -m "not slow"` |
| 3 | `./cadtool gen parts/<affected>.py` | regenerate the committed STEP(s) |
| 4 | `./cadtool pytest` + `./cadtool gen assemblies/arm.py` + snapshot | verify geometry and fit |

## Assembly & placements
- `reference/placements.json` (from `tools/extract_placements.py`) holds every occurrence:
  `rel` (to its parent node) and `world`, as `Location(position, rotation_xyz_deg)`; keys
  `"<part>#<n>"`, module `"gripper#1"`. Treat it as an immutable input.
- `assemblies/arm.py` / `gripper.py`: `OCCURRENCES = [(part, role|None, key), …]` in SolidWorks
  document order; `assemblies/_occurrences.py` places a **fresh** `gen_step()` copy per occurrence
  with `.moved(rel * LOCAL_FROM_REF⁻¹)` and locates the gripper module **in place** (`.locate`).
  Roles (`j2`/`j3`, `1`/`2`) only disambiguate duplicates; rename when joint semantics arrive.
- When adding source-level joints, use `cadgen.assembly.AssemblyHelper` frames/mates, keep
  placements parameter-driven, and validate with `inspect align/measure/frame`.

## Robot description (`robot/`)
- `robot/arm.urdf` is the **source of truth** (hand-authored XML, ledger comment on top);
  `arm.srdf` pairs by colocation + robot name; `arm.sdf` is derived from the URDF. Never build a
  Python generator for them - `tools/robot_frames.py --urdf-draft/--sdf-draft` only prints scaffolding
  to copy numbers from, and `--check` (run by `tests/test_robot.py`) catches drift.
- `robot/frames.py` is the kinematic SSOT: `LINKS` (occurrence keys per rigid link) and `JOINTS`
  (axis point/direction in the SolidWorks capture frame, limits from `lib/params.py`). Joint frame:
  Z on the axis, X along the child link; child link frame = joint frame at capture, so **all joints
  are 0 at the capture pose** and mesh origins are identity. Moving an occurrence between links or
  changing an axis = edit `frames.py`, re-export meshes, re-derive the affected numbers, re-check.
- Meshes: `tools/export_link_meshes.py` (mm STL per link, `scale 0.001` in the XML); never hand-edit.
  Inertials come from OCP `BRepGProp` (printed parts at `PETG_DENSITY`, COTS at `MASS_G`).
- Validate with `./cadtool validate <file> --strict` and snapshot with
  `./cadtool skill urdf snapshot --input robot/arm.urdf --output snapshots/x.png` after every edit;
  hand `.urdf` files to the viewer (`?file=robot/arm.urdf`).
- Placeholders to confirm before real use: joint limits/effort/velocity (`lib/params.py`), axis signs,
  jaw travel, the link-membership assumptions listed in the URDF ledger. The cycloidal drive (J1
  actuator) is not modelled; wrist_roll and the jaws are not driven by `src/config.py`.

## Tests (`./cadtool pytest`)
`test_parts_convention.py` (contract + geometry for every part, COTS envelopes + vendor frames),
`test_reference_match.py` (manifest checksums; converted parts vs reference), `test_placements.py`
(JSON integrity, tables cover every key once), `test_assembly.py` (34 leaves / 50 solids /
volume / bbox vs SolidWorks), `test_params_invariants.py` (locks), `test_robot.py` (link partition, frames, FK at zero = capture,
meshes, inertials, URDF/SRDF/SDF consistency + plugin validators). Geometry tests are `slow`.

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
