# CAD — build123d models (robotic arm)

**Purpose:** CAD-scoped agent guide — the `gen_step()` part convention, the wrapper → parametric
conversion workflow, shared-dimension rules, assembly placements, tests, tooling.
**Audience:** agent. Human docs: `README.md`. Reference provenance: `reference/README.md`.

`cad/` is a **separate uv project** (Python 3.12, build123d 0.10) inside the robotic-arm repo;
the root motor-control project never depends on it.

## Running things (always via `./cadtool` or `uv run`, from `cad/`)
- **Never call bare `python`** — the system Python is 3.14 without build123d. Python is pinned
  to **3.12** (vtk wheels) — don't bump it. Never run CAD code with the root repo's venv.
- `./cadtool step parts/<name>.py` — generate the **committed** `parts/<name>.step`
  (+ hidden `.<name>.step.glb`); `--stl exports/<name>.stl` for a printable sidecar.
- `./cadtool step assemblies/arm.py` — `assemblies/arm.step` (git-ignored).
- `./cadtool inspect refs <file.step> --facts --planes --positioning` (+ `measure|align|frame|diff`).
- `./cadtool snapshot --input assemblies/arm.step --output snapshots/arm.png --size-profile assembly --view-labels`.
- `./cadtool viewer` — CAD Viewer for this folder (URL printed; add `&file=<rel path>`).
- `./cadtool pytest [-m "not slow"]`, `./cadtool python -m assemblies.arm`.
- `uv add <pkg>` for deps (commit `pyproject.toml` + `uv.lock`); never `pip install`.
- `cadtool` puts the plugin's `cadpy` on `PYTHONPATH` (zero-install; survives `uv sync`) and
  runs `uv run --frozen`. `lib/assembly.py` falls back to a tiny `AssemblyHelper` when cadpy is
  absent, so plain `uv run pytest` also works — joints/mates need the real cadpy (`./cadtool`).

## Authoring a part (`parts/<name>.py`)
Every part MUST (enforced by `tests/test_parts_convention.py`):
- define a module-level **`gen_step()`** that **returns** a valid, labelled Part/Compound at its
  **local origin** — the assembly owns placement; label == module name;
- have **no import side effects** (`show()` only under `if __name__ == "__main__":`);
- keep the 2-line path shim (`sys.path.insert(0, parent.parent)`) above the build123d imports;
- pull shared dims from `lib/params.py`.

**Part states.** Custom parts declare `REFERENCE = NAME`, `CONVERTED` and `LOCAL_FROM_REF`:
- *wrapper* (`CONVERTED = False`, from `_wrapper_template.py`): `gen_step()` returns
  `reference/<name>.step` in the SolidWorks part-file frame — the day-one state of all 20 custom parts;
- *parametric* (`CONVERTED = True`, from `_template.py`): real build123d. To convert: rewrite
  `gen_step()`, set `CONVERTED = True`, optionally set `LOCAL_FROM_REF` (reference frame → new
  local frame; the assemblies compose `placement * LOCAL_FROM_REF⁻¹`, so `placements.json` never
  changes), then `./cadtool pytest tests/test_reference_match.py -k <name>` (volume ±0.5 %,
  bbox ±0.2 mm; per-part `REF_VOL_TOL` / `REF_BBOX_TOL`) and `./cadtool step parts/<name>.py`.
- Multi-body parts are registered in `MULTI_BODY` in `tests/test_parts_convention.py`.

## Purchased (COTS) parts (`parts/_cots_template.py`)
`COTS = True`, `MASS_G` (datasheet grams), `VENDOR_STEP = vendor/<name>.step`; `gen_step()` is
hybrid (vendor STEP if present, else `_envelope()` from `lib.params`, both in the SolidWorks
frame `placements.json` assumes — `test_cots_envelope_tracks_reference_bbox` checks the envelope
occupies the vendor bbox). Vendor STEPs are committed (`!/cad/vendor/*.step`); fetch better ones
with `/cad:step-parts`, re-orient inside `gen_step()` if needed.

## Shared dimensions (DRY)
`lib/params.py` is the single source of truth: mm and grams, every constant tagged
`[MEASURE] / [DATASHEET] / [DESIGN] / [REFERENCE] / [ESTIMATE]` with a derivation comment.
`lib/` never imports `parts/`. Docs name constants, never numbers.

Changing a shared dimension — touchpoints in order:

| # | Edit | What |
|---|---|---|
| 1 | `lib/params.py` | the value (keep tag + derivation) |
| 2 | `tests/test_params_invariants.py` | the lock; `./cadtool pytest -m "not slow"` |
| 3 | `./cadtool step parts/<affected>.py` | regenerate the committed STEP(s) |
| 4 | `./cadtool pytest` + `./cadtool step assemblies/arm.py` + snapshot | verify geometry and fit |

## Assembly & placements
- `reference/placements.json` (from `tools/extract_placements.py`) holds every occurrence:
  `rel` (to its parent node) and `world`, as `Location(position, rotation_xyz_deg)`; keys
  `"<part>#<n>"`, module `"gripper#1"`. Treat it as an immutable input.
- `assemblies/arm.py` / `gripper.py`: `OCCURRENCES = [(part, role|None, key), …]` in SolidWorks
  document order; `assemblies/_occurrences.py` places a **fresh** `gen_step()` copy per occurrence
  with `.moved(rel * LOCAL_FROM_REF⁻¹)` and locates the gripper module **in place** (`.locate`).
  Roles (`j2`/`j3`, `1`/`2`) only disambiguate duplicates; rename when joint semantics arrive.
- When adding source-level joints, use `cadpy.assembly.AssemblyHelper` frames/mates (run via
  `./cadtool`), keep placements parameter-driven, and validate with `inspect align/measure/frame`.

## Tests (`./cadtool pytest`)
`test_parts_convention.py` (contract + geometry for every part, COTS envelopes),
`test_reference_match.py` (manifest checksums; converted parts vs reference), `test_placements.py`
(JSON integrity, tables cover every key once), `test_assembly.py` (34 leaves / 50 solids /
volume / bbox vs SolidWorks), `test_params_invariants.py` (locks). Geometry tests are `slow`.

## Gotchas (all verified)
- `Compound.volume` skips nested sub-assemblies in build123d 0.10 — use `lib.reference.solid_volume()`.
- `is_valid` is a **property**; `Location.to_tuple()` is deprecated (`tuple(loc)` → two Vectors).
- `.moved()`/`.located()` deep-copy the shape **and its parent chain**; never call them on a child
  of a big imported assembly (copy its `solids()` instead). A shape added to two Compounds is
  silently re-parented — build a fresh `gen_step()` per occurrence.
- `import_step()` converts inch-unit files to mm and keeps the assembly hierarchy (labels mangle
  ` .()` → `_`; `lib.reference.clean_label` mirrors it). `j1_cap`, `j2_cap_1`, `j2_cap_2` have
  geometry far from their part origin — `placements.json` compensates; use `LOCAL_FROM_REF` when converting.
- `uv sync` prunes anything `uv pip install`-ed; `uv run` doesn't. Don't install cadpy — `cadtool` handles it.
- Don't compare large STEP/GLB artifacts with `git diff`; compare source, `inspect` output and snapshots.
