# CAD — build123d models (robotic arm)

**Purpose:** CAD-scoped agent guide — the `@step` model convention (cadgen 0.5), the wrapper → parametric
conversion workflow, shared-dimension rules, assembly placements, purchased parts, tests, tooling.
**Audience:** agent. Human docs: `README.md`. Reference provenance: `reference/README.md`. The
cycloidal drive (spec, port notes, attachment): `docs/cycloidal_drive.md`.
**Last updated:** 2026-09-12. Every commit that changes behaviour, layout or tooling gets a dated entry in the
root `CHANGELOG.md` and bumps the `Last updated` line of the docs it touches.

`cad/` is a **separate uv project** (Python 3.12, build123d 0.11, OCP 7.9, cadgen 0.5.x) inside the
robotic-arm repo; the root motor-control project never depends on it.

## Running things (always via `./cadtool` or `uv run`, from `cad/`)
- **Never call bare `python`** — the system Python is 3.14 without build123d. Python is pinned
  to **3.12** (`.python-version`; bump only with the full suite green). Never run CAD code with the root repo's venv.
- Toolchain: the `cadgen` PyPI package (`cadgen[snapshot]==<ver>`, a locked dependency) is the whole
  runtime — decorators, the `cadgen` CLI, viewer, snapshots. The `cad@text-to-cad` plugin **v0.5.x**
  (`~/.claude/plugins/cache/text-to-cad/cad/<ver>/skills/`) ships only the `/cad:*` skill docs (+ the
  step.parts script); its `skills/cad/requirements.txt` pins the same cadgen version — bump both
  together, `./cadtool doctor` checks (plugin updates need `git-lfs` on `PATH`).
- **A model is a script you run.** `./cadtool gen parts/<group>/<name>.py` (alias `step`) runs it
  and writes the **committed** sibling `parts/<group>/<name>.step`; a second run prints `current …`
  (freshness gate: source closure + tracked inputs + outputs hashed); `./cadtool why <model.py>`
  explains a verdict clause by clause; `--force` rebuilds. Every derived artefact lives in the
  content-addressed store `~/.cache/cadgen` (`./cadtool store gc`) — nothing in the tree.
- `./cadtool gen assemblies/arm.py` — `assemblies/arm.step` (git-ignored). The arm **calls its child
  models**: every stale part is rebuilt in parallel and its committed STEP rewritten; the gripper, the
  drive and the robot links link their children's trees, the arm inlines tinted copies (see Assembly).
  Pull semantics: a rebuilt part does not update the arm until the arm is rebuilt (`why` shows the
  pinned child).
- `./cadtool show <model.py>` — OCP CAD Viewer (VS Code) preview of the model BODY: no build, nothing
  written. `./cadtool python -c "from assemblies.cycloidal_drive import totals; print(totals())"`.
- `./cadtool export <file.step> stl|3mf|glb [out]` — one mesh file per call from a document
  (`node` ≥ 20 on `PATH`; relative `--mesh-tolerance`, default 1.5e-3 of the bounding diagonal).
- `./cadtool inspect refs <file.step> --facts --planes --positioning` (+ `measure|align|frame|diff|interfere|validate`).
- `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly --view-labels`
  (`--job job.json` for a multi-view packet; the path you name is the file you get - no timestamp).
  Snapshot review is mandatory after visible geometry changes.
- `./cadtool viewer` — CAD Viewer serving `cad/`: `http://127.0.0.1:3245/?file=<rel path>` (ships in
  cadgen, no Node; 12 h auto-stop; `./cadtool cadgen viewer list|stop --port N`). Hand every
  created/updated STEP to it.
- `./cadtool daemon stop` — stop cadgen's warm build daemon and its workers (they only reload code when
  restarted; `./cadtool cadgen daemon status` shows them).
- `./cadtool validate <file.urdf|.srdf|.sdf> [--strict]`, `./cadtool parts "<query>"`,
  `./cadtool skill <skill> <tool> …`, `./cadtool cadgen <anything>`.
- `./cadtool pytest [-m "not slow"]`.
- `uv add <pkg>` for deps (commit `pyproject.toml` + `uv.lock`); never `pip install`.
- Every committed STEP/STL under `cad/` is a **Git LFS** object (`/.gitattributes`): a checkout
  showing ~130-byte pointer files (checksum tests, `read_step`/`import_step` and cadgen fail on them)
  needs `git lfs pull`; every regenerated STEP is a new LFS object (cadgen writes deterministic bytes
  per kernel: an unchanged model rewrites an identical file, but the STEP header names the OCCT
  version, so a kernel bump rewrites every file once — stop the daemon first, see Gotchas).
- Cycloidal references: `cd ../cycloidal_drive && uv run python ../robotic-arm/cad/tools/cycloidal/export_cadquery.py`
  (the OLD repo's CadQuery venv — never ours), then `./cadtool python tools/cycloidal/import_reference.py`.
- Expected noise: every `gen` prints cadgen's "kernel imported eagerly" hint on stderr (the model files
  import build123d at module top) and pays the ~2.5 s import — accepted for now.
- **build123d 0.11.1 / OCP 7.9.3 are required by cadgen 0.5.x** (`pyproject.toml` pins `cadquery-ocp`
  to build123d's `cadquery-ocp-novtk` release — keep them equal). On 0.10 / 7.8.1 cadgen could not
  build 11 parts (OCCT 7.8.1 mis-read BinTools VERSION_4 component objects), could not export a linked
  child (`LazyCompound` needs 0.11's `wrapped` property) and its STEP writer needs `HArray1.Value`.
  `test_part_survives_cadgen_component_round_trip` locks the first.

## Authoring a part (`parts/<group>/<name>.py`)
Parts live in subsystem groups (`base`, `joints`, `wrist`, `gripper`, `cycloidal`; templates in
`parts/_templates/`). The group is only a directory: the part's NAME is its module stem, unique
across groups (`import parts` raises on a duplicate), and it keys the manifest,
`reference/<origin>/<name>.step`, `placements.json` and the URDF links. `parts.names()` /
`parts.load(name)` / `parts.model(name)` / `parts.build(name)` (a stdlib-only directory scan in
`parts/__init__.py`) are the only way code reaches a part — never `from parts import <name>`; note
`parts.base` is the *group package*, the part is `parts.load("base")`. Every part MUST (enforced by
`tests/test_parts_convention.py`):
- declare ONE model, **`@step def <name>()`** (`from cadgen import step`; NAME = file stem = model
  name; no parameters, no `out=` — the STEP is the sibling file), that **returns** a valid, labelled
  Part/Compound at its **local origin** — the assembly owns placement; label == module name;
- end with `if __name__ == "__main__": <name>()` — that call IS the build (`./cadtool gen` runs the
  file); no `show()` in the file, no import side effects;
- import `lib` / `parts` plainly — **no `sys.path` shim**: `cadtool` exports `PYTHONPATH=cad/`, pytest
  has `pythonpath = ["."]`, `.env` covers VS Code. cadgen loads `parts/<group>/<name>.py` as the package
  module `parts.<group>.<name>` (it walks the `__init__.py` chain) — the same object `parts.load()`
  returns. **Never add `cad/__init__.py`** (the package root would become the repo root);
- pull shared dims from `lib/params.py`.

**Never call a model to get its geometry outside a build**: with no cadgen build on the thread the
call runs the whole pipeline (gate, writes the STEP, talks to the warm daemon). `parts.build(name)` /
`lib.models.raw(model)` run the body in-process (tests, tools, previews); `lib.models.geometry(model)`
is what assembly bodies call for a child — the linked child while a build runs, the raw body
otherwise. `tests/conftest.py` makes an accidental top-level call under pytest fail loudly.

**Part states.** Custom parts declare `REFERENCE = NAME`, `CONVERTED` and `LOCAL_FROM_REF`:
- *wrapper* (`CONVERTED = False`, from `_templates/wrapper.py`): the model returns
  `reference/solidworks/<name>.step` (via `lib.reference.load` → `cadgen.read_step`, a tracked input)
  in the SolidWorks part-file frame — the day-one state of all 20 custom parts;
- *parametric* (`CONVERTED = True`, from `_templates/designed.py`): real build123d. To convert: rewrite
  the model body, set `CONVERTED = True`, optionally set `LOCAL_FROM_REF` (reference frame → new
  local frame; the assemblies compose `placement * LOCAL_FROM_REF⁻¹`, so `placements.json` never
  changes), then `./cadtool pytest tests/test_reference_match.py -k <name>` (volume ±0.5 %,
  bbox ±0.2 mm; per-part `REF_VOL_TOL` / `REF_BBOX_TOL`) and `./cadtool gen parts/<group>/<name>.py`.
- *designed* (`CONVERTED = True`, `REFERENCE = NAME`, registered in `lib/reference.py DESIGNED`):
  the cycloidal drive's printed parts — the reference is the CadQuery export they were ported
  from; `tests/cycloidal/test_port.py` additionally demands identical face sets / tessellations.
  Their geometry helpers live in `lib/cycloidal/` and each module exposes `build(cfg)` for tests.
- Multi-body parts are registered in `MULTI_BODY` in `tests/test_parts_convention.py`.
- Keep the importable `parts/<group>/<name>.py` naming (tests/assemblies reach them through
  `parts.load` / `parts.model`); upstream's `src/` + `STEP/` layout is deliberately not used. A part and
  its STEP stay siblings (cadgen's default `out`; the viewer pairs them into one entry).

## Purchased (COTS) parts (`parts/_templates/cots.py`)
`COTS = True`, `MASS_G` (datasheet grams), `VENDOR_STEP = vendor/<name>.step`, `VENDOR_TO_REF`
(vendor-file frame → SolidWorks frame; identity for the SolidWorks re-exports); the model is
hybrid (`cadgen.read_step(VENDOR_STEP)` if present — a tracked input, so a swapped vendor file makes
the part stale — else `_envelope()` from `lib.params`, both in the SolidWorks frame `placements.json`
assumes). `reference/solidworks/<name>.step` keeps the SolidWorks re-export of every COTS part as the
frame/size reference (for the drive's purchased parts: the CadQuery export of their simplified model
in `reference/cycloidal/`, `lib/reference.py CYCLOIDAL_COTS`; `path_of()` resolves the origin);
`test_cots_vendor_matches_reference_frame` (bbox within 1.5 mm, skipped when there is no vendor
file - the envelope is then the geometry) and `test_cots_envelope_tracks_reference_bbox` guard vendor
swaps. Swap procedure and what has been tried: `vendor/README.md` (`./cadtool parts …`, step.parts).

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
| 3 | `./cadtool gen parts/<group>/<affected>.py` | regenerate the committed STEP(s) (or just the arm: it rebuilds every stale part) |
| 4 | `./cadtool pytest` + `./cadtool gen assemblies/arm.py` + snapshot | verify geometry and fit |

## Assembly & placements
- `reference/placements.json` (from `tools/reference/extract_placements.py`) holds every occurrence:
  `rel` (to its parent node) and `world`, as `Location(position, rotation_xyz_deg)`; keys
  `"<part>#<n>"`, module `"gripper#1"`. Treat it as an immutable input.
- `assemblies/arm.py` / `gripper.py`: `OCCURRENCES = [(part, role|None, key), …]` in SolidWorks
  document order; `assemblies/_occurrences.py` places each occurrence as
  `lib.models.geometry(parts.model(part)).moved(rel * LOCAL_FROM_REF⁻¹)` — inside a build the linked
  child (built in parallel; the gripper's STEP links the part's tree), otherwise a fresh in-process
  copy — and `.moved()`s the modules the same way (`MODULES` maps a module name to its **model**;
  `.locate()` in place is gone: it would force and mutate a linked child). The tinted arm uses
  `geometry(model, inline=True)`: the child is still called (pinned, rebuilt, its STEP rewritten) but
  the arm owns a recoloured copy — cadgen's packager keeps a linked child's own colours, so tints
  through links are lost (verified by snapshot).
  Roles (`j2`/`j3`, `1`/`2`) only disambiguate duplicates; rename when joint semantics arrive.
- `arm.py GROUPS` buckets the occurrences into the component tree
  `arm → base_link/link1/link2/link3/wrist` — the `robot/frames.py LINKS` partition with the
  gripper module kept whole (`wrist` = wrist_roll_link + jaw links) — via
  `_occurrences.add_grouped_occurrences()`, which tints each subtree with its group's color
  (`MODULE_TINTS` overrides for the two modules) and raises unless the groups cover the keys
  exactly once. `test_assembly.py` locks the group labels + the LINKS mirror. The tints are
  per-leaf (a compound-level color doesn't cascade in ocp_tessellate) — hence the inline copies above.
- `assemblies/cycloidal_drive.py` is **code-driven**: rows are `(part, role, Location)` from
  `lib/cycloidal stack_positions` (`add_located`); its placement key `cycloidal_drive#1` is a
  `designed` module record in `placements.json` (pose from the SolidWorks node, no leaf records,
  `solidworks` cross-check block; `tools/reference/extract_placements.py` never descends into
  `DESIGNED_MODULES`). `world_rows(key)` expands a designed-module key into world-placed parts for
  links and inertials. Keep `EXPECTED` in step with the geometry (`totals()`).
- When adding source-level joints, use `cadgen.assembly.AssemblyHelper` frames/mates (persisted
  motion is `@step(kinematics=…)` — viewer sliders, posed snapshots), keep placements
  parameter-driven, and validate with `inspect align/measure/frame`.

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
- `robot/links/<link>.py` are `@step` models of each rigid link (`robot/_links.build_link`, in the link
  frame; `robot/links/<link>.step` git-ignored). Meshes: `tools/robot/export_link_meshes.py` (mm STL per
  link via build123d's `export_stl`, `scale 0.001` in the XML); never hand-edit.
  Inertials come from OCP `BRepGProp` (printed parts at `PETG_DENSITY`, COTS at `MASS_G`).
- Validate with `./cadtool validate <file> --strict` and snapshot with
  `./cadtool snapshot robot/arm.urdf snapshots/x.png` after every edit; hand `.urdf` files to the
  viewer (`?file=robot/arm.urdf`).
- Placeholders to confirm before real use: joint limits/effort/velocity (`lib/params.py`), axis signs,
  jaw travel, the link-membership assumptions listed in the URDF ledger. The cycloidal drive is
  physically the shoulder-pitch joint between `j1_coupler` and `j1_link` but is NOT a joint yet: its
  module key rides in `LINKS["link1"]`; which MKS motor drives which joint is unconfirmed;
  wrist_roll and the jaws are not driven by `src/config.py`.

## Tests (`./cadtool pytest`)
`tests/conftest.py` sets `CADGEN_DAEMON=0` and blocks top-level model builds (tests call bodies:
`parts.build(name)`, `lib.models.raw(model)`). `test_parts_convention.py` (contract + geometry for
every part, COTS envelopes + vendor frames), `test_reference_match.py` (manifest checksums; converted
parts vs reference), `test_placements.py` (JSON integrity, tables cover every key once, the designed
module record), `test_assembly.py` (34 + 18 leaves / 50 + 58 solids / volume / bbox vs SolidWorks + the
module lock), `test_params_invariants.py` (locks), `test_robot.py` (link partition, frames, FK at
zero = capture, meshes, inertials, URDF/SRDF/SDF consistency + cadgen's validators via
`./cadtool validate`), `tests/cycloidal/` (the drive: one module per part + housing / purchased /
fitment / assembly / port, ~230 tests, `from tests.cycloidal.helpers import …`). Geometry tests are
`slow`. Run pytest only through `./cadtool pytest` (rootdir `cad/`; the repo-root `tests/` is the
unrelated, broken motor-control suite).

## Gotchas (all verified)
- `Compound.volume` skips nested sub-assemblies (build123d 0.10 and 0.11) — use `lib.reference.solid_volume()`.
- `Shape.intersect` on composite operands changed in build123d 0.11 (a placed module against a part reported
  whole solids as common) — `tests/cycloidal/helpers.interference` runs the kernel's `BRepAlgoAPI_Common` directly.
- `is_valid` is a **property**; `Location.to_tuple()` is deprecated (`tuple(loc)` → two Vectors).
- `.moved()`/`.located()` deep-copy the shape **and its parent chain**; never call them on a child
  of a big imported assembly (copy its `solids()` instead). A shape added to two Compounds is
  silently re-parented — `_occurrences` builds a fresh body per occurrence outside a build (inside
  one, each call to a child model is its own linked occurrence).
- `read_step()` / `import_step()` convert inch-unit files to mm and keep the assembly hierarchy (labels
  mangle ` .()` → `_`; `lib.reference.clean_label` mirrors build123d's `import_step`, which the
  reference tools keep using). `j1_cap`, `j2_cap_1`, `j2_cap_2` have geometry far from their part
  origin — `placements.json` compensates; use `LOCAL_FROM_REF` when converting.
- `uv sync` prunes anything `uv pip install`-ed; `uv run` doesn't — that's why `cadgen` is a
  pyproject dependency, not a manual install.
- Don't compare large STEP artifacts with `git diff`; compare source, `inspect` output and snapshots.
  A STEP edited by anything but its model (or built under another `CADGEN_CACHE_DIR`) reads as stale
  in `./cadtool why` — rebuild it.
- New STEPs in a group directory are committed thanks to the recursive `!/cad/parts/**/*.step`
  re-admit in the root `.gitignore` (`git check-ignore -v --no-index cad/parts/<group>/<name>.step`
  from the repo root must print that line). Nothing else lands in the tree: no `__cadgen__/`, no
  sidecar unless a model declares `kinematics=` (then `<name>.step.json`, committed beside it).
- After `uv sync` changes cadgen / build123d / OCP, `./cadtool daemon stop`: the warm daemon's workers
  keep the old code loaded (its identity token only tracks cadgen's version and file mtimes; cadgen
  has no stop verb of its own and the daemon shrugs off a bare SIGTERM). The next `gen` starts a fresh one.
- A model run accepts only `--force --mesh-tolerance --mesh-angular-tolerance --verbose --json`;
  anything else (`--totals`, a preview flag) is an argparse error — use `./cadtool show` / `python -c`.
- Cycloidal discs: chamfer the lobe edges BEFORE cutting holes (the end face must carry only the
  spline edge); the profile is a periodic *interpolating* spline (`Edge.make_spline(periodic=True)`,
  never `make_spline_approx`); OCCT's analytic volume is ~0.3 % off on that face (both ours and the
  reference) - compare tessellations/face sets, not `.volume`. A boolean between the two discs takes
  minutes: probe points instead.
