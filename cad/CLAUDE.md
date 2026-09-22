# CAD — build123d models (robotic arm)

**Purpose:** CAD-scoped agent guide — the `@step` model convention (cadgen 0.6), the wrapper → parametric
conversion workflow, shared-dimension rules, assembly placements, purchased parts, tests, tooling.
**Audience:** agent. Human docs: `README.md`. Reference provenance: `reference/README.md`. The
cycloidal drive (spec, port notes, attachment): `docs/cycloidal_drive.md`.
**Last updated:** 2026-09-21 (motor mounts). Every commit that changes behaviour, layout or tooling gets a dated entry in the
root `CHANGELOG.md` and bumps the `Last updated` line of the docs it touches.

`cad/` is a **separate uv project** (Python 3.12, build123d 0.11, OCP 7.9, cadgen 0.6.x) inside the
robotic-arm repo; the root motor-control project never depends on it.

## Running things (always via `./cadtool` or `uv run`, from `cad/`)
- **Never call bare `python`** — the system Python is 3.14 without build123d. Python is pinned
  to **3.12** (`.python-version`; bump only with the full suite green). Never run CAD code with the root repo's venv.
- Toolchain: the `cadgen` PyPI package (`cadgen[snapshot]==<ver>`, a locked dependency) is the whole
  runtime — decorators, the `cadgen` CLI, viewer, snapshots. The `cad@text-to-cad` plugin **v0.6.x**
  (`~/.claude/plugins/cache/text-to-cad/cad/<ver>/skills/`) ships only the `/cad:*` skill docs (+ the
  step.parts script); its `skills/cad/requirements.txt` pins the same cadgen version — bump both
  together, `./cadtool doctor` checks (plugin updates need `git-lfs` on `PATH`).
- **A model is a script you run.** `./cadtool gen parts/<group>/<name>.py` (alias `step`) runs it
  and writes the sibling `parts/<group>/<name>.step` (git-ignored); a second run prints `current …`
  (freshness gate: source closure + tracked inputs + outputs hashed); `./cadtool why <model.py>`
  explains a verdict clause by clause; `--force` rebuilds. Every derived artefact lives in the
  content-addressed store `~/.cache/cadgen` (`./cadtool store gc`) — nothing in the tree.
- `./cadtool gen assemblies/arm.py` — `assemblies/arm.step` (git-ignored). The arm **calls its child
  models**: every stale part is rebuilt in parallel and its STEP rewritten; the gripper, the
  drive and the robot links link their children's trees, the arm inlines tinted copies (see Assembly).
  Pull semantics: a rebuilt part does not update the arm until the arm is rebuilt (`why` shows the
  pinned child).
- `./cadtool show <model.py>` — OCP CAD Viewer (VS Code) preview of the model BODY: no build, nothing
  written. `./cadtool python -c "from assemblies.cycloidal_drive import totals; print(totals())"`.
- **Printed vs. bought** — one label per part (`COTS = True` = bought, else printed; `parts.bought(name)`), everything
  else generated from it: `./cadtool python tools/bom.py [--module cycloidal_drive|gripper] [--md|--json]` (print
  list + buy list from the assembly tables, kernel-free; its `EXTRAS` = purchased items with no geometry),
  `./cadtool python tools/export_printables.py [--parts …]` (`print/<name>.stl` per printed part, git-ignored, mm,
  part-local frame), and the grey of purchased parts in `arm.step`, `gripper.step` and `cycloidal_drive.step` (see Assembly).
- `./cadtool export <file.step> stl|3mf|glb [out]` — one mesh file per call from a document
  (`node` ≥ 20 on `PATH`; relative `--mesh-tolerance`, default 1.5e-3 of the bounding diagonal).
- `./cadtool inspect <file.step> [--planes] [--json]` (leaf refs, solids, faces, volume, bbox; `--planes` = the
  planar faces as normal / offset / area) and `./cadtool inspect diff <a.step> <b.step> [--tol X]` (same geometry?
  exit 1 if not) — a **local** tool, `tools/step_facts.py`: cadgen 0.6.5 removed `cadgen step inspect` and ships no
  replacement command (the old `refs|measure|align|…` first argument exits 2 with the new syntax). Anything else
  is Python over `cadgen.read_scene(path)` (`.leaves()`, `.resolve("#o1.2.f7").shape()` — world-frame build123d
  geometry, the refs the viewer shows) + `cadgen.geometry` (`closest_points`, `overlap_volume`, `topology_errors`),
  kept as a test when it is worth re-running.
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
- **Generated STEPs are never committed** — part STEPs included (git-ignored since 2026-09-21, like
  `assemblies/*.step` and `robot/links/*.step`): a fresh clone has none until `./cadtool gen assemblies/arm.py`
  (~35 s cold). Committed are only the **inputs** — `reference/**/*.step`, `vendor/*.step` — and
  `robot/meshes/*.stl`, all **Git LFS** objects (`/.gitattributes`): a checkout showing ~130-byte pointer
  files (checksum tests, `read_step`/`import_step` and cadgen fail on them) needs `git lfs pull`.
  cadgen writes deterministic bytes per kernel **and per machine**: an unchanged model rewrites an identical
  file on the machine that built it, another machine writes the same geometry with other float noise
  (arm64 Mac vs the Fedora PC: 17 of 41 parts, ≤ 2e-10 mm apart), and the STEP header names the OCCT version.
  To prove a refactor changed no geometry: `shasum -a 256 parts/*/*.step > /tmp/before`, change, rebuild
  (daemon stopped first), `shasum -a 256 -c /tmp/before` on the SAME machine; a file may still come back with
  different numerical-zero terms (2026-09-18: `j1_link`, `gripper_clamp_bracket`, 12 values ≤ 1e-17 after a
  `lib/reference.py` edit, which makes every wrapper + COTS part stale) — settle those with
  `./cadtool inspect diff <old.step> <new.step>`.
- Cycloidal references: `cd ../cycloidal_drive && uv run python ../robotic-arm/cad/tools/cycloidal/export_cadquery.py`
  (the OLD repo's CadQuery venv — never ours), then `./cadtool python tools/cycloidal/import_cadquery.py`.
  SolidWorks references: `./cadtool python tools/reference/import_solidworks.py`. Each tool owns its own
  `reference/manifest.json` entries and keeps the other's; both build them with `lib/manifest.py`
  (`read()` / `write()` / `entry()` — no part imports it, so editing it never makes a part stale).
- **Model files never load the CAD kernel at import** (`tests/test_lazy_kernel.py`, fast lane): cadgen gates a
  model before paying for OCP, so an unchanged part re-runs in ~0.1 s (1.5 s with an eager import, which also
  prints cadgen's "kernel was imported before …" hint — that hint now means a regression). See Authoring.
- **build123d 0.11.1 / OCP 7.9.3**: cadgen 0.6.x requires `build123d>=0.11.1,<0.12` and
  `cadquery-ocp-novtk>=7.9,<8`; `pyproject.toml` pins the exact kernel (`cadquery-ocp-novtk==…`, the
  STEP bytes are per-kernel) and must never gain `cadquery-ocp`, the VTK build (see Gotchas). On 0.10 / 7.8.1 cadgen could not
  build 11 parts (OCCT 7.8.1 mis-read BinTools VERSION_4 component objects), could not export a linked
  child (`LazyCompound` needs 0.11's `wrapped` property) and its STEP writer needs `HArray1.Value`.
  `test_part_survives_cadgen_component_round_trip` locks the first.

## Two machines (Fedora Linux PC + arm64 Mac)
The repo is worked on from both; git is the only sync channel (push before leaving a machine, pull on arrival).
- **Per-machine state git does not carry** — after a pull that changes `pyproject.toml` / `uv.lock`:
  `./cadtool daemon stop && ./cadtool setup`, `claude plugin marketplace update text-to-cad && claude plugin
  update cad@text-to-cad` (each scope; restart Claude Code), then `./cadtool doctor` must be clean. No
  `CAD_PLUGIN` in a shell profile (it overrides the plugin detection).
- **No generated STEP is committed**, because its bytes differ per machine (see "Running things"): each
  machine builds its own, nothing to restore or avoid staging. Only `robot/meshes/*.stl` is generated AND
  committed — re-export those on one machine per change, never as a side effect.
- `cadtool` must stay runnable on macOS's stock **bash 3.2** (under `set -u` an empty array is unset: expand
  optional arrays as `${arr[@]+"${arr[@]}"}`) and without GNU coreutils on `PATH`. No exact-equality asserts on
  floating-point results (tessellations of spline geometry differ per architecture). Changes must work on both
  machines by construction; there is no per-machine sign-off to track or report.

## Authoring a part (`parts/<group>/<name>.py`)
Parts live in subsystem groups (`base`, `joints`, `wrist`, `gripper`, `cycloidal`; templates in
`parts/_templates/`). Groups follow the **physical stage along the arm**, not the name prefix and not
the URDF links (`j1_*` sit in `base/`; `gripper_clamp_bracket` / `gripper_j3_connector` sit in `wrist/`
as the wrist-side mount, the latter although `assemblies/gripper.py` places it) — put a new part with
the stage it bolts to. The group is only a directory: the part's NAME is its module stem, unique
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
  file; without it `gen` silently builds nothing — source-checked for every part and every
  `robot/links/<link>.py`); no `show()` in the file, no import side effects;
- import `lib` / `parts` plainly — **no `sys.path` shim**: `cadtool` exports `PYTHONPATH=cad/`, pytest
  has `pythonpath = ["."]`, `.env` covers VS Code. cadgen loads `parts/<group>/<name>.py` as the package
  module `parts.<group>.<name>` (it walks the `__init__.py` chain) — the same object `parts.load()`
  returns. **Never add `cad/__init__.py`** (the package root would become the repo root). The one
  `sys.path` line in the tree is `tools/cycloidal/export_cadquery.py` (it runs in the OTHER repo's venv);
- pull shared dims from `lib/params.py`;
- **keep the kernel lazy**: `from cadgen import build123d as bd` (a PEP 562 proxy — never
  `from build123d import …`, never `from cadgen.build123d import X`, both import it) and use `bd.<name>`
  **inside function bodies only**. No kernel object in a module-level constant, a class body, a decorator or
  an argument default (`align=bd.Align.MIN` as a default is eager — `lib/cycloidal/geom.align_min()` is a
  function for that reason); kernel types in annotations need `from __future__ import annotations`. The rule
  covers the model's whole import closure (`lib/`, `assemblies/_occurrences.py`, `robot/`). Frames a module
  declares are **data** — `(position mm, rotation_xyz_deg)`, `lib.datum.IDENTITY` for none — turned into a
  `Location` by `lib.datum.to_location()` inside a body; `lib.datum.base_frame()` and
  `assemblies.arm.arm_from_w()` are functions, the drive's `OCCURRENCES` rows carry positions.

**Layering** (locked by `tests/test_layering.py`, an AST scan — function-local imports count):
`lib ← parts ← assemblies ← robot ← tools ← tests`; a package imports only itself and the ones to its
left. So `assemblies/` never imports `robot/` (the base frame both need is `lib/datum.py`), `robot/`
builds on `assemblies/_occurrences.py` (`world_rows` reads a designed module's `OCCURRENCES` / `BODIES`),
and `lib/cycloidal/` never imports `lib/params.py` (which re-exports from it) — its globals come from
the leaf `lib/units.py` (`IN`, `NUDGE`; `lib/params.py` re-exports those too).

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
  local frame, as frame data `((x, y, z), (rx, ry, rz))` — `IDENTITY` until then; the assemblies
  compose `placement * to_location(LOCAL_FROM_REF)⁻¹`, so `placements.json` never
  changes), then `./cadtool pytest tests/test_reference_match.py -k <name>` (volume ±0.5 %,
  bbox ±0.2 mm; per-part `REF_VOL_TOL` / `REF_BBOX_TOL`) and `./cadtool gen parts/<group>/<name>.py`.
- *designed* (`CONVERTED = True`, `REFERENCE = NAME`, registered in `lib/reference.py DESIGNED`):
  the cycloidal drive's printed parts — the reference is the CadQuery export they were ported
  from; `tests/cycloidal/test_port.py` additionally demands identical face sets / tessellations (the two
  spline discs: the lobe profile within 1e-6 mm of the reference spline, `helpers.spline_deviation`, and the
  mesh within its chordal error — see "Two machines").
  Their geometry helpers live in `lib/cycloidal/` and each module exposes `build(cfg)` for tests
  (reached like any part: `parts.load(name).build(cfg)` — tests never import `parts.<group>` either).
- Multi-body parts are registered in `MULTI_BODY` in `tests/test_parts_convention.py`.
- Keep the importable `parts/<group>/<name>.py` naming (tests/assemblies reach them through
  `parts.load` / `parts.model`); upstream's `src/` + `STEP/` layout is deliberately not used. A part and
  its STEP stay siblings (cadgen's default `out`; the viewer pairs them into one entry).

## Purchased (COTS) parts (`parts/_templates/cots.py`)
`COTS = True`, `MASS_G` (datasheet grams), `PURCHASE_SPEC` (what to order) + `PURCHASE_QTY` (pieces per
occurrence — the whole pattern for a pin / fastener part, so it equals its `MULTI_BODY` count; the drive's are built
from `DEFAULT_CONFIG`, never retyped) + optional `PURCHASE_NOTE` — a printed part declares none of the three —,
`VENDOR_STEP = vendor/<name>.step`, `VENDOR_TO_REF`
(vendor-file frame → SolidWorks frame, as frame data; `IDENTITY` for the SolidWorks re-exports); the model is
hybrid (`cadgen.read_step(VENDOR_STEP)` if present — a tracked input, so a swapped vendor file makes
the part stale — else `_envelope()` from `lib.params`, both in the SolidWorks frame `placements.json`
assumes). `reference/solidworks/<name>.step` keeps the SolidWorks re-export of every COTS part as the
frame/size reference (for the drive's purchased parts: the CadQuery export of their simplified model
in `reference/cycloidal/`, `lib/reference.py CYCLOIDAL_COTS`; `path_of()` resolves the origin);
`test_cots_vendor_matches_reference_frame` (bbox within 1.5 mm, skipped when there is no vendor
file - the envelope is then the geometry) and `test_cots_envelope_tracks_reference_bbox` guard vendor
swaps. Swap procedure and what has been tried: `vendor/README.md` (`./cadtool parts …`, step.parts).
The third producer of vendor files is `tools/reference/split_mks_motor.py`: it splits the "NEMA 17 x 40 + MKS SERVO42D"
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
The two parts live in `parts/joints/` (`parts/cycloidal/` is locked to `CYCLOIDAL_COTS`); the drive motor's envelope
builder is `lib/cycloidal/motor.py nema17_motor()`, which the 40 mm envelope reuses with other `MotorParams`.

## Shared dimensions (DRY)
`lib/params.py` is the single source of truth: mm and grams, every constant tagged
`[MEASURE] / [DATASHEET] / [DESIGN] / [REFERENCE] / [ESTIMATE]` with a derivation comment.
`lib/` never imports `parts/`. Docs name constants, never numbers. Datum: the SolidWorks capture
frame is **Y up** (J1 axis); the URDF base frame (REP-103) is `lib/datum.py base_frame()` (with `frame()`,
`U`, `BASE_FORWARD`; `robot/frames.py` re-exports them and builds the kinematics on top) — and
`assemblies/arm.py` emits the arm in it (`arm_from_w()`, see Assembly), so `arm.step` is **Z up**.
The cycloidal drive's own dimensions are `lib/cycloidal/params.py` (`DriveConfig`, frozen
dataclasses, variants via `dataclasses.replace`); `lib/params.py` re-exports the interface values
(`CYCLOIDAL_*`, masses) from it - never retype a drive number.

Changing a shared dimension — touchpoints in order:

| # | Edit | What |
|---|---|---|
| 1 | `lib/params.py` | the value (keep tag + derivation) |
| 2 | `tests/test_params_invariants.py` | the lock; `./cadtool pytest -m "not slow"` |
| 3 | `./cadtool gen parts/<group>/<affected>.py` | regenerate the STEP(s) (or just the arm: it rebuilds every stale part) |
| 4 | `./cadtool pytest` + `./cadtool gen assemblies/arm.py` + snapshot | verify geometry and fit |

## Assembly & placements
- `reference/placements.json` (from `tools/reference/extract_placements.py`) holds every occurrence:
  `rel` (to its parent node) and `world`, as `Location(position, rotation_xyz_deg)`; keys
  `"<part>#<n>"`, module `"gripper#1"`. Treat it as an immutable input.
- **Mounted occurrences** (`lib/mounts.py`): the belt joints' motors - `nema17_40mm#1..3` + `mks_servo42d#1..3` on the
  NEMA 17 pads `base` / `j1_link` / `j2_link` carry - never existed in the SolidWorks capture. They are declared as
  frames-as-data in the host occurrence's frame (`Mount(key, part, host, link, joint, frame)`; a board's host is its
  motor) and `tools/reference/mount_placements.py` materialises them into `placements.json` as ordinary part records
  (parent `None`, `rel == world = host world * frame`, solids / volume / bbox from `parts.build`, a `mount` block; keys
  under top-level `mounted`, `P.keys(mounted=True)`), checking each motor's +Z against its joint axis. The extractor
  appends them on every run (`extract_placements.py --no-pancake` - the flag keeps this machine's bytes out of
  `vendor/nema17_pancake.step`); `mount_placements.py` alone is the merge mode that needs no monolith (change a spin or
  `J2_MOTOR_SLIDE_X` in `lib/params.py` → run it → re-derive the inertials). `assemblies/arm.py OCCURRENCES` /
  `GROUPS` and `robot/frames.py LINKS` list the keys like any other (roles = the joint names). `tests/test_mounts.py`
  re-checks the geometry: axis on the joint, mounting face on the host's pad, zero interference with the neighbours
  (the one budget: the Ø22 pilot in `j2_link`'s Ø20 slot). The drive's own board is a `cycloidal_drive.py` row
  (`stack_positions["z_mks_board"]`).
- `assemblies/arm.py` / `gripper.py`: `OCCURRENCES = [(part, role|None, key), …]` in SolidWorks
  document order; `assemblies/_occurrences.py` places each occurrence as
  `lib.models.geometry(parts.model(part)).moved(rel * LOCAL_FROM_REF⁻¹)` — inside a build the linked
  child (built in parallel; the gripper's STEP links the part's tree), otherwise a fresh in-process
  copy — and `.moved()`s the modules the same way (`MODULES` maps a module name to its **model**;
  `.locate()` in place is gone: it would force and mutate a linked child). The tinted arm uses
  `geometry(model, inline=True)`: the child is still called (pinned, rebuilt, its STEP rewritten) but
  the arm owns a recoloured copy — cadgen's packager keeps a linked child's own colours, so tints
  through links are lost (verified by snapshot).
- **The arm is emitted Z up.** `placements.json` is Y up, but cadgen's viewer and snapshot renderer
  hardcode +Z as up and have no up-axis option (no `@step` kwarg, sidecar field, URL parameter or
  flag), so a capture-frame `arm.step` renders lying on its side. `arm.py arm_from_w()` =
  `lib/datum.py base_frame()⁻¹` (capture frame → `base_link` frame: Z up, X forward, the base's
  mounting face on z = 0 — the frame `arm.urdf` uses, so both open in the same pose) goes into
  `grouped_children(…, root=)`, which composes `root * rel` into **every occurrence's
  placement**. Never `.moved()` the built root Compound instead: cadgen's STEP packager reads only the
  children's locations, so the in-process shape would rotate and the written STEP would not. The
  gripper and the drive keep their own module frames (both already have their axis on +Z), and
  `robot/` never reads the arm compound (it goes `placements.json world` → `world_rows`).
  `test_assembly.py` compares the SolidWorks world bbox through `arm_from_w()`.
- Roles (`j2`/`j3` = the elbow_pitch / wrist_pitch pulley + coupler pairs, `1`/`2`) only disambiguate
  duplicates; renaming them after the joints is a follow-up.
- `assemblies/arm_no_caps.py` is a **working view, not the robot**: `arm.py`'s `OCCURRENCES` / `GROUPS` minus
  `HIDDEN` (`j1_cap#1`, `j2_cap_1#1`, `j2_cap_2#1` — the covers over `j1_link` / `j2_link`), same frame, tree and
  tints, its own git-ignored `arm_no_caps.step` (56 leaves / 169 solids). `HIDDEN` is the one place to edit;
  never feed it to `robot/` or the SolidWorks totals. It is a second model because a model takes no
  parameters and the freshness gate sees no environment variable (an env switch in `arm()` would read
  `current` across a flip, and the daemon strips unlisted variables from its workers). Link edits land in
  the shared part file, so both arms pick them up on their **own** next `gen`; the caps are wrappers that do
  not follow a link change and this view cannot show a cap that no longer fits — after changing `j1_link` /
  `j2_link`, `./cadtool gen assemblies/arm.py` + snapshot too. For a one-off image no model is needed:
  `./cadtool snapshot assemblies/arm.step out.png --hide '#j1_cap' --hide '#j2_cap_1' --hide '#j2_cap_2'`
  (label refs; STEP input only, not with `--render` / `--focus`; the viewer has no `?hide=` parameter).
- **Printed vs. bought is a colour in every assembly, never a second model.** `gripper.py` and `cycloidal_drive.py`
  declare a `TINT` (their printed parts' colour, reused by `arm.py MODULE_TINTS`) and pass it to
  `occurrence_children(…, tint=)` / `located_children(…, tint=)`; `_tint_parts` gives every purchased part
  (`parts.bought()`) the one `_occurrences.BOUGHT_TINT` grey instead. Verified by snapshot 2026-09-21: a tint set on a
  module's DIRECT linked children does reach its STEP, so both modules stay linked (the arm's inline copies predate
  that finding and were left alone). `arm_make_buy` / `cycloidal_drive_make_buy` models existed for a commit or two
  (`f8b1c8d`) and were removed as duplicates — one STEP per assembly. A purchased item that is **not modelled** (the
  drive's 4 arm-mount bolts + 4 captive nuts, grease) lives only in `tools/bom.py EXTRAS` — it is on the buy list and
  absent from the model, the totals and the inertials; model it as a COTS pattern part (`cycloidal_housing_bolts` is
  the pattern) to change that.
- `arm.py GROUPS` buckets the occurrences into the component tree
  `arm → base_link/shoulder_link/upper_arm_link/forearm_link/wrist_pitch_link/wrist` — the
  `robot/frames.py LINKS` partition with the two modules kept whole (`wrist` = wrist_roll_link + jaw
  links; the cycloidal drive under `shoulder_link` although `LINKS` puts its rotor body in
  `upper_arm_link`) — via
  `_occurrences.grouped_children()`, which tints each subtree's **printed** parts with its group's color
  (`MODULE_TINTS` overrides for the two modules), every **purchased** part — inside the modules too — with the one
  `_occurrences.BOUGHT_TINT` grey (`_tint_parts`: grey always means bought, so no group / module tint may reuse it;
  `test_grey_means_bought`), and raises unless the groups cover the keys
  exactly once. `test_assembly.py` locks the group labels + the LINKS mirror. The tints are
  per-leaf (a compound-level color doesn't cascade in ocp_tessellate) — hence the inline copies above.
- `assemblies/cycloidal_drive.py` is **code-driven**: rows are `(part, role, position)` (data) from
  `lib/cycloidal stack_positions` (`located_children`); its placement key `cycloidal_drive#1` is a
  `designed` module record in `placements.json` (pose from the SolidWorks node, no leaf records,
  `solidworks` cross-check block; `tools/reference/extract_placements.py` never descends into
  `DESIGNED_MODULES`). `world_rows(key)` expands a designed-module key into world-placed parts for
  links and inertials; `BODIES` names the drive's rigid bodies (`stator` / `rotor`) and a `:<body>`
  key suffix (`"cycloidal_drive#1:rotor"`, `_occurrences.split_key`) selects one. Keep `EXPECTED`
  (whole module + `bodies`) in step with the geometry (`totals()` / `totals(body)`).
- **Assemblies are native build123d**: a model body returns `lib.assembly.assembly(name, children)` =
  `Compound(label=…, children=[…])`, and the `_occurrences` helpers (`occurrence_children`,
  `grouped_children`, `located_children`) RETURN the placed children, each labelled with cadgen's
  `label_shape` (`j3_coupler:j2`; `label_shape` / `label_text` are not deprecated). cadgen 0.6.5 deprecated
  its `AssemblyHelper` wrapper (a `FutureWarning` per build) — never reintroduce it; the 2026-09-18 migration
  rewrote all 11 assembly / link STEPs byte-identically.
- When adding source-level joints, declare them as data — `@step(kinematics={"mates": [cadgen.revolute(name,
  parent="#label", child="#label", …)]})` (viewer sliders, posed snapshots; the plugin's
  `skills/cad/references/kinematics.md`) — not as helper frames; keep placements
  parameter-driven, and validate with `cadgen.geometry.closest_points` / `overlap_volume` on
  `read_scene(…).resolve(ref).shape()` (the `inspect align/measure/frame` verbs went with cadgen 0.6.5).

## Robot description (`robot/`)
- `robot/arm.urdf` is the **source of truth** (hand-authored XML, ledger comment on top);
  `arm.srdf` pairs by colocation + robot name; `arm.sdf` is derived from the URDF. Never build a
  Python generator for them - `tools/robot/derive.py --urdf-draft/--sdf-draft` only prints scaffolding
  to copy numbers from, and `--check` (run by `tests/test_robot.py`) catches drift.
- `robot/frames.py` is the kinematic SSOT: `LINKS` (occurrence keys per rigid link; a designed-module
  key may carry a `:<body>` suffix — `cycloidal_drive#1:stator` / `:rotor` from
  `assemblies/cycloidal_drive.py BODIES`, expanded by `_occurrences.world_rows`) and `JOINTS` (axis
  point/direction in the SolidWorks capture frame, limits from `lib/params.py`). Chain: `base_link →
  base_yaw → shoulder_link → shoulder_pitch → upper_arm_link → elbow_pitch → forearm_link → wrist_pitch →
  wrist_pitch_link → wrist_roll → wrist_roll_link → jaw_a/jaw_b` (+ `tool0`). Joint frame:
  Z on the axis, X along the child link; child link frame = joint frame at capture, so **all joints
  are 0 at the capture pose** and mesh origins are identity. Moving an occurrence between links or
  changing an axis = edit `frames.py`, re-export meshes, re-derive the affected numbers, re-check.
- `robot/links/<link>.py` are `@step` models of each rigid link (`robot/_links.build_link`, in the link
  frame; `robot/links/<link>.step` git-ignored). Meshes: `tools/robot/export_link_meshes.py` (mm STL per
  link via build123d's `export_stl`, `scale 0.001` in the XML); never hand-edit.
  Inertials come from OCP `BRepGProp` (printed parts at `PETG_DENSITY`, COTS at `MASS_G`).
- Validate with `./cadtool validate <file> --strict` and snapshot with
  `./cadtool snapshot robot/arm.urdf snapshots/x.png` after every edit; hand `.urdf` files to the
  viewer (`?file=robot/arm.urdf`). (The 0.5.x renderers needed a local runtime patch; fixed upstream in
  0.6.0 and retired here.)
- Placeholders to confirm before real use: joint limits/effort/velocity (`lib/params.py`), axis signs,
  jaw travel, the link-membership assumptions listed in the URDF ledger. The cycloidal drive IS the
  `shoulder_pitch` joint (stator with the yawing `j1_coupler` in `shoulder_link`, rotor with `j1_link`
  in `upper_arm_link`); the base_yaw / elbow_pitch / wrist_pitch motors are the mounted `nema17_40mm#1..3` +
  `mks_servo42d#1..3` (`lib/mounts.py`, see Assembly); which CAN id (`src/config.py` J1..J3) drives which joint is unconfirmed;
  wrist_roll and the jaws are not driven by `src/config.py`.

## Tests (`./cadtool pytest`)
`tests/conftest.py` sets `CADGEN_DAEMON=0` and blocks top-level model builds (tests call bodies:
`parts.build(name)`, `lib.models.raw(model)`). `test_parts_convention.py` (contract + geometry for
every part, COTS envelopes + vendor frames), `test_reference_match.py` (manifest checksums; converted
parts vs reference), `test_placements.py` (JSON integrity, tables cover every key once, the designed
module record + the 6 mounted records vs `lib/mounts.py`), `test_assembly.py` (40 + 19 leaves / 95 + 77 solids / volume / bbox
vs SolidWorks + the module lock; `arm_no_caps` = the arm's tables minus `HIDDEN`, 56 leaves / 169 solids; the arm's leaf colours -
purchased = `BOUGHT_TINT`, which no group / module may reuse, printed = the link's / module's tint, in the arm and in
the standalone gripper and drive), `test_bom.py` (the print / buy lists
partition `parts.names()` by the flag, 34 + 25 occurrences, the drive's pieces follow `DEFAULT_CONFIG`, `EXTRAS`
well-formed), `test_params_invariants.py` (locks), `test_robot.py` (link partition, frames, FK at
zero = capture, meshes, inertials, URDF/SRDF/SDF consistency + cadgen's validators via
`./cadtool validate`), `test_tooling.py` (the installed cadgen and OCP kernel are the pinned ones, one complete OCP distribution,
`./cadtool inspect` agrees with the kernel),
`test_mounts.py` (the mounted motors: axis on the joint, face on the pad, board on the rear face, interference budget,
the base stack above the base bottom, the 90T planes within the shafts),
`test_layering.py` (the package layering, no `sys.path`, no direct part-module imports — AST scan),
`test_lazy_kernel.py` (a fresh interpreter imports every template, part, assembly and link model without
loading `build123d` / `OCP`; names the first offender — a new assembly model goes in its module list),
`source_checks.py` (the shared `runs_its_model()` check that a model file ends with its build call),
`tests/cycloidal/` (the drive: one module per part + housing / purchased / fitment / assembly / port,
~230 tests; `from tests.cycloidal.helpers import CFG, …` for the shared config + geometry helpers, the
`stack` fixture is `tests/cycloidal/conftest.py`). Geometry tests are
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
- A part's STEP is git-ignored by the root `.gitignore`'s `*.step` (`git check-ignore -v
  cad/parts/<group>/<name>.step` from the repo root must print that rule; only `vendor/` and `reference/`
  are re-admitted). Nothing else lands in the tree: no `__cadgen__/`, no sidecar unless a model declares
  `kinematics=` (then `<name>.step.json`, committed beside the model).
- After `uv sync` changes cadgen / build123d / OCP, `./cadtool daemon stop`: the warm daemon's workers
  keep the old code loaded (its identity token only tracks cadgen's version and file mtimes; cadgen
  has no stop verb of its own and the daemon shrugs off a bare SIGTERM). The next `gen` starts a fresh one.
- `cadquery-ocp` (VTK) and `cadquery-ocp-novtk` own the same 322 `OCP/` files (the 162 MB kernel `.so`
  included). When uv removes one (2026-09-18: cadgen 0.5 → 0.6 dropped `cadquery-ocp`) it deletes them and
  still counts the other as installed: `uv sync` reports success and `import OCP.gp` fails (a bare `import OCP`
  can still succeed: the leftover `OCP/` directory is an empty namespace package). Repair with
  `uv sync --reinstall-package cadquery-ocp-novtk` (`./cadtool setup` does it when `OCP.gp` does not import;
  `test_tooling.py` checks every RECORD file exists). cadgen's `doctor` does NOT flag an absent kernel.
- cadgen makes hard cutovers (0.6.0: cache / sidecar schemas, so every model read stale once; 0.6.5: the
  inspect CLI): a retired interface fails with a teaching error, never an alias. On a bump re-check the
  private names this repo leans on — `cadgen.authoring.build_in_progress` / `_build` / `ModelDef.func|fmt|script_path|out`
  (`lib/models.py`, `tests/conftest.py`, `test_parts_convention.py`), `cadgen._internal.component_package`
  (`_shape_brep_bytes`, `_build123d_shape_from_brep_bytes`), the `-m cadgen.daemon` cmdline
  of the venv interpreter (`./cadtool daemon stop`; it is `.venv/bin/python3` on Linux and `.venv/bin/python` on macOS —
  the pattern takes both, `test_tooling.py`) — and that `./cadtool why assemblies/arm.py` still lists the 61 children as
  pinned (a `build_in_progress` that silently read False would inline every child and still build).
- A model run accepts only `--force --mesh-tolerance --mesh-angular-tolerance --verbose --json`;
  anything else (`--totals`, a preview flag) is an argparse error — use `./cadtool show` / `python -c`.
- Cycloidal discs: chamfer the lobe edges BEFORE cutting holes (the end face must carry only the
  spline edge); the profile is a periodic *interpolating* spline (`Edge.make_spline(periodic=True)`,
  never `make_spline_approx`); OCCT's analytic volume is ~0.3 % off on that face (both ours and the
  reference) - compare tessellations/face sets, not `.volume`. A boolean between the two discs takes
  minutes: probe points instead.
