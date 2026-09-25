# parts/ — one part per file

Loads when you work in `parts/`. The rules every folder shares (layering, the lazy kernel, never calling a model
outside a build, no `sys.path` shim, the build call at the end of a model file) are in `cad/CLAUDE.md`; the full
regeneration checklist is Recipe C there.

## Authoring a part (`parts/<group>/<name>.py`)
Parts live in subsystem groups (`base`, `joints`, `wrist`, `gripper`, `cycloidal`; templates in
`parts/_templates/`). Groups follow the **physical stage along the arm**, not the name prefix and not
the URDF links (`j1_*` sit in `base/`; `gripper_clamp_bracket` / `gripper_j3_connector` sit in `wrist/`
as the wrist-side mount, the latter although `assemblies/gripper.py` places it) — put a new part with
the stage it bolts to. The group is only a directory: the part's NAME is its key and `parts.load(name)` & co. are
the only way in (`cad/CLAUDE.md` "Rules for every folder"; `import parts` raises on a duplicate name) — a stdlib-only
directory scan in `parts/__init__.py`; note `parts.base` is the *group package*, the part is `parts.load("base")`.
Every part MUST (enforced by `tests/test_parts_convention.py`):
- declare ONE model, **`@step def <name>()`** (`from cadgen import step`; NAME = file stem = model
  name; no parameters, no `out=` — the STEP is the sibling file), that **returns** a valid, labelled
  Part/Compound at its **local origin** — the assembly owns placement; label == module name;
- end with the build call, with no `show()` and no import side effects (`cad/CLAUDE.md` "Rules for every folder");
- import `lib` / `parts` plainly (`cad/CLAUDE.md`): cadgen loads `parts/<group>/<name>.py` as the package module
  `parts.<group>.<name>` (it walks the `__init__.py` chain) — the same object `parts.load()` returns;
- pull shared dims from `lib/params.py`;
- **keep the kernel lazy** (`cad/CLAUDE.md` "Lazy kernel").

Keep the importable `parts/<group>/<name>.py` naming (tests/assemblies reach them through
`parts.load` / `parts.model`); upstream's `src/` + `STEP/` layout is deliberately not used. A part and
its STEP stay siblings (cadgen's default `out`; the viewer pairs them into one entry).

## Part states
Custom parts declare `REFERENCE = NAME`, `CONVERTED` and `LOCAL_FROM_REF`:
- *wrapper* (`CONVERTED = False`, from `_templates/wrapper.py`): the model returns
  `reference/solidworks/<name>.step` (via `lib.reference.load` → `cadgen.read_step`, a tracked input)
  in the SolidWorks part-file frame — the day-one state of every custom part (the two links are parametric: `j2_link` in
  `lib/forearm/` - a `ForearmConfig` with `LEGACY` = the SolidWorks part and `DEFAULT` = what is built -, `j1_link` in
  `lib/upper_arm/` - an `UpperArmConfig`, the same pattern);
- *parametric* (`CONVERTED = True`, from `_templates/designed.py`): real build123d — see "Converting a part" below.
- *designed* (`CONVERTED = True`, `REFERENCE = NAME`, registered in `lib/reference.py DESIGNED`):
  the cycloidal drive's printed parts — the reference is the CadQuery export they were ported
  from; `tests/cycloidal/test_port.py` additionally demands identical face sets / tessellations (the two
  spline discs: the lobe profile within 1e-6 mm of the reference spline, `helpers.spline_deviation`, and the
  mesh within its chordal error — see `cad/CLAUDE.md` "Two machines").
  Their geometry helpers live in `lib/cycloidal/` and each module exposes `build(cfg)` for tests
  (reached like any part: `parts.load(name).build(cfg)` — tests never import `parts.<group>` either).
- *native* (`CONVERTED = True`, `REFERENCE = NAME`, registered in `lib/reference.py NATIVE`; a purchased part
  with neither a SolidWorks export nor a catalog model goes in `NATIVE_COTS`, its envelope IS the geometry):
  designed in this repo with no external origin, so the reference is the build the author ACCEPTED —
  `reference/native/<name>.step` + the manifest entry, written by `./cadtool python
  tools/reference/import_native.py [--only NAME] [--force]` ONCE on one machine (LFS, like a vendor file;
  `--force` = accept a changed design; `import_solidworks.py` keeps the entries). The reference-match test then
  locks the geometry like every other part's; a native part's own tests hold its design intent.
  A new native part: `tools/reference/import_native.py` once, then Recipe C.
- *diverged conversion*: a converted CUSTOM part whose DEFAULT build deliberately leaves its SolidWorks
  reference (the forearm parts, whose elbow end gave way to the roll joint) declares `REFERENCE_BUILD`, a
  zero-arg callable returning the LEGACY configuration that still reproduces the reference —
  `tests/test_reference_match.py` matches THAT build; the default one is locked by the part's own tests.
  A converted CUSTOM part contributes its own build (volume, bbox) to the arm / link totals locks
  (`tests/totals.py`) instead of its SolidWorks record — the record stays in `placements.json` untouched.
- Multi-body parts are registered in `MULTI_BODY` in `tests/test_parts_convention.py`.

## Converting a part (wrapper → parametric)
1. Inspect the reference: `./cadtool inspect reference/solidworks/<name>.step --planes`
   (dimensions: the CAD Viewer's measure tool, or build123d on `cadgen.read_scene(…).resolve("#o1.f7").shape()`);
   `parts/<group>/<name>.py`'s docstring lists units,
   solids, bounding box and where it is used.
2. Rewrite the model body with `BuildPart`/… code. Put every shared dimension in `lib/params.py`
   with a provenance tag (`[MEASURE]`, `[DATASHEET]`, `[DESIGN]`, `[REFERENCE]`, `[ESTIMATE]`)
   and a lock in `tests/test_params_invariants.py`.
3. Set `CONVERTED = True`. If you pick a nicer local origin than the SolidWorks one, set
   `LOCAL_FROM_REF` to the transform *reference frame → new local frame* (frame data `((x, y, z), (rx, ry, rz))`;
   `IDENTITY` until then); the assemblies compose `placement * to_location(LOCAL_FROM_REF)⁻¹`, so
   `reference/placements.json` never changes.
   A SolidWorks part file can hold its geometry far from its own origin (the removed link caps did, ~1 m) —
   `placements.json` compensates; such a part wants this.
4. `./cadtool pytest tests/test_reference_match.py -k <name>` — volume within 0.5 % and bounding
   box within 0.2 mm of the reference (per-part overrides: `REF_VOL_TOL`, `REF_BBOX_TOL`).
5. `./cadtool gen parts/<group>/<name>.py` to regenerate the STEP, then
   `./cadtool gen assemblies/arm.py` + `./cadtool snapshot …` to eyeball it in place — and the rest of Recipe C.

`read_step()` / `import_step()` convert inch-unit files to mm and keep the assembly hierarchy (labels
mangle ` .()` → `_`; `lib.reference.clean_label` mirrors build123d's `import_step`, which the
reference tools keep using).

## Purchased (COTS) parts (`parts/_templates/cots.py`)
`COTS = True`, `MASS_G` (datasheet grams), `PURCHASE_SPEC` (what to order) + `PURCHASE_QTY` (pieces per
occurrence — the whole pattern for a pin / fastener part, so it equals its `MULTI_BODY` count; the drive's are built
from `DEFAULT_CONFIG`, never retyped) + optional `PURCHASE_NOTE` — a printed part declares none of the three —,
`VENDOR_STEP = vendor/<name>.step`, `VENDOR_TO_REF`
(vendor-file frame → SolidWorks frame, as frame data; `IDENTITY` for the SolidWorks re-exports); the model is
hybrid — `return lib.cots.hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)`, the one body every COTS part
shares: `cadgen.read_step(VENDOR_STEP)` if present (a tracked input, so a swapped vendor file makes the part
stale), else `_envelope()` from `lib.params`, both in the SolidWorks frame `placements.json` assumes. `reference/solidworks/<name>.step` keeps the SolidWorks re-export of every COTS part as the
frame/size reference (for the drive's purchased parts: the CadQuery export of their simplified model
in `reference/cycloidal/`, `lib/reference.py CYCLOIDAL_COTS`; `path_of()` resolves the origin);
`test_cots_vendor_matches_reference_frame` (bbox within 1.5 mm, skipped when there is no vendor
file - the envelope is then the geometry) and `test_cots_envelope_tracks_reference_bbox` guard vendor
swaps. Swap procedure (Recipe D): `vendor/CLAUDE.md`; what has been tried and where the vendor files come from:
`vendor/README.md`.
The kit parts `nema17_40mm` / `mks_servo42d` live in `parts/joints/` (`parts/cycloidal/` is locked to
`CYCLOIDAL_COTS`); the drive motor's envelope builder is `lib/cycloidal/motor.py nema17_motor()`, which the 40 mm
envelope reuses with other `MotorParams`.

**Recipe A — add a purchased part** (`parts/<group>/<name>.py`): copy `parts/_templates/cots.py` → declare `COTS`,
`MASS_G`, `PURCHASE_SPEC` / `PURCHASE_QTY` / `PURCHASE_NOTE`, an `_envelope()` from `lib/params.py` → register in
`lib/reference.py COTS` (`rel=None` when the vendor file IS the reference) → put `vendor/<name>.step` in place (Recipe D,
`vendor/CLAUDE.md`) → `./cadtool python tools/reference/import_solidworks.py` (mirrors it into `reference/solidworks/`,
manifest entry) → `tests/test_parts_convention.py MULTI_BODY` if it is several solids → give it an occurrence (a
SolidWorks key, a module row, or Recipe B in `assemblies/CLAUDE.md`) → Recipe C.

## Printed vs. bought
Every part carries the make/buy label once: `COTS = True` in its module means **bought**, anything else is
**printed** (`parts.bought(name)`). Nothing else is kept by hand — the folders follow the arm's physical
stages, not make/buy: never sort parts into make/buy folders or keep a second list by hand. Everything else is
generated from the label:
- **Lists** — `./cadtool python tools/bom.py [--module <module>] [--md|--json]` (a module of `assemblies/arm.py MODULES`) (kernel-free) prints
  what to print (part, quantity) and what to buy (`PURCHASE_SPEC`, pieces = occurrences × `PURCHASE_QTY`, mass, vendor
  file or envelope), counted from the assembly tables. Purchased items that are **not modelled** are the one
  hand-kept table, `EXTRAS` in that tool (`assemblies/CLAUDE.md`).
- **STLs** — `./cadtool python tools/export_printables.py [--parts …]` writes `print/<name>.stl` for every printed part
  (git-ignored, mm, part-local frame, with the quantity to print); bought parts are refused.
- **Colours** — the grey of purchased parts in `arm.step` and every module's STEP (`gripper.step`,
  `cycloidal_drive.step`, `forearm_roll_drive.step`; `assemblies/CLAUDE.md`).
