# Changelog

Dated record of notable changes to this repository (newest first). Every commit that changes
behaviour, layout or tooling gets an entry here; the commit hashes are on `main` (the former
`cad-setup` working branch was fast-forward-only and has been retired).

## 2026-09-21 — `./cadtool daemon stop` did nothing on the Mac (branch `cad/daemon-stop-macos`)

### Fixed — `cad/cadtool`: the daemon's process pattern knew only `python3` (`ef3abb3`)
- `daemon stop` greps the process list for `<cad>/.venv/bin/python3 -m cadgen.daemon`. On the arm64 Mac the venv's
  `python3` is a symlink to `python` and the daemon runs as `.venv/bin/python -m cadgen.daemon`, so the command
  printed "no cadgen daemon running for this venv" and left the daemon and its workers alive - with whatever code
  they had loaded. The pattern is now `python[0-9.]*` (python, python3, python3.12), defined once and reused by
  the `pgrep` and both `pkill`s. Found while cleaning up after the printed-vs-bought work: every `daemon stop` of
  that session had been a no-op (its results stand - the rebuilds picked the changes up, the 41 part STEPs matched
  their checksums and the tests build in-process).
- Verified on the Mac: a forced build starts the daemon + 3 workers, `daemon stop` ends all four, a second stop
  reports none. `tests/test_tooling.py` locks the pattern against both interpreter names (and against matching
  another checkout's daemon). Linux behaviour is unchanged by construction; the Fedora run is still owed.

## 2026-09-21 — printed vs. bought: lists, colours and STLs from the one `COTS` label (branch `cad/printed-vs-bought`)

An audit of the cycloidal drive's purchased parts found all 10 modelled ones consistent (module, occurrence,
mass, reference, manifest — quantities match the spec), and one gap: the 4 arm-mount M4 bolts + 4 captive
nuts (and grease) are bought but were tracked only in the doc's hand-written shopping list. The make/buy label
already existed once per part (`COTS = True`); nothing used it. Commits: `f8b1c8d`, `8f23614`, `2974e67`
(the last two remove the make/buy views again).

### Added — what to order, on every purchased part
- The 15 COTS modules (+ `parts/_templates/cots.py`) declare `PURCHASE_SPEC`, `PURCHASE_QTY` (pieces per
  occurrence; the whole pattern for the drive's pin / fastener parts) and an optional `PURCHASE_NOTE`, next to
  `MASS_G`. The drive's are built from `DEFAULT_CONFIG` (never retyped). `parts.bought(name)` is the one reader of
  the label. Specs of the five SolidWorks-era parts are read off `vendor/README.md` / `lib/params.py` and flagged
  "confirm" where the repo names no exact model (pancake motor, 6 mm rail, 20T pulley bore).
- No geometry changed: all 41 part STEPs rebuilt byte-identical (`shasum -a 256 -c`, arm64 Mac).

### Added — `cad/tools/bom.py`: the print list and the buy list
- `./cadtool python tools/bom.py [--module cycloidal_drive|gripper] [--md|--json]`, counted from the assembly
  tables (kernel-free): 26 printed parts / 34 to print; 15 bought parts / 18 occurrences / 58 pieces. `EXTRAS`
  is the one hand-kept table — purchased items with no geometry (the arm-mount bolts and nuts, grease): on the
  buy list, absent from the model, the totals and the inertials. `docs/cycloidal_drive.md` §9 now points at the
  command instead of retyping the list.

### Changed — every assembly shows what was bought; one STEP per assembly (`assemblies/_occurrences.py`)
- `_tint_parts()`: every purchased part (`parts.bought()`) gets the one `BOUGHT_TINT` grey, printed parts keep
  their link / module colour. `grouped_children()` applies it in the arm (inside the gripper and the drive too);
  `occurrence_children(…, tint=)` / `located_children(…, tint=)` apply it in the standalone `gripper.step` and
  `cycloidal_drive.step`, whose `TINT` constants `arm.py MODULE_TINTS` now reuses. `base_link` moved from grey to
  brown so grey only ever means bought (`test_grey_means_bought`). Colours only — same trees, leaves, solids,
  volumes and bboxes; both modules still link their parts (a tint on a module's direct linked children does
  reach its STEP — verified by snapshot).
- `f8b1c8d` had added two make/buy working views (`arm_make_buy.py`, `cycloidal_drive_make_buy.py`: the same
  leaves under `printed` / `bought` nodes). Both are gone again (`8f23614`, `2974e67`): with the
  colours in the models themselves they were second copies of the same STEP.

### Added — `cad/tools/export_printables.py`: one STL per printed part
- Writes `cad/print/<name>.stl` (git-ignored by the root `*.stl` rule; mm, part-local frame, 0.01 mm / 0.1 rad)
  from the in-process body, with the quantity to print; a bought part is refused, a stale STL is removed.

### Tests
- `test_bom.py` (new, fast), `test_parts_convention.py` (the `PURCHASE_*` contract; a printed part declares
  none), `test_assembly.py` (grey is reserved for purchased parts; leaf colours of the arm, `arm_no_caps`, the
  gripper and the drive). Passed on the arm64 Mac;
  the Fedora run is still owed.

## 2026-09-21 — a caps-off working view of the arm (branch `cad/arm-no-caps`)

### Added — `cad/assemblies/arm_no_caps.py` (`c5cc196`)
- `j1_cap`, `j2_cap_1` and `j2_cap_2` cover `j1_link` / `j2_link`, the parts being worked on. The new model is
  `arm.py`'s `OCCURRENCES` / `GROUPS` minus `HIDDEN` (derived, never retyped), in the same base_link frame,
  component tree and tints, writing its own git-ignored `assemblies/arm_no_caps.step` (49 leaves / 105 solids,
  51 pinned children). `arm.py`, `robot/`, the URDF, the meshes and every existing total are untouched — the
  caps stay part of the robot.
- A second model rather than a switch: `@step` models take no parameters, the freshness gate does not hash
  environment variables, and the build daemon strips unlisted ones from its workers.
- Link edits land in the shared part file, so each arm picks them up on its own next `gen`; the caps are
  wrappers that do not follow a link change — check `assemblies/arm.py` after changing `j1_link` / `j2_link`.
- Tests: `test_assembly.py` (tables = the arm's minus `HIDDEN`, no group emptied, the file ends with its build
  call; slow: the build has 49 leaves / 105 solids / the arm's volume minus the caps'), `test_lazy_kernel.py`
  probes the new module. Passed on the arm64 Mac; the Fedora run is still owed.
- Docs: the zero-code alternative for images is verified and documented —
  `./cadtool snapshot assemblies/arm.step out.png --hide '#j1_cap' --hide '#j2_cap_1' --hide '#j2_cap_2'`.

## 2026-09-21 — the repo also runs on the arm64 Mac; part STEPs are no longer committed (branch `cad/two-machines`)

The repo is now worked on from the Fedora PC **and** an arm64 Mac. Fedora never exercised three platform
assumptions in the tooling, and the committed part STEPs turned out to be per-machine files.

### Changed — the 41 generated part STEPs leave git (`cad/parts/**/*.step`) (`aaa9193`)
- cadgen's STEP bytes are deterministic per kernel **and per machine**: an arm build on the Mac rewrote 17 of
  the 41 committed part STEPs with float noise (≤ 2e-10 mm, `same geometry` in `./cadtool inspect diff`,
  `--force` reproduces the Mac bytes), so either machine's build dirtied the other's files — a new LFS object
  per file per switch. They are build outputs (nothing reads them: the assemblies call the part models, every
  test builds in-process), so they are now git-ignored like `assemblies/*.step` and `robot/links/*.step`:
  the `!/cad/parts/**/*.step` re-admit is gone from `.gitignore`, the files were `git rm --cached` (kept on
  disk). Still committed (Git LFS): the inputs `reference/` (41) + `vendor/` (6), and `robot/meshes/` (8).
- What replaces "`git status`: 0 of 41 changed" as the refactor check: `shasum -a 256 parts/*/*.step` before /
  `-c` after on the same machine (what the assembly STEPs already used), `./cadtool inspect diff` for a file
  that only moved numerical-zero terms.
- **On the next pull git deletes the 41 files from that working tree** (the commit removes them);
  `./cadtool gen assemblies/arm.py` recreates them (a missing STEP reads `STALE (output missing)`).
  A fresh clone has no part STEP until that first build (~35 s).

### Fixed — `cad/cadtool` on macOS's stock bash 3.2 / without GNU coreutils (`aaa9193`)
- `./cadtool gen <model.py>` without flags died with `flags[@]: unbound variable` (bash 3.2 treats an empty
  array as unset under `set -u`): the flags expand as `${flags[@]+"${flags[@]}"}`.
- `./cadtool viewer` uses `timeout`, else `gtimeout`, else runs without the 12 h auto-stop and says so.
- `./cadtool setup` probes `import OCP.gp`: after uv removed the old VTK kernel the hollow `OCP/` directory
  still imported as an empty namespace package, so the `import OCP` probe passed and the repair never ran.

### Changed — `cad/tests/cycloidal/test_port.py`, the two spline discs only (`aaa9193`)
- arm64 meshes the lobe spline into 4–6 more / fewer of ~22 000 triangles (mesh volume 4e-5, centroid
  2e-3 mm) on a profile that matches the reference to 1e-11 mm. The discs now lock the profile itself (new
  `helpers.spline_deviation`, ≤ 1e-6 mm; a 0.01° rotation reads 9e-3 mm) and hold the mesh to its chordal
  error (triangles 0.1 %, volume 1e-4, centroid 5e-3 mm). The other 14 designed parts are unchanged.

### Docs
- `cad/CLAUDE.md` "Two machines" (+ a pointer in the root `CLAUDE.md`): git is the only sync channel, the
  per-machine steps after a toolchain bump, no generated STEP is committed. "committed STEP" reworded in
  `cad/CLAUDE.md`, `cad/README.md`, `cad/docs/cycloidal_drive.md`, `cad/vendor/README.md`, the three
  `parts/_templates/`, `tests/conftest.py` and a `cadtool` comment.

Verified on the Mac: `./cadtool pytest` 587 passed + 9 skipped (before: 2 failed), `gen` without flags,
viewer under `timeout`; with one part STEP removed the arm build recreated it, and after that build
`git status` lists no STEP. **Still to run on Fedora:** pull, `./cadtool gen assemblies/arm.py`,
`./cadtool pytest` (expect the same 587 + 9).

## 2026-09-18 — model files no longer load the CAD kernel at import (branch `cad/lazy-kernel-import`)

cadgen gates a model (freshness check, warm-daemon dispatch) before paying for OCP — but only while
neither `build123d` nor `OCP` is in `sys.modules` when the model is called. Every model file here
imported build123d at module top, so each run, even of a current model, paid the import and printed
cadgen's "kernel was imported before …" hint (documented until now as accepted noise).

### Changed — the whole model import closure is kernel-free at import (`216a119`)
- 55 files (`parts/` 45, `lib/` 7, `assemblies/` 2, `robot/` 1): `from build123d import …` →
  `from cadgen import build123d as bd` (cadgen's PEP 562 proxy) with `bd.<name>` inside function bodies;
  rewritten by an AST-located tool (strings / comments untouched), `from __future__ import annotations`
  added where annotations name kernel types. 33 part files need no kernel import at all any more.
- Kernel objects that were built at import are now data or functions:
  - the 44 frame constants (28 `LOCAL_FROM_REF`, 16 `VENDOR_TO_REF`, all identity) are **frame data** —
    `lib.datum.IDENTITY` or `((x, y, z), (rx, ry, rz))`, made a `Location` by `lib.datum.to_location()`
    inside the body (`IDENTITY` → a bare `Location()`, so nothing changes in the written STEP);
  - `lib.datum.BASE_FRAME` → `base_frame()` (cached), `assemblies.arm.ARM_FROM_W` → `arm_from_w()`;
    `robot/frames.py` re-exports `base_frame`;
  - `assemblies/cycloidal_drive.py OCCURRENCES` rows are `(part, role, position)` tuples, not
    `(part, role, Location)`; `_occurrences.located_children` / `world_rows` build the `Location`;
  - `lib/cycloidal/geom.MIN` (a tuple of `Align`) → `align_min()`.
- New `cad/tests/test_lazy_kernel.py` (fast lane, 0.1 s): a fresh interpreter imports the 3 templates,
  the 41 parts, the 3 assemblies and the 8 link models one by one and fails on the first that loads
  `build123d` / `OCP`. Tests follow the renames (`arm.arm_from_w()`, `F.base_frame()`, frame data in
  `test_reference_match.py`, position rows in `tests/cycloidal/test_assembly.py`).
- Docs: `cad/CLAUDE.md` (the "expected noise" bullet is now the rule + an Authoring bullet: lazy `bd`,
  no kernel object at module level / in defaults, frames as data), `cad/README.md`,
  `cad/docs/cycloidal_drive.md`, the three `parts/_templates/`.

Verified: `./cadtool pytest -q -W error::FutureWarning` 587 passed + 9 skipped (+1 guard test, +1
layering row for it). Every source changed, so all 52 models were stale: rebuilt (daemon stopped first)
in 54 s — 52 STEPs written, **0 eager-kernel hints**, no committed part STEP changed (`git status`: 0 of
41), the 11 assembly / link STEPs byte-identical (`sha256sum -c`), `./cadtool why assemblies/arm.py` =
`current`, 54 children pinned. No-op re-run of an unchanged model: **`parts/base/base.py` 1.5 s → 0.09 s**
(3 runs each), `assemblies/arm.py` 0.25 s, a link 0.10 s, all 41 parts 3.7 s (was ~1.5 s each); the first
no-op check of a model right after a rebuild is slower (up to ~1.6 s, once). `python -X importtime` on
a link and on the arm: 0 kernel modules. Negative control: importing `tools/robot/derive.py` (module-level
`OCP` import, outside any model's closure) trips the same check.

## 2026-09-18 — assemblies are native build123d Compounds (branch `cad/assemblyhelper-migration`)

The follow-up of the cadgen 0.6.5 upgrade below: 0.6.5 deprecated `cadgen.assembly.AssemblyHelper`
(a `FutureWarning` on every assembly / link build, 15 in the suite). Its `add` / `add_module` /
`build` were three lines over native build123d, so the migration changes no output.

### Changed — `AssemblyHelper` replaced by `lib.assembly.assembly()` + children-returning helpers (`a55a516`)
- `cad/lib/assembly.py` is no longer a re-export: `assembly(name, children, *details, color=None)` =
  `Compound(label=label_text(…), children=list(children))`; it still re-exports cadgen's `label_shape` /
  `label_text` (not deprecated — they normalise the label tokens).
- `cad/assemblies/_occurrences.py`: `add_occurrences(asm, …)` → `occurrence_children(…)`,
  `add_grouped_occurrences(asm, …)` → `grouped_children(…)`, `add_located(asm, …)` →
  `located_children(…)` — each RETURNS its placed, labelled children (the group nodes are
  `assembly(group_label, members, color=…)`). The three model bodies (`arm`, `gripper`,
  `cycloidal_drive`) and `robot/_links.build_link` are one `return assembly(name, children)`.
- Docs: `cad/CLAUDE.md` (Assembly: the native convention, "never reintroduce `AssemblyHelper`"; joints are
  `@step(kinematics=…)` data with `cadgen.revolute(…)` mates, not helper frames), `cad/README.md` trees.

Verified: `./cadtool gen` of the 3 assemblies + 8 robot links (daemon stopped first) wrote 11 STEPs with
0 `FutureWarning`s, and **all 11 are byte-identical** (`sha256sum -c`) to the files the `AssemblyHelper`
code wrote on the same cadgen — same component tree, labels, tints, placements and links, so no
snapshot differs; `./cadtool why assemblies/arm.py` = `current`, 54 children pinned, tree `components 50
occurrences 52 links 0` (gripper 19 links, drive 18) as before; no committed part STEP changed (parts
never import these modules). `./cadtool pytest -q -W error::FutureWarning`: 585 passed + 9 skipped
(the 15 warnings of the previous entry are gone).

## 2026-09-18 — cad/ upgraded to text-to-cad v0.6.5 / cadgen 0.6.5 (branch `cad/cadgen-0.6.5`)

Upstream shipped v0.6.0 → v0.6.5 (2026-09-16 … 09-18). What mattered here: the URDF renderer bug this
repo patched locally is **fixed upstream** (0.6.0, `fd566e3e` — `buildUrdfMeshGeometry` now declares
`partTransformsBaked: false`), `cadgen step inspect` was **removed** (0.6.5, hard cutover, no
replacement command), cadgen now depends on `cadquery-ocp-novtk` instead of `cadquery-ocp`, and the
0.6.0 cache / sidecar schema cut makes every model read stale once. Kernel unchanged (build123d 0.11.1 /
OCP 7.9.3); no geometry, placement or robot-description number changed.

### Changed — cadgen 0.6.5, the runtime patch retired, a local `./cadtool inspect` (`bf039e6`)
- Plugin updated 0.5.1 → 0.6.5 (user + project scope). `cad/pyproject.toml`: `cadgen[snapshot]==0.6.5`;
  `cadquery-ocp==7.9.3.1.1` replaced by an explicit `cadquery-ocp-novtk==7.9.3.1.1` (cadgen 0.6 requires
  `build123d>=0.11.1,<0.12` + `cadquery-ocp-novtk>=7.9,<8`; the pin freezes the kernel behind the
  committed STEPs). `uv.lock`: `cadquery-ocp`, `vtk` 9.6.2, `matplotlib` + `contourpy`, `cycler`,
  `kiwisolver` removed.
- **Lesson — `uv sync` gutted the kernel.** `cadquery-ocp` and `cadquery-ocp-novtk` own the same 322
  `OCP/` files (the 162 MB `.so` included); removing the first deleted them while uv still counted
  novtk as installed, so the sync reported success and `OCP/*.so` was gone. Repair:
  `uv sync --reinstall-package cadquery-ocp-novtk` (398 files, 0 missing). `./cadtool setup` now does
  that whenever `import OCP` fails, and `tests/test_tooling.py` checks every RECORD file exists and
  that `cadquery-ocp` is absent. cadgen's own `doctor` does not flag an absent kernel.
- Removed `cad/tools/cadgen_patches.py`, the `./cadtool patch` verb, the `viewer` / `snapshot`
  warnings and the `setup` / `doctor` hooks (the patch was reverted before the sync so no `.orig`
  bundle was orphaned in site-packages). `robot/arm.urdf` renders assembled on the unpatched runtime.
- New `cad/tools/step_facts.py` behind `./cadtool inspect <file.step> [--planes] [--json]` (leaf refs,
  solids, faces, volume, bbox; `--planes` = planar faces as normal / offset / area, for frame-finding)
  and `./cadtool inspect diff <a.step> <b.step> [--tol X]` (exit 1 when the geometry differs) — the two
  jobs the old verb did here, on `cadgen.read_scene`. The retired first arguments
  (`refs|measure|align|frame|interfere|validate`) exit 2 with the new syntax; distances and overlaps
  are `cadgen.geometry.closest_points` / `overlap_volume` in a test.
- `cad/tests/test_tooling.py` rewritten: installed cadgen == the pyproject pin, one complete pinned OCP
  distribution, the retired-verb message, and (slow) `step_facts` totals == `lib.reference.describe()`.
- Docs: 0.5 → 0.6 in both READMEs, both CLAUDE.md, `cad/vendor/README.md`,
  `cad/docs/cycloidal_drive.md`, `cad/.env` and five source comments; the Known-issues cadgen bullet is
  gone from the root `CLAUDE.md`; `cad/CLAUDE.md` Gotchas gained the OCP overlap and the list of
  private cadgen names to re-check on a bump.
- Verified still present in 0.6.5 (all private): `cadgen.authoring.build_in_progress` (thread-local),
  `_build(defn)`, `ModelDef.func/fmt/script_path/out`, `__cadgen_model__`, `__wrapped__`,
  `_internal.component_package._shape_brep_bytes` / `_build123d_shape_from_brep_bytes`,
  `AssemblyHelper.add/add_module(color=)/build`, the `python3 -m cadgen.daemon` cmdline. Unchanged:
  model-run flags, snapshot flags, `store why|gc`, viewer port, no up-axis option (`ARM_FROM_W` stays),
  no upstream `daemon stop` verb.
- **Follow-up:** cadgen 0.6.5 deprecates `AssemblyHelper` (`FutureWarning` on every assembly / link
  build and 15 times in the suite; "existing models still build"). `assemblies/_occurrences.py` and
  `robot/_links.py` should move to native build123d `Compound(children=…)`. Not adopted: `@memo`,
  `declare_input`, `@step(materials=/animation=/kinematics=)`, linked-child tints.

### Changed — the 41 part STEPs regenerated: canonical sign-of-zero only (`816a2a5`)
- cadgen 0.6 writes a canonical zero, so every `-0.` in the STEP text became `0.` and all 41 LFS
  objects changed. After normalising that, old and new `gripper_link_1` (72 changed lines) and
  `j1_link` (464) are line-identical. No sidecar (`*.step.json`) appeared.

Verified: baseline on 0.5.1 — fast lane 349 passed, full suite 582 passed + 9 skipped, `doctor` clean,
arm `current` with 54 children pinned. On 0.6.5 — `./cadtool doctor` clean (`kernel OK`, pin
`cadgen==0.6.5` against the plugin); fast lane 351 passed, full suite **585 passed + 9 skipped** (−1 patch
test, +4 tooling tests); `--force` rebuild of the 41 parts (1 m 29 s) then parts + 3 assemblies + 8 robot
links (daemon stopped first): kernel facts (solids, faces, volume, bbox via `import_step`) of all 41
STEPs equal the 0.5.1 files, 0 of 41 byte-identical, a second 0.6.5 build reproduces the new bytes
41 / 41, `./cadtool inspect diff <old> <new>` = same geometry; `./cadtool why assemblies/arm.py` =
`current`, 54 children pinned, tree `components 50 occurrences 52 links 0` as before (gripper 19 links,
drive 18), second `gen` = `current`; `validate robot/arm.urdf --strict` and `arm.srdf --strict` OK,
`arm.sdf --gz-check never` OK (its `--strict` fails on the 8 `collision_reuses_visual_mesh` warnings,
which 0.5.1 raises too — the test and the docs never used it); snapshots compared with the 0.5.1 ones:
`robot/arm.urdf` assembled with no patch, a posed one (`shoulder_pitch` 45, `elbow_pitch` −30) moves the
rotor + upper arm and leaves the stator, `assemblies/arm.step` keeps its tints and Z-up pose; viewer
HTTP 200 for `?file=robot/arm.urdf` and `?file=assemblies/arm.step` (sliders not clicked — the posed
snapshot runs the same kinematics code); `./cadtool export parts/base/base.step stl` OK (Node);
`./cadtool store gc` removed 660 dead 0.5 objects (91.1 MB), models still `current`.

## 2026-09-18 — `cad/` organization cleanup (branch `cad/organization-cleanup`)

An audit of `cad/` (layout, docs vs. tree, import graph) found the structure sound — `lib/` never
imported upward, 41 parts ↔ 41 STEPs ↔ 41 references, LFS and ignore rules consistent — plus the
defects and smells below. No geometry, placement or robot-description number changed.

### Fixed — models that could not be built, dead code, stale docs (`a2204e0`)
- `cad/robot/links/{forearm,shoulder,upper_arm,wrist_pitch}_link.py` lacked the
  `if __name__ == "__main__": <name>()` build call, so `./cadtool gen` on them was a silent no-op (only
  four of the eight link STEPs had ever been written). New `cad/tests/source_checks.py
  runs_its_model()` (AST) is asserted for every part (`test_part_declares_its_contract`) and every
  link model (`test_every_physical_link_has_a_runnable_model`) — the guard was untested before.
- Removed `cad/lib/export.py` (no importers, superseded by `./cadtool gen` / `export`) and the
  `cad/exports/` directory only it used, with their `.gitignore` rules, `cadtool clean --all` branch and
  README lines. Dropped five unused `hex_prism` imports in `cad/parts/cycloidal/` (STEPs byte-identical).
- Stale text: the "path shim" note and "3-joint arm" in `cad/pyproject.toml`; the mesh-tool path in the
  root `.gitignore`; `cad/README.md` (`gripper_*` count 8 → 7, `cadgen_patches.py` and `robot/_links.py`
  missing from the trees, `__cadgen__/` in the `clean` row); root `CLAUDE.md` (verbs `store`, `daemon`,
  `step`; it now carries a `Last updated` line like every other doc).
- Documented what the `parts/` groups mean — the physical stage along the arm, not the name prefix
  and not the URDF links (`j1_*` in `base/`; `gripper_clamp_bracket` / `gripper_j3_connector` in
  `wrist/`, the latter although `assemblies/gripper.py` places it). No part moved.

### Changed — the two import cycles are gone and the layering is locked (`82613bb`)
- `assemblies ↔ robot`: new `cad/lib/datum.py` owns `frame()`, `U`, `BASE_FORWARD`, `BASE_BOTTOM_Y`
  and `BASE_FRAME`; `robot/frames.py` re-exports them and `assemblies/arm.py` takes `BASE_FRAME` from
  `lib/`. The other direction stays: `robot/_links.py` builds on `assemblies/_occurrences.py`, whose
  `world_rows()` reads a designed module's `OCCURRENCES` / `BODIES`.
- `lib.params ↔ lib.cycloidal`: new leaf `cad/lib/units.py` (`IN`, `NUDGE`); `lib/cycloidal/geom.py`
  imports it instead of `lib/params.py`, which re-exports both (every `from lib.params import NUDGE`
  is unchanged).
- New `cad/tests/test_layering.py` (AST scan, function-local imports count):
  `lib ← parts ← assemblies ← robot ← tools ← tests`, `lib/cycloidal/` never imports `lib/params.py`,
  no direct `parts.<group>` imports outside `parts/`, no `sys.path` outside
  `tools/cycloidal/export_cadquery.py` (it runs in the other repo's venv), no `cad/__init__.py`.

### Changed — `tests/cycloidal/` follows the part-access rule (`cae3ed5`)
- The nine modules that did `from parts.cycloidal import …` now bind `<name> = parts.load("<name>")`.
  `CFG` lives once in `tests/cycloidal/helpers.py` (was pasted into nine files) and the identical
  `stack` fixture moved to the new `tests/cycloidal/conftest.py` (was in three).

### Changed — distinct tool names, shared manifest code (`d0df451`, `2515d37`)
- Renamed (history kept): `tools/robot/frames.py` → `tools/robot/derive.py` (two `frames.py` with
  different jobs), `tools/reference/import_reference.py` → `import_solidworks.py`,
  `tools/cycloidal/import_reference.py` → `import_cadquery.py` (two scripts with one name writing one
  manifest). Docs, docstrings, test messages and the URDF / SDF header comments follow; the older
  entries of this changelog keep the old names.
- New `cad/lib/manifest.py` (`read()` / `write()` / `entry()`) replaces the manifest code both import
  tools duplicated; `test_reference_match.py` and `test_parts_convention.py` read through it.
  Re-running both tools leaves `reference/manifest.json` byte-identical.
- `2515d37`: editing `lib/reference.py` made every wrapper / COTS part stale; all rebuilt
  byte-identical except `parts/base/j1_link.step` and `parts/wrist/gripper_clamp_bracket.step`, which
  came back with 12 numerical-zero terms (≤ 1e-17) written differently — same solids, faces, volume
  and bbox; `--force` reproduces the new bytes (noted in `cad/CLAUDE.md`).

Verified: `./cadtool pytest` 582 passed / 9 skipped (the COTS parts without a vendor file);
`tools/robot/derive.py --check` OK for the URDF and SDF; `./cadtool validate` OK (`.urdf` / `.srdf`
`--strict`, `.sdf --gz-check never`); `./cadtool gen robot/links/forearm_link.py` now writes its STEP;
arm snapshot upright with its tints; `./cadtool doctor` clean.

Left for later (not organization): three COTS envelopes hard-code numbers instead of using
`lib/params.py` (`mg996r_servo` — whose 55.8 disagrees with `MG996R_TAB_L` 54.5 — `mg996r_horn`,
`gt2_pulley_20t`); `parts/_templates/` has no `__init__.py`; `tests/` and `tests/cycloidal/` both hold a
`test_assembly.py`; `robot/arm.sdf` fails `validate --strict` on 8 `collision_reuses_visual_mesh`
warnings (the tests deliberately run it without `--strict`).

## 2026-09-18 — `assemblies/arm.step` is emitted Z up (it rendered lying on its side)

### Fixed — the arm assembly's output frame, branch `cad/arm-step-z-up` (`4328d61`)
- Symptom: `?file=assemblies/arm.step` in the CAD Viewer (and `./cadtool snapshot assemblies/arm.step`)
  showed the arm on its side. Cause: the arm was composed and written in the SolidWorks capture frame
  of `reference/placements.json` (**+Y up**), while cadgen 0.5.1's viewer and snapshot renderer
  hardcode +Z as world up, load STEP coordinates verbatim (only GLB gets a Y→Z correction) and expose
  no up-axis option — no `@step` argument, sidecar section, viewer URL parameter, CLI flag or env var.
  `robot/arm.urdf` was already upright because `robot/frames.py` converts to the REP-103 `base_link` frame.
- `cad/assemblies/arm.py`: `ARM_FROM_W = robot.frames.BASE_FRAME.inverse()` (capture frame → `base_link`
  frame: Z up, X forward, the base's mounting face on z = 0), passed as
  `add_grouped_occurrences(…, root=ARM_FROM_W)`. `cad/assemblies/_occurrences.py`: `place()` and
  `add_grouped_occurrences()` take `root` and compose `root * rel` into every occurrence's placement —
  a `.moved()` on the built root Compound would not reach the file (cadgen's STEP packager reads only
  the children's locations). `arm.step` and `arm.urdf` now open in the same pose.
- Unchanged: `placements.json`, every part and committed STEP, `assemblies/gripper.py` /
  `cycloidal_drive.py` (their own module frames, already axis-on-+Z), all of `cad/robot/` (it reads
  `placements.json` `world`, never the arm compound).
- `cad/tests/test_assembly.py`: the SolidWorks world bbox is compared through `ARM_FROM_W` (which also
  locks "the base stands on z = 0"); new fast `test_arm_is_emitted_z_up`. Docs: `cad/README.md`,
  `cad/CLAUDE.md`, the `lib/params.py` datum note.

## 2026-09-13 — docs: robot description tables, `./cadtool patch` in every guide; cleanup

### Changed — Markdown only (`81858ba`)
- Root `README.md` / `CLAUDE.md`: the robot description chain (links, joints, the cycloidal drive as
  `shoulder_pitch`), the unconfirmed MKS motor → joint mapping, `./cadtool patch` in the verb list and
  the setup line, the cadgen 0.5.x renderer regression under Known issues.
- `cad/README.md`: joint and link tables in the robot section, the arm described by its joints instead
  of "3-joint", the tooling test in the fast lane; `cad/CLAUDE.md`: `BODIES` / `split_key` in the drive
  bullet, the roles note, `test_tooling.py` in the test list.
- Cleanup: the merged `fix/urdf-viewer-render` branch deleted (its commits are on `main`), caches
  dropped (`./cadtool clean`).

## 2026-09-12 — cadgen 0.5.x URDF renderer patched: robots render again in the viewer and in snapshots

### Fixed — `./cadtool patch` (`cad/tools/cadgen_patches.py`), branch `fix/urdf-viewer-render` (`2da0ea4`)
- Symptom: `?file=robot/arm.urdf` / `.srdf` in the CAD Viewer and `./cadtool snapshot robot/arm.urdf`
  drew the robot as a pile of huge overlapping shards, and the joint sliders / `--joint-values` changed
  nothing. Root cause (cadgen's shipped source, `packages/cadgen-js/src`): the robot loader composes
  every link's `linkWorld · visualOrigin · meshScale` correctly, but since 0.5.0
  `common/stepModuleEffects.js displayTransformForPart` applies a part's transform only when the mesh
  data declares `partTransformsBaked: false`, which `lib/urdf/kinematics.js buildUrdfMeshGeometry` never
  sets for robots - so each link was drawn at identity in raw STL millimetres. 0.4.28 still had the
  fallback branch (hence the correct 2026-08-28 renders); nothing in the URDF or the meshes was wrong.
- `cad/tools/cadgen_patches.py` inserts the minified flag into the two installed bundles
  (`cadgen/_runtime/viewer/assets/index-*.js`, `cadgen/_runtime/browser/snapshot-render.js`; `.orig`
  backups beside them; idempotent; anchored on one unique string; refuses cadgen releases outside the
  known list without `--force`). `cadtool`: new `patch [apply|check|revert]` verb, `setup` applies it
  after `uv sync`, `doctor` checks it, `viewer` / `snapshot` warn when it is missing;
  `cad/tests/test_tooling.py` (fast) asserts it is applied.
- Upstream: one line in `packages/cadgen-js/src/lib/urdf/kinematics.js` (add `partTransformsBaked:
  false,` to the lightweight return object of `buildUrdfMeshGeometry`) - github.com/earthtojake/text-to-cad,
  not fixed as of 0.5.1.
- Correction to the 2026-09-11 entry below: the URDF snapshot did NOT match the 2026-08-31 one -
  `arm_urdf_cadgen051.png` was already the pile; it went unnoticed until the 2026-09-12 review.
- Docs: `cad/README.md` (setup, command table, robot section), `cad/CLAUDE.md` (Running things, Robot
  description, Gotchas).

## 2026-09-12 — the cycloidal drive is the `shoulder_pitch` joint; descriptive link/joint names

### Changed — robot description: `link1` split at the drive's output, chain renamed (`63fdc96`)
- Confirmed structure: `j1_coupler` (the holder) yaws on the base and carries the drive's housing +
  motor in its yoke; the drive's output hub is bolted to `j1_link`. The drive is therefore a joint,
  not a rigid part of `link1`. `cad/robot/frames.py LINKS`: `shoulder_link` = `j1_coupler#1` +
  `cycloidal_drive#1:stator` (15 leaves / 52 solids, 1.629 kg), `upper_arm_link` =
  `cycloidal_drive#1:rotor` (output hub, output pins, 625 - 3 leaves / 6 solids) + `j1_link#1` +
  `j1_cap#1` (0.786 kg); the new `shoulder_pitch` joint turns about `N` (the drive's -Z) through
  `SHOULDER_ORIGIN` (= `j1_link#1`'s origin, on the drive axis; `SHOULDER_TO_ELBOW_INPLANE` 210.0 mm).
- Module bodies: `cad/assemblies/cycloidal_drive.py ROTOR` / `BODIES`, `totals(body)`,
  `EXPECTED["bodies"]`; `cad/assemblies/_occurrences.py split_key` / `module_bodies`, and
  `world_rows("<module>#n:<body>")` expands one rigid body (unsuffixed keys unchanged).
  `tools/robot/frames.py link_inertial` and the link-build test look keys up through `split_key`.
- Descriptive names (ROS/UR style) replace the `j1..j3` / `link1..3` numbering - it was meant to mirror
  the MKS motors J1..J3, impossible with four arm joints and an unconfirmed mapping, and collided
  with the part names: joints `base_yaw`, `shoulder_pitch`, `elbow_pitch`, `wrist_pitch` (+ `wrist_roll`,
  `jaw_a`, `jaw_b`, `tool0_joint`); links `base_link`, `shoulder_link`, `upper_arm_link`, `forearm_link`,
  `wrist_pitch_link` (+ `wrist_roll_link`, the jaws, `tool0`). `cad/lib/params.py`: `BASE_YAW_LIMIT_DEG`,
  new `SHOULDER_PITCH_LIMIT_DEG` (120 [ESTIMATE]), `ELBOW_PITCH_LIMIT_DEG`, `WRIST_PITCH_LIMIT_DEG`.
  Part names are untouched.
- `cad/robot/arm.urdf`: new link + joint, `shoulder_link` / `upper_arm_link` inertials and the
  `elbow_pitch` origin re-derived (`--urdf-draft`), ledger rewritten; `arm.sdf` regenerated from
  `--sdf-draft`; `arm.srdf` header, `home` state (+ `shoulder_pitch`), seven Adjacent pairs.
  `cad/robot/links/{shoulder_link,upper_arm_link,forearm_link,wrist_pitch_link}.py`; meshes:
  `shoulder_link.stl` + `upper_arm_link.stl` exported (53 + 8 solids = the former link1's 61),
  `link2.stl` / `link3.stl` renamed to `forearm_link.stl` / `wrist_pitch_link.stl` (frames unchanged,
  bytes kept), `link1.stl` removed.
- `cad/assemblies/arm.py GROUPS` relabelled (`upper_arm_link` tint `#CCB974`); the drive module stays
  whole under `shoulder_link` in the STEP/viewer tree (one linked child; the URDF meshes split it).
- Tests: the partition test accepts `:<body>` keys and demands every body of a designed module exactly
  once; `world_rows` body expansion + error cases; `BODIES` partition and per-body totals locks;
  `TestPoseInTheArm` checks `shoulder_pitch`'s origin on the drive axis and its axis = -(drive Z); the
  FK-at-zero test uses a 5 um per-chain tolerance (six-decimal URDF numbers composed over five
  joints). `./cadtool pytest` 457 passed / 9 skipped; `--check`, `validate --strict` green.
- Docs: `cad/README.md`, `cad/CLAUDE.md`, `cad/docs/cycloidal_drive.md` §12/§13. Snapshots (local,
  git-ignored): `cad/snapshots/arm_shoulder_split.png` (the relabelled arm STEP) and
  `arm_shoulder45_posed.png` - the links posed at `shoulder_pitch` = 45° from `robot/frames.py` and
  written as a STEP, because `./cadtool snapshot robot/arm.urdf` (cadgen 0.5.1) renders the link
  meshes piled at the origin - already so in `arm_urdf_cadgen051.png` from 2026-09-11, i.e. a
  pre-existing renderer problem, not this change (the 2026-08-28 renders were fine).
- Still unconfirmed / untouched: which MKS motor (`src/config.py` J1..J3, all `gear_ratio` 1.0) drives
  which joint - `CYCLOIDAL_RATIO` = 20 applies to `shoulder_pitch`.

## 2026-09-12 — every committed part STEP regenerated on OCP 7.9.3 (writer formatting only, geometry unchanged)

### Changed — the 41 `cad/parts/<group>/<name>.step` files rewritten on the locked kernel (`4284a3a`)
- `d0cffaa`'s STEPs were written by warm daemon workers that still had OCP 7.8.1 loaded (their header
  line read `Open CASCADE STEP processor 7.8` although `cad/uv.lock` already pinned 7.9.3.1.1 — the
  `./cadtool daemon stop` gotcha), so its "byte-identical from here on" held only per kernel: any part
  rebuilt on 7.9 (a `--force`, an edit to a shared `lib/` file) became a new LFS object with a two-line
  diff, first seen on `base` and `j1_cap`. All 41 parts force-rebuilt per model after stopping the
  daemon (`find parts … | xargs -P4 -I{} ./cadtool python {} --force`; `./cadtool gen` runs files
  sequentially and `--force` never cascades to children).
- Verified against the committed LFS objects: 35 files differ only in the header line and the
  `COLOUR_RGB` float formatting (equal sizes); six (`j1_coupler`, `mg996r_servo`, `nema17_48mm`,
  `cycloidal_eccentric_shaft`, `cycloidal_motor_plate`, `cycloidal_ring_gear_body`) additionally carry
  float noise in `DIRECTION` / `CARTESIAN_POINT` values (1e-32 components, `-0.`, 1e-16 residues), and
  the two housing parts a different stored shape tolerance (`UNCERTAINTY_MEASURE` 1e-7 → 1e-5 / 2e-5)
  with renumbered edge references. `./cadtool inspect diff <old> <new>`: same topology and counts for
  all six, bbox deltas ≤ 6e-14 mm; `test_reference_match` + `tests/cycloidal` for the two housing parts
  61 passed, `-m "not slow"` 222 passed, `./cadtool gen assemblies/arm.py` builds and reads current.
  Nothing in the repo reads these files (tests and tools build in-process). From here `--force`
  rebuilds are byte-identical on the locked kernel.
- `cad/CLAUDE.md`: the deterministic-bytes note now says deterministic *per kernel* (the STEP header
  names the OCCT version, so a kernel bump rewrites every file once — stop the daemon first).

## 2026-09-11 — cad/ migrated to text-to-cad v0.5.1 / cadgen 0.5.1 (`@step` models, the `cadgen` CLI, build123d 0.11)

The installed `cad@text-to-cad` plugin was 0.4.28 (2026-08-26, the last 0.4.x); upstream shipped
0.5.0 (2026-09-04) and 0.5.1 (2026-09-08), a deliberate rewrite with **no compatibility layer**
(`docs/migrations/migrating-0.4-to-0.5.md` upstream): no `gen_step()`, no generation CLI, no skill
scripts, no in-tree `__cadgen__/`. The plugin now ships only the `/cad:*` skill docs; the toolchain is
the `cadgen` PyPI package, which is developed against build123d 0.11 / OCP 7.9 — so the CAD kernel
moved too. Migration in two commits: the code/tooling/tests/docs, then the regenerated STEPs (Git
LFS churn kept out of the review diff).

### Changed — cadgen 0.5.1: `@step` models, `cadtool` over the `cadgen` CLI, linked assemblies, build123d 0.11.1 / OCP 7.9.3 (`44cf9be`)
- Plugin updated 0.4.28 → 0.5.1 (user + project scope). `cad/pyproject.toml`: `cadgen[snapshot]==0.5.1`
  (the `playwright` dev dep folded into the extra), **`build123d==0.11.1` and `cadquery-ocp==7.9.3.1.1`**
  (build123d pulls `cadquery-ocp-novtk`, cadgen pulls `cadquery-ocp` unconstrained — pinned to the same
  release or two OCP builds land in the venv; vtk 9.3.1 → 9.6.2). Python stays 3.12. On the old
  build123d 0.10.0 / OCP 7.8.1 pair cadgen 0.5.1 was verified to fail three ways: OCCT 7.8.1's BinTools
  reader mis-frames the VERSION_4 component objects cadgen stores (`ReadShape` uses the "type byte
  first" vertex layout only for VERSION_3; 11 of the 41 parts hit "UnExpected BRep_PointRepresentation"
  and could not be materialized), a linked child could not be exported (`LazyCompound` relies on the
  `wrapped` property build123d 0.11 introduced — OCP raised `AddShape(): incompatible function
  arguments`), and cadgen's STEP writer needs OCP 7.9's `HArray1.Value`. All three vanish on 0.11 / 7.9.
- **A model is a script you run.** All 54 model files (41 parts, 3 templates, 3 assemblies, 7 robot
  links) declare `@step def <name>()` (the old `gen_step()` body; NAME = file stem = model name) and end
  with `if __name__ == "__main__": <name>()` — that call builds and writes the sibling STEP (the
  committed `parts/<group>/<name>.step`; layout unchanged, upstream's `src/`+`STEP/` not adopted).
  The `sys.path` shims are gone everywhere (models, `_occurrences.py`, `tools/`): `cadtool` exports
  `PYTHONPATH=cad/`, pytest has `pythonpath = ["."]`, the new `cad/.env` covers VS Code; cadgen imports
  a model as its package module (`parts.<group>.<name>`, the `__init__.py` chain).
- New `lib/models.py` (`model_of`, `raw` = the body in-process, `geometry` = the linked child while a
  cadgen build runs on the thread, the body otherwise) and `parts.model(name)` / `parts.build(name)`.
  `assemblies/_occurrences.py` places every child through `geometry()` — inside `./cadtool gen
  assemblies/arm.py` the 41 parts + gripper + drive are child models built in parallel and their
  committed STEPs rewritten when stale (pull semantics); the gripper, the drive and the robot links
  link their children's trees, the tinted arm inlines copies (`geometry(model, inline=True)`: cadgen
  keeps a linked child's own colours, so the per-group tints would be lost — verified by snapshot);
  modules are `.moved()` (no in-place `.locate()` on a linked child). `lib/assembly.py` is a plain re-export of cadgen's `AssemblyHelper`
  (the pre-0.4 `cadpy` / pure-build123d fallbacks and their `relations` field deleted).
- Vendor and reference STEPs are read with `cadgen.read_step` (`lib.reference.load`, the 5 plain COTS
  parts, `parts/cycloidal/_cots.hybrid`): store-cached, and a tracked input — swapping a vendor file
  makes the part stale. `lib.reference.describe` and `tools/reference/extract_placements.py` keep
  `build123d.import_step` (label mangling `clean_label` mirrors).
- `cad/cadtool` rewritten: `gen`/`step` runs the model script(s) (`--force`, `--json`, …); new `show`
  (OCP preview of the body via `tools/preview.py`, since `__main__` now builds), `why` (`cadgen store
  why`), `doctor` (plugin pin / node / chromium), `export <file.step> stl|3mf|glb [out]`,
  `snapshot <doc> <out.png>`, `inspect …` (`cadgen step inspect`), `validate` (`cadgen urdf|srdf|sdf
  validate`), `viewer` (`cadgen viewer`, URL `http://127.0.0.1:3245/?file=<rel>`), `cadgen|store|daemon`
  passthrough and `daemon stop` (cadgen has no stop verb; its warm workers keep old code loaded after
  a `uv sync` and ignore a bare SIGTERM). Gone: `artifact`, `CADGEN_WARM`, the vendored-cadgen
  `PYTHONPATH` fallback, `VIEWER_CAD_PYTHON`; the plugin dir is picked by the venv's `cadgen --version`.
- Tests: new `tests/conftest.py` (`CADGEN_DAEMON=0` + a guard that fails any test calling a model at
  top level — a top-level call would rewrite the STEP and start the daemon); `test_parts_convention`
  asserts the `@step` model named after the file with `fmt == "step"` and no `out=`, that no `gen_step`
  survives, and (new, slow) that every part round-trips through cadgen's component BREP writer; every
  geometry call site uses `parts.build(name)` / `lib.models.raw(model)`; the validator test no longer
  skips. `tests/cycloidal/helpers.interference` runs OCCT's `BRepAlgoAPI_Common` directly: build123d
  0.11 reworked `Shape.intersect` for composite operands (the placed drive against the base reported
  73 818 mm³ instead of 0).
- Root `.gitignore`: the `__cadgen__/`, `.*.step.glb`, `.*.step.js`, `.*.step/` rules removed (0.5 keeps
  everything derived in `~/.cache/cadgen`); `cad/assemblies/__cadgen__/` deleted.
- Docs rewritten for the new commands and conventions: `cad/README.md`, `cad/CLAUDE.md`, root
  `README.md` / `CLAUDE.md`, `cad/docs/cycloidal_drive.md`, the three templates, `lib/export.py`.
- Accepted behaviour changes: every `gen` prints cadgen's eager-kernel hint (build123d imported at
  module top; ~2.5 s per run — lazy `cadgen.build123d` imports are a follow-up); `export` writes one
  mesh format per call and needs Node ≥ 20; snapshots land exactly at the path given (no timestamp,
  no GIF orbit); `--totals` is `totals()` via `./cadtool python -c`; the `/cad:*` skills in Claude
  Code change after a restart.
- Verified: `./cadtool pytest` 413 passed + 9 skipped on 0.4.28 (baseline) and **454 passed + 9 skipped**
  on 0.5.1 / build123d 0.11.1 (the 41 new round-trip tests; no warnings); the fast lane 222 passed;
  `tools/robot/export_link_meshes.py` reproduces `robot/meshes/*.stl` byte-identically (raw mode +
  `read_step` = the 0.4 geometry); the three validators pass; `./cadtool gen assemblies/arm.py` builds
  the 41 parts + gripper + drive as child jobs (`why`: 54 children pinned == current), a rerun prints
  `current`, `--force` rewrites all 44 STEPs byte-identically; `cadgen step inspect refs --facts` of every
  part, the three assemblies and `robot/links/link1.step` is identical to the 0.4 baseline (faces, edges,
  occurrences, size, centre, diagonal; `inspect diff` flags only float-level writer differences, no face
  or edge deltas); the arm / drive / URDF snapshots match the 2026-08-31 ones, group tints included;
  `./cadtool viewer` serves `?file=assemblies/arm.step` (HTTP 200, `viewer list|stop` work);
  `./cadtool export assemblies/cycloidal_drive.step stl` writes 5.5 MB through Node; `./cadtool doctor`
  → pin OK, node 22, chromium OK.

### Changed — every committed part STEP regenerated with cadgen 0.5.1 (`d0cffaa`)
- The 41 `cad/parts/<group>/<name>.step` files rewritten by `./cadtool gen assemblies/arm.py` (each
  part a child job of the arm) — new Git LFS objects. The 0.4 writer embedded provenance
  (`cadgen:sourceHash`, the generator path) and a build timestamp in every STEP; 0.5 writes none and
  pins the timestamp, so `--force` rebuilds are byte-identical from here on. Geometry facts (faces,
  edges, occurrences, bounds) identical to the 0.4 files for all 41; `assemblies/*.step` and
  `robot/links/*.step` stay git-ignored and regenerate the same way.

## 2026-08-31 — arm assembly grouped into link components, per-group viewer tints (`bdb69d0`)

`assemblies/arm.py gen_step()` now builds the component tree `arm → base_link/link1/link2/
link3/wrist` — the rigid-link partition of `robot/frames.py LINKS`, with the `gripper` and
`cycloidal_drive` modules kept whole (`wrist` = wrist_roll_link + the jaw links) — so each
component toggles as one node in the OCP CAD Viewer / CAD Viewer trees, and every subtree is
tinted with its group's color (`arm.py GROUPS`; `MODULE_TINTS` keeps the two named modules
distinct). New `assemblies/_occurrences.py add_grouped_occurrences()` does the bucketing and
tinting and raises unless the groups cover the occurrence keys exactly once;
`tests/test_assembly.py` locks the group labels and the LINKS mirror
(`test_arm_groups_mirror_links`). Totals unchanged: 52 leaves / 108 solids / volume / bbox.
Note: the tints ride the in-memory compound and the regenerated (git-ignored)
`assemblies/arm.step`; a few COTS/multi-solid leaves lose their color in the STEP round-trip —
the live `./cadtool python -m assemblies.arm` preview shows all of them.

## 2026-08-28 — `cad/` reorganised: grouped parts, split references, packaged tests/tools, `cadtool clean`, Git LFS

The folder had outgrown its flat directories (87 files in `parts/`, 44 in `reference/`, 17 test
modules). Parts are now grouped by subsystem, references by origin, the drive's tests and the
tools are packages, and the 95 committed STEP/STL binaries are Git LFS objects. No part was
renamed: every name still keys the manifest, the reference file, `placements.json` and the URDF
links. Baseline suite 410 + 9 skipped → 412 + 9 skipped (two new discovery locks).

### Changed — tests/tools packages, robot/links stubs, `cadtool clean` (`35dbdc5`)
- `cad/tests/cycloidal/` (the drive's 10 modules + `helpers.py`, imported as
  `tests.cycloidal.helpers`) and `cad/tools/{reference,cycloidal,robot}/` as packages
  (`robot_frames.py` → `tools/robot/frames.py`, `export_cycloidal_cadquery.py` →
  `tools/cycloidal/export_cadquery.py`, …); `test_robot` imports `tools.robot.frames` instead of
  hacking `sys.path`. `robot/links/*.py` are 4-line stubs (a literal `def gen_step()`, which the
  plugin's AST lookup needs). `./cadtool clean [--all]` deletes `__cadgen__/` (261 MB of viewer
  caches), `__pycache__/`, `.pytest_cache/` (and, with `--all`, the git-ignored review artifacts).

### Changed — `parts/` grouped by subsystem (`843eaf6`)
- `cad/parts/{base,joints,wrist,gripper,cycloidal}/` — each part module moved *with* its
  committed STEP (the viewer pairs siblings and scans recursively); templates in
  `parts/_templates/`, the drive's shared COTS body in `parts/cycloidal/_cots.py`; path shims
  `parents[2]`. `parts/__init__.py` scans the groups (`MODULES`, `GROUPS`, `names()`, `load()`,
  duplicate stems raise) and every part import goes through `parts.load()`.
- Root `.gitignore`: the STEP re-admits are recursive (`!/cad/parts/**/*.step`,
  `!/cad/reference/**/*.step`) — the old single-level globs would have silently un-committed any
  new STEP in a group directory. `test_reference_match` gains a discovery guard,
  `test_parts_convention` a `parts/cycloidal == CYCLOIDAL_PARTS` lock.

### Changed — `reference/` split by origin (`ced1497`)
- `cad/reference/solidworks/` (25 SolidWorks exports) and `cad/reference/cycloidal/` (16 CadQuery
  exports of the drive); `lib.reference.path_of()` resolves the origin from the registries; both
  import tools write there and record a `file` field in `manifest.json` (added to all 41 entries
  without re-running the importers — every checksum unchanged).

### Changed — Git LFS for the `cad/` binaries (`c4fa78e`)
- `/.gitattributes`: `cad/**/*.step` and `cad/**/*.stl` use the LFS filter; `git add --renormalize`
  converted the 95 tracked files (40 MB: parts 41, reference 41, vendor 6, robot meshes 7) into
  pointers in one commit — history untouched. Clones need `git-lfs` (`git lfs install`; `git lfs pull`
  on a checkout that shows pointer files); every regenerated STEP is a new LFS object against the
  GitHub LFS quota.

### Changed — docs (`6301516`)
- `cad/README.md` (setup / LFS, cadtool table incl. `clean`, the layout tree, part conventions,
  converting, purchased parts, references, robot, tests), `cad/CLAUDE.md`, `cad/docs/cycloidal_drive.md`,
  `cad/reference/README.md`, `cad/vendor/README.md`, root `README.md` / `CLAUDE.md` follow the new paths.

## 2026-08-28 — the cycloidal drive: imported, ported to build123d, attached

The 20:1 cycloidal shoulder drive designed in the separate `cycloidal_drive` CadQuery repo now
lives in `cad/` as parametric build123d, verified against the CadQuery exports, attached to the
arm at its SolidWorks pose and included in the robot description. Spec, port notes and attachment:
`cad/docs/cycloidal_drive.md`. The old repo is untouched.

### Added — `cycloidal_drive` history imported (`6739509`)
- `github.com/hbenuid/cycloidal_drive@2f1f67d` (117 commits) merged with the subtree technique
  (`git fetch` + `merge -s ours --no-commit --allow-unrelated-histories` + `read-tree --prefix`,
  `git subtree` is not installed) under `cad/cycloidal_import/`; the directory is removed again in
  the docs commit below once everything was ported — `git log` keeps every original commit.

### Added — printed parts ported (`f670d20`)
- `cad/lib/cycloidal/`: `DriveConfig` (ten frozen dataclass groups; dead fields dropped, the
  builders' magic numbers promoted to tagged fields), `layout.py` (hole patterns, engagement depths,
  `stack_positions` = the drive repo's `assembly.py` layout: ring pins z 5.5, output pins z 11 —
  its `export.py` had them 1–2 mm off), `profiles.py` (numpy epitrochoid), `housing.py` (shared
  reveal-window cutter, outer-silhouette chamfer by end-face height, hex prisms), `disc.py`
  (periodic interpolating spline, lobe chamfer applied before the holes), `geom.py`.
- `parts/cycloidal_disc_1`, `cycloidal_disc_2` (−9° profile phase — the discs are distinct
  parts), `cycloidal_eccentric_shaft` (D-bore as bore ∩ half-space, no 0.25 mm sliver),
  `cycloidal_motor_plate`, `cycloidal_ring_gear_body` (cones instead of 21 ruled lofts),
  `cycloidal_output_hub` — the first `CONVERTED = True` parts of the repo.
- References: `tools/export_cycloidal_cadquery.py` (runs in the old repo's venv) exports the
  CadQuery builders as house-named STEPs; `tools/import_cycloidal_reference.py` copies them into
  `reference/` with manifest kind `designed` / `cots` (`origin: cycloidal_drive@2f1f67d`);
  `import_reference.py` leaves those entries alone. `lib/reference.py` gains `DESIGNED`,
  `CYCLOIDAL_COTS`, `CYCLOIDAL_PARTS`, `DESIGNED_MODULES` and the file helpers.
- `lib/params.py`: `CYCLOIDAL_*` interface constants (ratio 20, OD 140, stack depth 60, hub OD /
  proud / output face z 65, arm-mount 4× M4 on Ø50 @ 45°) re-exported from the config, steel
  density and the bearing / motor / fastener masses, with locks. `uv add numpy`.
- Tests: the drive's 129 part tests ported one module per part (`tests/test_cycloidal_{disc,
  eccentric_shaft,motor_plate,ring_gear_body,output_hub,housing}.py`, `cycloidal_helpers.py`) and
  `test_cycloidal_port.py`: every designed part reproduces its CadQuery export — identical face
  sets and tessellations (the analytic parts also to 1e-13 in volume). OCCT's analytic volume is
  ~0.3 % off on the discs' 2000-knot spline face (both sides), so the discs keep the 0.5 % default
  in the reference match and are compared by tessellation instead.

### Added — purchased parts ported (`7c4b9aa`)
- `parts/bearing_6003`, `bearing_6814`, `bearing_625`, `nema17_48mm`, `cycloidal_ring_pins` (21),
  `cycloidal_output_pins` (4), `cycloidal_shaft_support_pin`, `cycloidal_motor_bolts` (4),
  `cycloidal_housing_bolts` (8), `cycloidal_housing_nuts` (8): COTS modules whose `_envelope()` is
  the drive repo's simplified model (also their reference STEP); `MULTI_BODY` entries.
- step.parts: `bearing_625_2rs_sealed_simple` adopted (`vendor/bearing_625.step`, identity
  frame). Rejected: `bearing_6003_2rs_sealed_simple` (it is a Ø24 × 8 bearing) and
  `stepper_motor_nema17_l0048_single_shaft` (14.8 mm shaft, the drive needs 22 mm); no 6814 in the
  catalog. The manifest `vendor` block is now optional and `test_reference_match` tolerates its
  absence (envelope in use). Tests: `test_cycloidal_purchased.py` (35), `test_cycloidal_fitment.py` (16).

### Changed — assembly: `cycloidal_drive` module at the SolidWorks pose (`8b89e20`)
- `assemblies/cycloidal_drive.py`: 18 rows `(part, role, Location)` from `stack_positions`,
  fasteners included; `EXPECTED` lock 18 leaves / 58 solids / 691 936.8 mm³; `--totals` helper.
  `assemblies/_occurrences.py`: `place_at` / `add_located` (code-driven rows), `world_rows` /
  `place_world_at` (expand a designed module into world-placed parts).
- `reference/placements.json` regenerated: the `New cyloidal assembly` node is a **designed module**
  record `cycloidal_drive#1` (pose verbatim the former `skipped[0]`, the node's totals / bbox kept
  as a `solidworks` cross-check, no descent) — `extract_placements.py` designed-module branch,
  `lib/placements.py keys(designed=…)`; nothing is skipped any more. `assemblies/arm.py` places
  the module after `j1_coupler`: 16 top-level occurrences + 2 modules = 52 leaves / 108 solids.
- Verified in place: the drive intersects nothing in base / j1_link / j1_cap and only touches the
  `j1_coupler` yoke (≤ 150 mm³ contact); its hub face is coplanar with `j1_link`'s mounting face;
  its world bbox matches the SolidWorks node within 1.5 mm. `tests/test_cycloidal_assembly.py`
  (49): the 43 ported clearance checks, the layout / totals locks, the interference budget (only
  the 7 designed overlaps: 6814/hub press fits, bolts through the solid nuts, motor-bolt heads /
  tips, 6003/lobe press fits) and the pose checks; `test_placements` / `test_assembly` updated.

### Changed — robot description: the drive rides in `link1` (`646e762`)
- The drive is physically the **shoulder-pitch joint** between `j1_coupler` (housing in its yoke)
  and `j1_link` (hub bolted to it) — the earlier "J1 base-yaw actuator" wording was wrong. It is
  **not modelled as a joint yet**: `LINKS["link1"]` carries the module key, `robot/_links.py` and
  `tools/robot_frames.py` expand it through `world_rows`; link1 is now 61 solids / 2.416 kg
  (`meshes/link1.stl` 2.79 MB), URDF + SDF inertials re-derived, ledger rewritten (drive assumption,
  motor→joint mapping unconfirmed). `--check`, strict validators and `tests/test_robot.py` green.
- `src/config.py` is untouched: J1 `gear_ratio` stays 1.0 while the CAD says
  `CYCLOIDAL_RATIO = 20` — which MKS motor drives which joint is still to be confirmed.

### Changed — docs: how to view the drive and the URDF (`bbb3e51`)
- `cad/docs/cycloidal_drive.md` "Viewing the drive" (CAD Viewer URLs, snapshots, orbit GIF, OCP
  viewer) and `cad/README.md` robot section (the URDF in the viewer: meshes + joint sliders, the
  drive moves with `link1`).

### Changed — docs; `cad/cycloidal_import/` removed (`f055a1a`)
- `cad/docs/cycloidal_drive.md`: the drive's spec carried over and corrected (its old §10 claimed
  "both discs are identical, the 180° offset is applied in the assembly" — wrong; 7.6 mm → 7.4 mm
  disc holes; rotted 67 / 134 / 120 mm comments), where things live in `cad/`, port notes
  (idiom table, promoted numbers, dropped fields, interference budget, OCCT volume caveat), the
  attachment and the change policy.
- `cad/README.md`, `cad/CLAUDE.md`, `cad/reference/README.md`, `cad/vendor/README.md`, root
  `README.md` / `CLAUDE.md` updated (designed part state, code-driven module, 16 reference rows,
  catalog decisions); the CadQuery import directory deleted. Suite: 419 tests (410 + 9 skipped
  vendor-frame checks for envelope-only parts).

## 2026-08-28 — text-to-cad v0.4.28, purchased-part workflow, robot description

### Changed — CAD plugin updated to `cad@text-to-cad` v0.4.28 (`cc1b028`)
- The installed plugin was 0.3.2 (June); upstream had moved to 0.4.28 with breaking changes.
  The update needed `git-lfs` on `PATH` (installed to `~/.local/bin`, user scope).
- `cadgen==0.4.28` (the plugin's Python runtime, PyPI) is now a locked dependency of `cad/`
  (`requires-python >=3.11`), replacing the old `cadpy` PYTHONPATH trick; `lib/assembly.py`
  imports `cadgen.assembly` first.
- `cad/cadtool` remapped: `gen` (alias `step`) → `scripts/gen … --write`, new `export`
  (STL/3MF/GLB), `artifact`, `validate` (URDF/SRDF/SDF), `parts` (step.parts downloader),
  generic `skill <skill> <tool>`; `viewer` runs the new Python backend in the CAD venv
  (`VIEWER_CAD_PYTHON`, port 3245, URL `http://127.0.0.1:3245/<abs cad>?file=…`, 12 h timeout).
- Hidden viewer artifacts moved from `.<name>.step.glb` to `__cadgen__/` directories
  (git-ignored); orphaned sidecars deleted. All 27 STEPs regenerated with the new CLI.
- Docs rewritten for the new commands; the CAD datum comment corrected (the SolidWorks
  capture frame is **Y up**).

### Added — step.parts vendor workflow (`8e16a0f`)
- Every purchased (COTS) part keeps its SolidWorks re-export in `cad/reference/<name>.step`
  as an immutable frame/size reference; `cad/vendor/<name>.step` is the current best model
  and may be replaced. `reference/manifest.json` records both (`vendor` sub-entry).
- COTS parts declare `VENDOR_TO_REF` (vendor-file frame → SolidWorks frame);
  `test_cots_vendor_matches_reference_frame` guards swapped models (bbox within 1.5 mm).
- Procedure in `cad/vendor/README.md`. Tried `gt2_pulley_20t_bore5_w6` from step.parts and
  rejected it (analytic simplified model); MG996R not in the catalog; pancake catalog model is
  simplified; no Ø6 rod — all kept as the SolidWorks re-exports.

### Added — robot description from the CAD (`d84354a`)
- `cad/robot/frames.py`: 7 rigid links partitioning all 34 part occurrences and 5 actuated
  joints (`j1`–`j3` revolute, `wrist_roll` revolute, `jaw_a`/`jaw_b` prismatic with mimic)
  + `tool0`, computed from `reference/placements.json`. REP-103 base frame; every joint frame
  has Z on its axis; all joints are 0 at the SolidWorks capture pose.
- Per-link generators (`robot/links/*.py`) and meshes (`robot/meshes/*.stl`, 6.7 MB, mm);
  `tools/export_link_meshes.py`, `tools/robot_frames.py` (joint origins, OCP inertials,
  URDF/SDF drafts, `--check`).
- `robot/arm.urdf` (source of truth, ledger comment), `robot/arm.srdf` (MoveIt2 groups,
  states, end effector), `robot/arm.sdf` (model-level 1.12) — all pass the plugin's strict
  validators; URDF at zero reproduces the CAD; posed sweeps verified visually.
- Placeholder joint limits / effort / velocity in `lib/params.py` (`[ESTIMATE]`);
  `tests/test_robot.py` (20 tests). Suite: 117 tests.
- Known placeholders: axis signs, limits, jaw travel, two link-membership assumptions
  (see the URDF ledger); the cycloidal drive was not modelled (imported and attached the same
  day, see above); wrist roll and jaws are not driven by `src/config.py`.

## 2026-08-27 — CAD workspace scaffold (`91bfcf1`)

### Added
- `cad/`: a separate uv project (Python 3.12, build123d 0.10) wired to the `cad@text-to-cad`
  plugin: `cadtool` wrapper, `lib/` (`params.py` dimension SSOT, `reference.py`,
  `placements.py`, `assembly.py`, `export.py`), part templates.
- 20 custom parts as import wrappers around their renamed SolidWorks reference STEPs
  (`cad/reference/`, immutable, with `manifest.json`) and 5 COTS parts with vendor STEPs; each
  part's generated `.step` committed beside its source.
- `assemblies/arm.py` + `gripper.py` placing all 34 occurrences from placements extracted
  from the full SolidWorks assembly (`tools/extract_placements.py`, `reference/placements.json`);
  the rebuilt arm matches SolidWorks exactly (50 solids, 1 938 168.8 mm³, same bbox).
  The cycloidal drive is skipped (lives in the `cycloidal_drive` repo; pose recorded).
- Tests (92): part convention, reference match, placements integrity, assembly totals,
  params locks. Docs: `cad/README.md`, `cad/CLAUDE.md`, `cad/reference/README.md`,
  `cad/vendor/README.md`; root `.gitignore`/`README.md`/`CLAUDE.md` updates;
  `.claude/settings.json` enabling the plugin marketplace; `robotic-arm.code-workspace`.
