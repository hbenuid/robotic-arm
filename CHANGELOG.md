# Changelog

Dated record of notable changes to this repository (newest first). Every commit that changes
behaviour, layout or tooling gets an entry here; the commit hashes are on `main` (the former
`cad-setup` working branch was fast-forward-only and has been retired).

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
