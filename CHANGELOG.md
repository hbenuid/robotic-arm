# Changelog

Dated record of notable changes to this repository (newest first). Every commit that changes
behaviour, layout or tooling gets an entry here; the commit hashes are on branch `cad-setup`.

## 2026-08-31 — arm assembly grouped into link components, per-group viewer tints

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
