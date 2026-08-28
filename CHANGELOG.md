# Changelog

Dated record of notable changes to this repository (newest first). Every commit that changes
behaviour, layout or tooling gets an entry here; the commit hashes are on branch `cad-setup`.

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
  (see the URDF ledger); the cycloidal drive is not modelled; wrist roll and jaws are not
  driven by `src/config.py`.

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
