# tools/ — scripts that derive, check and export

Loads when you work in `tools/`. Tools sit near the right end of the layering (`cad/CLAUDE.md` Layering): they may
import `lib`, `parts`, `assemblies` and `robot`; only `tests` imports them.

## Rules
- Run a tool from `cad/` as `./cadtool python tools/<path>.py [args]` (the venv, `PYTHONPATH=cad/`) — never bare
  `python`. `./cadtool inspect` is `tools/step_facts.py`.
- A tool is not a model: it may import build123d at module level (the lazy-kernel rule covers models and their
  import closure), and it builds bodies in-process — `parts.build(name)` / `lib.models.raw(model)` — never by calling
  a model (`cad/CLAUDE.md` "Never call a model outside a build").
- `tools/cycloidal/export_cadquery.py` runs in the cycloidal_drive repo's CadQuery venv, never ours, and holds the
  tree's one `sys.path` line (`tests/test_layering.py SYS_PATH_ALLOWED`).
- The tools marked *writes* below change committed inputs: run them only as their recipe says — Recipe C
  (`cad/CLAUDE.md`), the regeneration order (`reference/CLAUDE.md`), Recipe D (`vendor/CLAUDE.md`). The ones that
  write a STEP or STL (its bytes differ per run and per machine) run on ONE machine per change.

## What is here
| tool | does | writes |
|---|---|---|
| `bom.py` | the print list and the buy list from the make/buy label; `EXTRAS` = purchased items with no geometry, and the order line of a part modelled but not placed yet (`parts/CLAUDE.md`) | — (stdout) |
| `export_printables.py` | one STL per printed part into `print/` (git-ignored) | — |
| `step_facts.py` | `./cadtool inspect` / `inspect diff` of saved STEPs | — |
| `robot/derive.py` | joint origins, link inertials, URDF / SDF drafts; `--check` compares the checked-in files | — (stdout) |
| `robot/joint_loads.py` | each joint's worst-case static load (N·m) and inertia from `robot/arm.urdf`'s inertials, a payload at `tool0` (`--payload`) - what a drive has to hold | — (stdout) |
| `robot/export_link_meshes.py` | `robot/meshes/<link>.stl` (`tests/test_robot.py` fails on a stale one) | *writes* |
| `reference/import_solidworks.py` | the SolidWorks exports → `reference/solidworks/`, seeds `vendor/`, `manifest.json` | *writes* |
| `reference/extract_placements.py` | the monolith → `placements.json` (+ `vendor/nema17_pancake.step` unless `--no-pancake`) | *writes* |
| `reference/mount_placements.py` | the `lib/mounts.py` records merged into `placements.json` | *writes* |
| `reference/split_mks_motor.py` | the MKS kit exports → `vendor/nema17_40mm`, `mks_servo42d`, `nema17_48mm` | *writes* |
| `reference/import_native.py` | a native part's accepted build → `reference/native/`, its manifest entry | *writes* |
| `cycloidal/export_cadquery.py` | the drive's CadQuery exports (in the other repo, its git-ignored `export/`) | — |
| `cycloidal/import_cadquery.py` | those exports → `reference/cycloidal/`, their manifest entries | *writes* |
