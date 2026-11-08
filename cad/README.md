# robotic-arm — CAD (build123d)

Parametric CAD-as-code for the desktop arm (base yaw, cycloidal shoulder pitch (`CYCLOIDAL_RATIO`), elbow pitch,
forearm roll, wrist pitch, wrist roll — every joint but the shoulder belt-driven — and a rack & pinion parallel gripper
on an MG996R),
converted part-by-part from the original SolidWorks design. This folder is a **separate uv project** (Python 3.14) —
the motor-control software (`software/control/`) never depends on it.

**Status:** the printed parts are real build123d, bar the few still an *import wrapper* around their SolidWorks
export (`reference/solidworks/<name>.step`; [`docs/open_issues.md`](docs/open_issues.md) "Not converted yet");
purchased parts use a vendor STEP where there is one, else their parametric envelope; and `assemblies/arm.py` places
them from `reference/placements.json` (the SolidWorks capture plus the mounts `lib/mounts.py` declares), the drive
modules placing their own parts from code — so the whole arm assembles, renders and is tested. Converting a part means
replacing its wrapper body with real build123d code (see [`parts/AGENTS.md`](parts/AGENTS.md) "Converting a part").
The **cycloidal shoulder drive is fully parametric build123d** (`lib/cycloidal/`, `assemblies/cycloidal_drive.py`),
ported from the `cycloidal_drive` repo and verified against its CadQuery exports; its shell is the output, the
`j1_coupler` yoke holding its hub and motor — see [`docs/cycloidal_drive.md`](docs/cycloidal_drive.md) §5.5. The
forearm roll and wrist roll drives are code-driven modules too ([`docs/forearm_roll.md`](docs/forearm_roll.md),
[`docs/wrist_roll.md`](docs/wrist_roll.md)).

## Setup (once per machine)

Requirements: [`uv`](https://docs.astral.sh/uv/) and Node 20+ (only for STL/3MF/GLB export). For Claude Code, the
[`text-to-cad@earthtojake`](https://github.com/earthtojake/text-to-cad) plugin at the pinned release (AI CAD assistance,
below; the repo's `.claude/settings.json` enables its marketplace) — installed AFTER `./cadtool setup`, once per scope,
`--scope project` for the repo's own:

```bash
cd cad
./cadtool setup                  # uv sync (build123d/OCP/cad into ./.venv, bytecode compiled; reinstalls the OCP kernel if it does not import)
                                 # + the git pre-commit hook (ruff on staged .py files) + git notes fetching
                                 # + the machine's cad-server at the pin, its telemetry stored off first
claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git
claude plugin install text-to-cad@earthtojake
claude plugin install text-to-cad@earthtojake --scope project   # the repo's own scope (docs/toolchain.md)
./cadtool gen assemblies/arm.py  # build the arm: generated STEPs are git-ignored, each machine builds its own
./cadtool pytest                 # everything green?
```

Always run through `./cadtool …` from `cad/`; `./cadtool doctor` checks the install. The pins (Python, cad,
the CAD kernel), updating the plugin and what a pull that changes `pyproject.toml` / `uv.lock` needs:
[`docs/toolchain.md`](docs/toolchain.md). CI (run by hand, a clean clone on fresh runners): the root
[`AGENTS.md`](../AGENTS.md) "CI".

## `./cadtool` — the one entry point

| Command | What it does |
|---|---|
| `./cadtool gen parts/<group>/<name>.py` (alias `step`) | **run the model script**: the machine's cad server builds it on this venv's Python and writes `parts/<group>/<name>.step` beside it (git-ignored); `took … nothing ran` when nothing changed, else `rebuilt:` names what did; `--force` runs every kernel call again |
| `./cadtool gen assemblies/arm.py` | build `assemblies/arm.step` (git-ignored); every part runs with it (replayed when unchanged) and writes its own STEP |
| `./cadtool python tools/bom.py [--module <module>] [--md\|--json]` | the **print list** and the **buy list**, of the arm or of one module of `assemblies/arm.py MODULES` ([`parts/AGENTS.md`](parts/AGENTS.md) Printed vs. bought) |
| `./cadtool python tools/export_printables.py [--parts …]` | one STL per **printed** part into `print/` (git-ignored) |
| `./cadtool export <file.step> stl\|3mf\|glb [out]` | one mesh file per call from a STEP document (default: beside it) |
| `./cadtool inspect <file.step> [--planes] [--json]` | every leaf's ref, solids, faces, volume, bbox of a saved STEP (`--planes`: its planar faces) |
| `./cadtool cad step inspect <file.step> "<ref>"` | what one ref (an occurrence path, `…/face 7`) names, and its measures |
| `./cadtool inspect diff <a.step> <b.step> [--tol X]` | same geometry, leaf by leaf, as a copy kept from before a change? |
| `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly` | a PNG still for review — also `.urdf`/`.sdf`/`.stl` inputs |
| `./cadtool viewer [file…]` | the CAD Viewer: opens each file's page on the machine's server (default `assemblies/arm.step`; the link is good for a day) |
| `./cadtool status` / `./cadtool stop` | the machine's cad server: what it runs and holds, for every session / end this venv's idle workers (after a `pyproject.toml` change, when leaving a worktree); `stop --device` stops the server for every session |
| `./cadtool server` | the machine's `cad-server` at this project's pin, telemetry and automatic updates stored off first (`setup` runs it; a linked worktree only checks) |
| `./cadtool validate robot/arm.urdf --strict` (`.srdf`, `.sdf --gz-check never`) | robot-description validators |
| `./cadtool cad stl dfm print/<name>.stl --process printing` | printability of a printed part's STL: walls, overhangs, support ([`parts/AGENTS.md`](parts/AGENTS.md) Printed vs. bought) |
| `./cadtool skill <skill> <tool> [args]` | a plugin skill script (`print3d orca_presets.py`), its `requirements.txt`, if any, fetched into uv's cache |
| `./cadtool cad …` | any `cad` subcommand but `setup`, `update`, `uninstall`, `doctor`, `run` ([`docs/toolchain.md`](docs/toolchain.md) Gotchas) |
| `./cadtool doctor` | the venv's, the server's and the plugin's release against the pin, the stored telemetry and automatic-update choices, the OCP kernel; changes nothing |
| `./cadtool pytest [-m "not slow"] [-n 4]` | test suite (the fast lane skips geometry builds; `-n`: worker processes, a whole test file each) |
| `./cadtool lint [--fix] [path…]` | `ruff check` over `cad/` |
| `./cadtool python …` | any python in the venv with `PYTHONPATH=cad/` (`-c "from assemblies.cycloidal_drive import totals; print(totals())"`) |
| `./cadtool clean [--all]` | delete `__pycache__/`, `.pytest_cache/`, `.ruff_cache/` (and any stray `__cadgen__/`); `--all` also empties `snapshots/` and `print/` and removes the git-ignored `assemblies/*.step`, `robot/links/*.step` |

`cadtool` always `cd`s to `cad/` (cad resolves paths from the working directory) and exports
`PYTHONPATH=cad/` so `lib`, `parts`, `assemblies`, `robot` import the same way for you and for the
server's workers. Everything derived — each model's document as last built, tessellations — lives in the
server and its cache `~/.cache/cad/documents` (`./cadtool cad delete <id> --yes` removes one; a model's
next run builds it again).


## Layout

```
cad/
├── cadtool       # the one entry point (above)
├── .env          # PYTHONPATH=. for the VS Code Python extension (cadtool / pytest set it themselves)
├── AGENTS.md     # the agent map + the rules for every folder; the folders below carry their own
├── lib/          # shared dimensions (params.py, the single source of truth) + geometry code, a package per converted subsystem
├── parts/        # one part model per file, grouped by the stage along the arm (+ _templates/)
├── assemblies/   # the arm, the gripper and the drive modules (arm.py MODULES)
├── robot/        # URDF (source of truth) / SRDF / SDF, per-link models, per-link meshes (committed)
├── reference/    # immutable reference STEPs (solidworks/, cycloidal/, native/), manifest.json, placements.json
├── vendor/       # purchased-part STEPs, replaceable by better catalog models
├── tools/        # import / extract / BOM / print export / inspect scripts (./cadtool python tools/…)
├── tests/        # pytest (the fast lane, the geometry tests, the per-subsystem suites)
├── docs/         # the drives' specs, the open-issues list, the toolchain notes
├── print/        # git-ignored: one STL per printed part (tools/export_printables.py)
└── snapshots/    # git-ignored: snapshot PNGs
```
Every folder with rules has its own guide — written for agents, readable by anyone:
[`lib/`](lib/AGENTS.md) · [`parts/`](parts/AGENTS.md) (part conventions, converting a part, purchased parts, printed
vs. bought) · [`assemblies/`](assemblies/AGENTS.md) · [`robot/`](robot/AGENTS.md) (joints, links, actuators) ·
[`tests/`](tests/AGENTS.md) · [`tools/`](tools/AGENTS.md) · [`reference/`](reference/AGENTS.md) (what is there:
[`README`](reference/README.md)) · [`vendor/`](vendor/AGENTS.md) (what is there: [`README`](vendor/README.md)). The drives:
[`docs/cycloidal_drive.md`](docs/cycloidal_drive.md), [`docs/forearm_roll.md`](docs/forearm_roll.md),
[`docs/wrist_roll.md`](docs/wrist_roll.md); what is not settled yet: [`docs/open_issues.md`](docs/open_issues.md).

## Tests

```bash
./cadtool pytest                 # everything (the geometry builds too; never writes a STEP)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON, tooling (the pinned cad + one complete OCP kernel)
./cadtool pytest tests/cycloidal # the cycloidal drive's tests only (one folder per subsystem: tests/AGENTS.md)
```

## Credits
The gripper's base, racks, jaws and pinion (`parts/gripper/gripper_base.py`, `gripper_rack.py`, `gripper_jaw.py`,
`gripper_pinion.py`, `lib/gripper/`) are modelled on "Rack & Pinion Robotic Gripper Jaw" by PapaBravo (Printables,
https://www.printables.com - licensed CC BY 4.0, https://creativecommons.org/licenses/by/4.0/), measured from its STEP
files; the cage that holds it on the wrist is designed here.

## AI CAD assistance

The `text-to-cad@earthtojake` plugin's `/text-to-cad:*` skills drive the run-the-model → inspect → snapshot loop
this repo is aligned with (plus `robot-simulation` for the URDF/SRDF/SDF and `print3d` for slicing; printability is
`cad stl dfm`); they run the `cad` command — inside this project, `./cadtool …`, which builds on this venv and never runs
`cad run` or the installer they mention ([`AGENTS.md`](AGENTS.md) Running things).
The plugin also starts CAD's MCP server (`cad`) with every Claude Code session: asked to show a model, Claude
answers with a link that opens it in the CAD Viewer (in Claude Desktop, a viewer card in the chat). It shows saved
files (STEP, URDF, STL …) on the machine's one cad server, and sends no telemetry and installs no update by itself
once `./cadtool setup` has stored cad's telemetry and automatic updates off ([`docs/toolchain.md`](docs/toolchain.md) Pins); `/mcp` turns the server off.
Agent-facing conventions live in `AGENTS.md` and each folder's own `AGENTS.md`.
