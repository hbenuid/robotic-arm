# robotic-arm — CAD (build123d)

Parametric CAD-as-code for the desktop arm (base yaw, 21:1 cycloidal shoulder pitch, belt-driven
elbow pitch, forearm roll and wrist pitch, wrist roll, MG996R parallel gripper), converted part-by-part from the original
SolidWorks design. This folder is a **separate uv project** (Python 3.12) — the motor-control
software (`software/control/`) never depends on it.

**Status:** every SolidWorks custom part exists as an *import wrapper* around its reference
geometry (`reference/solidworks/<name>.step`) until it is converted, purchased parts use their vendor STEPs, and
`assemblies/arm.py` places all of them from placements extracted from the SolidWorks
assembly — so the whole arm already assembles, renders and is tested. Converting a part means
replacing its wrapper body with real build123d code (see [`parts/AGENTS.md`](parts/AGENTS.md) "Converting a part"). The **21:1
cycloidal shoulder drive is fully parametric build123d** (`lib/cycloidal/`,
`assemblies/cycloidal_drive.py`), ported from the `cycloidal_drive` repo and verified against its
CadQuery exports, its shell turning since (the yoke holds its hub and motor) — see
[`docs/cycloidal_drive.md`](docs/cycloidal_drive.md).

## Setup (once per machine)

Requirements: [`uv`](https://docs.astral.sh/uv/), `git-lfs` on `PATH` — every committed STEP/STL here (the inputs in
`reference/` and `vendor/`, plus `robot/meshes/`) is a Git LFS object, so `git lfs install` before cloning (a clone that
shows ~130-byte pointer files needs `git lfs pull`) — and Node 20+ (only for STL/3MF/GLB export). For Claude Code, the
[`text-to-cad@earthtojake`](https://github.com/earthtojake/text-to-cad) plugin **v0.7.x** (AI CAD assistance, below; the
repo's `.claude/settings.json` enables its marketplace) — install it once per scope, `--scope project` for the repo's own:

```bash
claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git
claude plugin install text-to-cad@earthtojake
cd cad
./cadtool setup                  # uv sync (build123d/OCP/cadgen into ./.venv; reinstalls the OCP kernel if it does not import)
                                 # + the git pre-commit hook (ruff on staged .py files) + git notes fetching
                                 # + Playwright Chromium (~150 MB, snapshots only)
./cadtool gen assemblies/arm.py  # build the arm (~35 s the first time): generated STEPs are git-ignored, each machine builds its own
./cadtool pytest                 # everything green?
```

Always run through `./cadtool …` (or `uv run …`) from `cad/`; `./cadtool doctor` checks the install. The pins
(Python, cadgen, the CAD kernel), updating the plugin and what a pull that changes `pyproject.toml` / `uv.lock` needs:
[`docs/toolchain.md`](docs/toolchain.md). CI (run by hand, a clean clone on a fresh runner): the root
[`AGENTS.md`](../AGENTS.md) "CI".

## `./cadtool` — the one entry point

| Command | What it does |
|---|---|
| `./cadtool gen parts/<group>/<name>.py` (alias `step`) | **run the model script**: writes `parts/<group>/<name>.step` beside it (git-ignored), or prints `current …` when nothing changed; `--force` rebuilds |
| `./cadtool gen assemblies/arm.py` | build `assemblies/arm.step` (git-ignored), rebuilding every stale part first |
| `./cadtool python tools/bom.py [--module cycloidal_drive\|forearm_roll_drive\|gripper] [--md\|--json]` | the **print list** and the **buy list** ([`parts/AGENTS.md`](parts/AGENTS.md) Printed vs. bought) |
| `./cadtool python tools/export_printables.py [--parts …]` | one STL per **printed** part into `print/` (git-ignored) |
| `./cadtool why <model.py>` | why the model is current or stale, clause by clause (`cadgen store why`) |
| `./cadtool export <file.step> stl\|3mf\|glb [out]` | one mesh file per call from a STEP document (Node 20+; `--mesh-tolerance` is *relative*, default 1.5e-3 of the bounding diagonal) |
| `./cadtool inspect <file.step> [--planes] [--json]` | leaf refs, solids, faces, volume, bbox of a saved STEP (`--planes`: its planar faces) |
| `./cadtool inspect diff <a.step> <b.step> [--tol X]` | same geometry, leaf by leaf, as a copy kept from before a change? |
| `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly --view-labels` | a PNG still for review — also `.urdf`/`.sdf`/`.stl` inputs |
| `./cadtool viewer [--port N]` | the CAD Viewer: `http://127.0.0.1:3245/?file=assemblies/arm.step` |
| `./cadtool validate robot/arm.urdf --strict` (`.srdf`, `.sdf --gz-check never`) | robot-description validators |
| `./cadtool parts "<query>" [--download --id <id> --filename <name>.step]` | step.parts search / download into `vendor/` |
| `./cadtool skill <skill> <tool> [args]` | a plugin skill script (`dfam-check dfam_tool.py`, `dfm mold_tool.py`, …), its `requirements.txt` fetched into uv's cache |
| `./cadtool cadgen …` / `store …` / `daemon …` | any `cadgen` subcommand; `./cadtool daemon stop` ends the warm build daemon |
| `./cadtool doctor` | installed cadgen vs the plugin's pin, Node, Playwright Chromium |
| `./cadtool pytest [-m "not slow"] [-n 4]` | test suite (the fast lane skips geometry builds; `-n`: worker processes, a whole test file each) |
| `./cadtool lint [--fix] [path…]` | `ruff check` over `cad/` |
| `./cadtool python …` | any python in the venv with `PYTHONPATH=cad/` (`-c "from assemblies.cycloidal_drive import totals; print(totals())"`) |
| `./cadtool clean [--all]` | delete `__pycache__/`, `.pytest_cache/`, `.ruff_cache/` (and any stray `__cadgen__/`); `--all` also empties `snapshots/` and removes the git-ignored `assemblies/*.step`, `robot/links/*.step` |

`cadtool` always `cd`s to `cad/` (cadgen resolves paths from the working directory and the viewer
serves it) and exports `PYTHONPATH=cad/` so `lib`, `parts`, `assemblies`, `robot` import the same way
for you, for cadgen's dependency scan and for its warm build daemon. Everything derived — trees,
tessellations, the freshness records — lives in `~/.cache/cadgen` (`./cadtool store gc` sweeps it;
deleting it is always safe). `CADGEN_DAEMON=0` runs a build on transient workers instead of the daemon.


## Layout

```
cad/
├── cadtool       # the one entry point (above)
├── .env          # PYTHONPATH=. for the VS Code Python extension (cadtool / pytest set it themselves)
├── AGENTS.md     # the agent map + the rules for every folder; the folders below carry their own
├── lib/          # shared dimensions (params.py, the single source of truth) + geometry code, a package per converted subsystem
├── parts/        # one part model per file, grouped by the stage along the arm (+ _templates/)
├── assemblies/   # the arm, the gripper and the two drive modules (cycloidal shoulder, forearm roll)
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
[`docs/cycloidal_drive.md`](docs/cycloidal_drive.md), [`docs/forearm_roll.md`](docs/forearm_roll.md); what is not
settled yet: [`docs/open_issues.md`](docs/open_issues.md).

## Tests

```bash
./cadtool pytest                 # everything (the geometry builds take ~1.5 min, under 1 min with -n 4; never writes a STEP)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON, tooling (the pinned cadgen + one complete OCP kernel)
./cadtool pytest tests/cycloidal # the cycloidal drive's tests only (tests/cycloidal/test_<part>.py + helpers.py)
uv run pytest                    # equivalent (cadgen is a normal dependency)
```

## AI CAD assistance

The `text-to-cad@earthtojake` plugin's `/text-to-cad:*` skills drive the run-the-model → inspect → snapshot loop
this repo is aligned with (plus `dfam-check` for printability, `step-parts`, and the URDF/SRDF/SDF
skills); they assume the `cadgen` CLI on `PATH` — inside this project that is `./cadtool cadgen …`.
The plugin also starts CAD's MCP server (`cad`) with every Claude Code session: asked to show a model, Claude
answers with a link that opens it in the CAD Viewer (in Claude Desktop, a viewer card in the chat). It shows saved
STEPs only, runs its own pinned cadgen (not this venv), and asks once before sending anonymous usage counts; `/mcp`
turns it off.
Agent-facing conventions live in `AGENTS.md` and each folder's own `AGENTS.md`.
