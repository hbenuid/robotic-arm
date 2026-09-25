# robotic-arm — CAD (build123d)

Parametric CAD-as-code for the desktop arm (base yaw, 20:1 cycloidal shoulder pitch, belt-driven
elbow and wrist pitch, wrist roll, MG996R parallel gripper), converted part-by-part from the original
SolidWorks design. This folder is a **separate uv project** (Python 3.12) — the motor-control
software (`software/control/`) never depends on it.

**Status:** every SolidWorks custom part exists as an *import wrapper* around its reference
geometry (`reference/solidworks/<name>.step`) until it is converted (the two links are: `j1_link`,
`j2_link`), purchased parts use their vendor STEPs, and
`assemblies/arm.py` places all of them from placements extracted from the SolidWorks
assembly — so the whole arm already assembles, renders and is tested. Converting a part means
replacing its wrapper body with real build123d code (see [`parts/CLAUDE.md`](parts/CLAUDE.md) "Converting a part"). The **20:1
cycloidal shoulder drive is fully parametric build123d** (`lib/cycloidal/`,
`assemblies/cycloidal_drive.py`), ported from the `cycloidal_drive` repo and verified against its
CadQuery exports — see [`docs/cycloidal_drive.md`](docs/cycloidal_drive.md).

## Setup (once per machine)

Requirements: [`uv`](https://docs.astral.sh/uv/), Node 20+ on `PATH` (only for STL/3MF/GLB export),
`git-lfs` on `PATH` — the plugin marketplace uses LFS **and so does this folder**: every committed
STEP/STL (the inputs in `reference/` and `vendor/`, plus `robot/meshes/`) is a Git LFS object, so `git lfs
install` once per machine before cloning (a clone that shows ~130-byte pointer files needs `git lfs pull`).
Generated STEPs — each part's `parts/<group>/<name>.step` included — are **git-ignored**: their bytes differ
per machine, so every machine builds its own (`./cadtool gen assemblies/arm.py`, ~35 s the first time) — and
the [`cad@text-to-cad`](https://github.com/earthtojake/text-to-cad) Claude Code plugin **v0.6.x**
(the repo's `.claude/settings.json` enables its marketplace; install/update with
`claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git`,
`claude plugin install cad@text-to-cad`, later `claude plugin marketplace update text-to-cad &&
claude plugin update cad@text-to-cad` — once per scope, `--scope project` for the repo's own install). The plugin only carries the `/cad:*` skill docs; the toolchain
itself is the [`cadgen`](https://pypi.org/project/cadgen/) package installed into this venv.

```bash
cd cad
./cadtool setup        # uv sync (build123d/OCP/cadgen into ./.venv; reinstalls the OCP kernel if it does not import)
                       # + the git pre-commit hook (ruff on staged .py files) + Playwright Chromium (~150 MB, snapshots only)
./cadtool pytest       # everything green?
```

CI (GitHub Actions, `../.github/workflows/ci.yml`, run by hand: `gh workflow run ci.yml --ref <branch>`) runs the lint,
the whole suite and `./cadtool gen` of both arms in a clean clone on a fresh Ubuntu runner (~13 min); when it is worth
running: the root `CLAUDE.md` "CI".

Python is pinned to **3.12** in `.python-version` (the system Python is 3.14; the OCP
wheels are verified on 3.12 — bump deliberately, with the full suite). Never run bare `python` here —
always `./cadtool …` or `uv run …` from `cad/`.

`cadgen` (the text-to-cad runtime, on PyPI) is a **locked dependency** pinned to the installed
plugin version (`cadgen[snapshot]==…` in `pyproject.toml`; the plugin's `skills/cad/requirements.txt`
pins the same) — bump both together; `./cadtool doctor` checks the pair, the CAD kernel, Node and
Chromium. The pins, how to upgrade and what each bump broke: [`docs/toolchain.md`](docs/toolchain.md) and
`git log -- pyproject.toml` from `cad/` (older entries as git notes under each commit).

## `./cadtool` — the one entry point

| Command | What it does |
|---|---|
| `./cadtool gen parts/<group>/<name>.py` (alias `step`) | **run the model script**: writes `parts/<group>/<name>.step` beside it (git-ignored); a second run prints `current …` (freshness gate), `--force` rebuilds |
| `./cadtool gen assemblies/arm.py` | build `assemblies/arm.step` (git-ignored) — calls every part model, so each stale part is rebuilt (in parallel) and its STEP rewritten |
| `./cadtool python tools/bom.py [--module cycloidal_drive\|gripper] [--md\|--json]` | the **print list** and the **buy list** (what to order, pieces, mass), generated from the assembly tables + each part's `COTS` flag; ends with the purchased items that are not modelled (`EXTRAS`). No CAD kernel, instant |
| `./cadtool python tools/export_printables.py [--parts …]` | one STL per **printed** part into `print/` (git-ignored; mm, part-local frame) with the quantity to print; bought parts are refused |
| `./cadtool why <model.py>` | why the model is current or stale, clause by clause (`cadgen store why`) |
| `./cadtool export <file.step> stl\|3mf\|glb [out]` | one mesh file per call from a STEP document (Node 20+; `--mesh-tolerance` is *relative*, default 1.5e-3 of the bounding diagonal) |
| `./cadtool inspect <file.step> [--planes] [--json]` | leaf refs, solids, faces, volume, bbox of a saved STEP (`--planes`: its planar faces as normal / offset / area) — a **local** tool, `tools/step_facts.py`: cadgen 0.6.5 removed `cadgen step inspect`; distances and overlaps are `cadgen.geometry.closest_points` / `overlap_volume` in a test |
| `./cadtool inspect diff <a.step> <b.step> [--tol X]` | same geometry leaf by leaf? exit 1 if not (checking a regenerated STEP against a copy kept from before the change) |
| `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly --view-labels` | PNG review still (`--job job.json` for a multi-view packet; the path you name is the file written) — also `.urdf`/`.sdf`/`.stl` inputs |
| `./cadtool viewer [--port N]` | CAD Viewer serving this folder — `http://127.0.0.1:3245/?file=assemblies/arm.step` (ships inside cadgen; stops after 12 h or Ctrl+C; `./cadtool cadgen viewer list\|stop --port N`) |
| `./cadtool validate robot/arm.urdf --strict` (`.srdf`, `.sdf --gz-check never`) | robot-description validators |
| `./cadtool parts "<query>" [--download --id <id> --filename <name>.step]` | step.parts search / download into `vendor/` |
| `./cadtool skill <skill> <tool> [args]` | a plugin skill script (`dfam-check dfam_tool.py`, `gcode gcode_tool.py`, …) |
| `./cadtool cadgen …` / `store …` / `daemon …` | any `cadgen` subcommand (`store info\|gc`, `daemon status`, `doctor`); `./cadtool daemon stop` ends the warm build daemon + workers (cadgen has no stop verb) |
| `./cadtool doctor` | installed cadgen vs the plugin's pin, Node, Playwright Chromium |
| `./cadtool pytest [-m "not slow"]` | test suite (the fast lane skips geometry builds) |
| `./cadtool lint [--fix] [path…]` | `ruff check` over `cad/` (rules in `pyproject.toml [tool.ruff]`; lint only - no `ruff format`, the tables are hand-aligned) |
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
├── CLAUDE.md     # the agent map + the rules for every folder; the folders below carry their own
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
[`lib/`](lib/CLAUDE.md) · [`parts/`](parts/CLAUDE.md) (part conventions, converting a part, purchased parts, printed
vs. bought) · [`assemblies/`](assemblies/CLAUDE.md) · [`robot/`](robot/CLAUDE.md) (joints, links, actuators) ·
[`tests/`](tests/CLAUDE.md) · [`reference/`](reference/README.md) · [`vendor/`](vendor/README.md). The drives:
[`docs/cycloidal_drive.md`](docs/cycloidal_drive.md), [`docs/forearm_roll.md`](docs/forearm_roll.md); what is not
settled yet: [`docs/open_issues.md`](docs/open_issues.md).

## Tests

```bash
./cadtool pytest                 # everything (~460 tests; the geometry builds take ~2 min; never writes a STEP)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON, tooling (the pinned cadgen + one complete OCP kernel)
./cadtool pytest tests/cycloidal # the cycloidal drive's tests only (tests/cycloidal/test_<part>.py + helpers.py)
uv run pytest                    # equivalent (cadgen is a normal dependency)
```

## AI CAD assistance

The `cad@text-to-cad` plugin's `/cad:*` skills drive the run-the-model → inspect → snapshot loop
this repo is aligned with (plus `dfam-check` for printability, `step-parts`, and the URDF/SRDF/SDF
skills); they assume the `cadgen` CLI on `PATH` — inside this project that is `./cadtool cadgen …`.
Agent-facing conventions live in `CLAUDE.md` and each folder's own `CLAUDE.md`.
