# CAD — build123d models (robotic arm)

**Purpose:** the map of `cad/` and the rules that hold in every folder — running things, the regeneration
checklist (Recipe C), layering, the lazy kernel, the two machines. **Audience:** agent; human docs: `README.md`. Each
folder's own rules live in its CLAUDE.md — `parts/`, `assemblies/`, `robot/`, `lib/`, `tests/`, `tools/`, `reference/`,
`vendor/` — which loads when you work there; Start here says which file owns which task.

The root `CLAUDE.md` loads with this file, so its rules (git, uv, ruff and the hooks, CI, docs) are not repeated here.
The toolchain, its pins and upgrades: `docs/toolchain.md`.

## Start here — task → where to look
| you want to … | read | then follow |
|---|---|---|
| run / build / inspect / snapshot anything | Running things (below) | — |
| add or convert a printed part | `parts/CLAUDE.md` | Recipe C after the geometry |
| add a purchased part | `parts/CLAUDE.md` Purchased (COTS) parts | Recipe A (there) |
| add a part designed HERE (no SolidWorks / CadQuery origin), or a purchased part with no model at all | `parts/CLAUDE.md` Part states → *native* | `tools/reference/import_native.py` once, then Recipe C |
| place something the SolidWorks capture never had (a motor, a board) | `assemblies/CLAUDE.md` → Mounted occurrences | Recipe B (there) |
| produce or swap a vendor STEP | `vendor/CLAUDE.md` | Recipe D (there) |
| a new SolidWorks / vendor export was handed over | `reference/CLAUDE.md` | Recipe E (there) |
| run or change a tool (bom, exporters, the reference importers) | `tools/CLAUDE.md` | its recipe |
| changed any geometry, mass or placement | — | **Recipe C** (below, the regeneration checklist) |
| change a shared dimension | `lib/CLAUDE.md` Shared dimensions (its own table) | Recipe C |
| touch the URDF / links / inertials | `robot/CLAUDE.md` | Recipe C steps 7–9 |
| print list, buy list, STLs to print | `parts/CLAUDE.md` Printed vs. bought | — |
| bump cadgen / build123d / OCP / Python, or a pull changed `pyproject.toml` / `uv.lock` | `docs/toolchain.md` | — |
| work on the cycloidal drive / the forearm roll drive | `docs/cycloidal_drive.md` / `docs/forearm_roll.md` | — |
| know what is unsettled (fit problems, estimates, unmodelled hardware) | `docs/open_issues.md` | add / remove rows as you go |
| something behaves oddly | Gotchas (below), then the Gotchas of the folder's CLAUDE.md and `docs/toolchain.md` | — |

## Recipe C — the regeneration checklist, after ANY geometry / mass / placement change, in this order
1. `./cadtool daemon stop` when `lib/reference.py`, `pyproject.toml` or the kernel changed (workers keep old code).
2. `shasum -a 256 parts/*/*.step assemblies/*.step robot/links/*.step > /tmp/before.txt` (the hash gate).
3. `./cadtool gen assemblies/arm.py` (every stale child rebuilds);
   a part whose vendor file is NEW needs `./cadtool gen parts/<group>/<name>.py --force` first (see Gotchas below).
4. `shasum -a 256 -c /tmp/before.txt` on the SAME machine (STEP bytes differ per machine: Two machines, below) —
   only the STEPs you meant to change may fail; a file can still come back with numerical-zero terms (≤ 1e-17, seen
   after a `lib/reference.py` edit): settle float noise with `./cadtool inspect diff <old.step> <new.step>`.
5. `./cadtool pytest -m "not slow"`, then `./cadtool pytest`; bump the locks the failures name (`tests/CLAUDE.md`
   "Where the locks live"; a drive's `EXPECTED` from
   `./cadtool python -c "from assemblies.cycloidal_drive import totals; print(totals(), totals('stator'))"`).
6. A mounted part's geometry changed → `./cadtool python tools/reference/mount_placements.py` (its records carry
   solids / volume / bbox); a SolidWorks-derived change → `extract_placements.py --no-pancake`.
7. `./cadtool python tools/robot/derive.py --check robot/arm.urdf robot/arm.sdf`; for every link it names, copy that
   link's `<inertial>` block from `--urdf-draft` / `--sdf-draft` into `robot/arm.urdf` / `arm.sdf` (never a generator).
8. `./cadtool python tools/robot/export_link_meshes.py --links <those links>` and `./cadtool gen robot/links/<link>.py`
   (a mesh left stale fails `tests/test_robot.py`'s slow lane).
9. `./cadtool validate robot/arm.urdf --strict` (and `arm.srdf --strict`, `arm.sdf --gz-check never`); `--check` again.
10. `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly --view-labels` (+ the drive,
    `robot/arm.urdf`) and LOOK at them.
11. Docs: the doc that owns the changed rule (Docs, below), `docs/open_issues.md`
    rows added / closed. Then commit on the branch, the message being the record — why, and every number that
    changed (root `CLAUDE.md` Git workflow).

## Running things (always via `./cadtool` or `uv run`, from `cad/`)
Every command, its flags and what it prints: `./cadtool help` (for people: the `README.md` table). The rules:
- **Never call bare `python`** — the system Python is 3.14 without build123d.
- **Never add `cadquery-ocp`** (the VTK build) to `pyproject.toml`: it owns the same `OCP/` files as
  `cadquery-ocp-novtk`, so uv removing one guts the other (`docs/toolchain.md` Gotchas). The `cadgen` package is the
  whole runtime — decorators, CLI, viewer, snapshots; the `cad@text-to-cad` plugin only ships the `/cad:*` skill docs.
- **A model is a script you run.** `./cadtool gen <model.py>` writes its sibling STEP (git-ignored); a second run
  prints `current …` — the freshness gate hashes the source closure, the tracked inputs and the outputs (`./cadtool
  why` explains a verdict clause by clause). Every derived artefact lives in the store `~/.cache/cadgen` — nothing in
  the tree.
- **An assembly calls its child models**: `./cadtool gen assemblies/arm.py` rebuilds every stale part in parallel and
  rewrites its STEP; the gripper, the drives and the robot links link their children's trees, the arm inlines tinted
  copies (`assemblies/CLAUDE.md`). Pull semantics: a rebuilt part does not update the arm until the arm is rebuilt
  (`why` shows the pinned child).
- **Look at what you built**: hand every created / updated STEP to `./cadtool viewer`; snapshot review
  (`./cadtool snapshot …`) is mandatory after visible geometry changes. No `ocp-vscode`.
- `./cadtool inspect` is a local tool (`tools/step_facts.py`). Anything it doesn't answer is Python over
  `cadgen.read_scene(path)` (`.leaves()`, `.resolve("#o1.2.f7").shape()` — world-frame build123d geometry, the refs the
  viewer shows) + `cadgen.geometry` (`closest_points`, `overlap_volume`, `topology_errors`), kept as a test when it is
  worth re-running.
- Run pytest only through `./cadtool pytest` (rootdir `cad/`; `software/control/tests/` is the unrelated motor-control
  suite); the suite: `tests/CLAUDE.md`.
- A lint fix in a model's import closure makes the model stale like any source edit — rebuild and hash-gate it
  (Recipe C 2–4); fix the hook's findings before the rebuild, not after. The lint rules: root `CLAUDE.md` Toolchain.

## Rules for every folder
- A part's NAME (its module stem, unique across groups) is its key everywhere — the manifest,
  `reference/<origin>/<name>.step`, `placements.json`, the URDF links. Code reaches a part only through
  `parts.names()` / `parts.load(name)` / `parts.model(name)` / `parts.build(name)` — never `from parts import <name>`
  (`parts/CLAUDE.md`).
- Every model file — each part, each assembly, each `robot/links/<link>.py` — ends with
  `if __name__ == "__main__": <name>()` — that call IS the build (`./cadtool gen` runs the file; without it `gen`
  silently builds nothing — source-checked for every part and every `robot/links/<link>.py`); no `show()` in the
  file, no import side effects.
- Import `lib` / `parts` plainly — **no `sys.path` shim**: `cadtool` exports `PYTHONPATH=cad/`, pytest
  has `pythonpath = ["."]`, `.env` covers VS Code. **Never add `cad/__init__.py`** (the package root would become the
  repo root). The one `sys.path` line in the tree is `tools/cycloidal/export_cadquery.py` (it runs in the OTHER repo's
  venv).

### Layering
Locked by `tests/test_layering.py` (an AST scan — function-local imports count):
`lib ← parts ← assemblies ← robot ← tools ← tests`; a package imports only itself and the ones to its
left. So `assemblies/` never imports `robot/` (the base frame both need is `lib/datum.py`), `robot/`
builds on `assemblies/_occurrences.py` (`world_rows` reads a designed module's `OCCURRENCES` / `BODIES`),
and inside `lib/` the packages `lib/params.py` re-exports from never import it (`lib/CLAUDE.md`).

### Lazy kernel
**Model files never load the CAD kernel at import** (`tests/test_lazy_kernel.py`, fast lane): cadgen gates a
model before paying for OCP, so an unchanged part re-runs in ~0.1 s (1.5 s with an eager import, which also
prints cadgen's "kernel was imported before …" hint — that hint now means a regression).
**Keep the kernel lazy** in every model and its import closure: `from cadgen import build123d as bd` (a PEP 562 proxy — never
`from build123d import …`, never `from cadgen.build123d import X`, both import it) and use `bd.<name>`
**inside function bodies only**. No kernel object in a module-level constant, a class body, a decorator or
an argument default (`align=bd.Align.MIN` as a default is eager — `lib/geom.align_min()` is a
function for that reason); kernel types in annotations need `from __future__ import annotations`. The rule
covers the model's whole import closure (`lib/`, `assemblies/_occurrences.py`, `robot/`). Frames a module
declares are **data** — `(position mm, rotation_xyz_deg)`, `lib.datum.IDENTITY` for none — turned into a
`Location` by `lib.datum.to_location()` inside a body; `lib.datum.base_frame()` and
`assemblies.arm.arm_from_w()` are functions, the drive's `OCCURRENCES` rows carry positions.

### Never call a model outside a build
Never call a model to get its geometry outside a build: with no cadgen build on the thread the
call runs the whole pipeline (gate, writes the STEP, talks to the warm daemon). `parts.build(name)` /
`lib.models.raw(model)` run the body in-process (tests, tools, previews); `lib.models.geometry(model)`
is what assembly bodies call for a child — the linked child while a build runs, the raw body
otherwise. `tests/conftest.py` makes an accidental top-level call under pytest fail loudly.

## Two machines (Fedora Linux PC + arm64 Mac)
Push before leaving a machine, pull on arrival — git is the only sync channel (root `CLAUDE.md` "Two development
machines"; CI is the third machine: root `CLAUDE.md` "CI").
- **Per-machine state git does not carry** — after a pull that changes `pyproject.toml` / `uv.lock`, follow
  `docs/toolchain.md`.
- **What is committed:** only the inputs — `reference/**/*.step`, `vendor/*.step` — and `robot/meshes/*.stl`, all
  Git LFS objects (`/.gitattributes`); a checkout showing ~130-byte pointer files (checksum tests,
  `read_step` / `import_step` and cadgen fail on them) needs `git lfs pull`. **No generated STEP is committed**, part
  STEPs included (the root `.gitignore`'s `*.step`: `git check-ignore -v cad/parts/<group>/<name>.step` from the repo
  root must print that rule): cadgen writes deterministic bytes per kernel AND per machine — the other machine writes
  the same geometry with other float noise (≤ 2e-10 mm), and the STEP header names the OCCT version — so each machine
  builds its own; a fresh clone has none until `./cadtool gen assemblies/arm.py`. Nothing else lands in the tree: no
  `__cadgen__/`, no sidecar unless a model declares `kinematics=` (then `<name>.step.json`, committed beside it).
- **Written once, on one machine:** the committed files a tool generates — `robot/meshes/*.stl` (re-exported per
  change, never as a side effect), `vendor/*.step` (`vendor/CLAUDE.md`), `reference/native/` (`reference/CLAUDE.md`).
- `cadtool` runs on bash 3.2 (root `CLAUDE.md`): under `set -u` an empty array is unset — expand optional arrays as
  `${arr[@]+"${arr[@]}"}`. No exact-equality asserts on floating-point results (tessellations of spline geometry
  differ per architecture). Changes must work on both machines by construction; there is no per-machine sign-off to
  track or report.

## Gotchas (all verified)
- cadgen's freshness gate tracks only the inputs the LAST build actually read: a part whose vendor file appears after
  its last build still reads `current` (its body never called `read_step` on that file) — `./cadtool gen <part> --force`
  once; the assemblies then follow. Verified: `nema17_48mm` kept its envelope STEP until forced.
- Don't compare large STEP artifacts with `git diff`; compare source, `inspect` output and snapshots.
  A STEP edited by anything but its model (or built under another `CADGEN_CACHE_DIR`) reads as stale
  in `./cadtool why` — rebuild it.
- A model run accepts only `--force --mesh-tolerance --mesh-angular-tolerance --verbose --json` (`./cadtool gen
  <model.py> --help` lists them); anything else (`--totals`, a preview flag) is an argparse error — use
  `./cadtool python -c`.

## Docs
- A rule lives in the ONE CLAUDE.md / README of the folder it governs; a rule that spans folders lives here. Change it
  there — never restate it in a second file (point at it instead).
- Docs name constants, never numbers, and point at code; they never list inventories (parts, counts) or quote totals
  (leaves / solids / bought pieces / pinned children) — `./cadtool help`, `parts.names()`, the tests and the locks
  (`tests/CLAUDE.md` "Where the locks live") are those lists; their history is `git log -p` on the lock files.
- `docs/open_issues.md`, the commit message as the record, no dated history: root `CLAUDE.md` Docs / Git workflow.
