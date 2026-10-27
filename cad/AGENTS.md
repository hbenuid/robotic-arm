# CAD — build123d models (robotic arm)

**Purpose:** the map of `cad/` and the rules that hold in every folder — running things, the regeneration
checklist (Recipe C), layering, the lazy kernel, the two machines. **Audience:** agent; human docs: `README.md`. Each
folder's own rules live in its AGENTS.md — `parts/`, `assemblies/`, `robot/`, `lib/`, `tests/`, `tools/`, `reference/`,
`vendor/` — which loads when you work there; Start here says which file owns which task.

The root `AGENTS.md` loads with this file, so its rules (git, uv, ruff and the hooks, CI, docs) are not repeated here.
The toolchain, its pins and upgrades: `docs/toolchain.md`.

## Start here — task → where to look
| you want to … | read | then follow |
|---|---|---|
| run / build / inspect / snapshot anything | Running things (below) | — |
| add or convert a printed part | `parts/AGENTS.md` | Recipe C after the geometry |
| add a purchased part | `parts/AGENTS.md` Purchased (COTS) parts | Recipe A (there) |
| add a part designed HERE (no SolidWorks / CadQuery origin), or a purchased part with no model at all | `parts/AGENTS.md` Part states → *no reference* | its numbers in a test of its own, then Recipe C |
| place something the SolidWorks capture never had (a motor, a board) | `assemblies/AGENTS.md` → Mounted occurrences | Recipe B (there) |
| produce or swap a vendor STEP | `vendor/AGENTS.md` | Recipe D (there) |
| a new SolidWorks / vendor export was handed over | `reference/AGENTS.md` | Recipe E (there) |
| run or change a tool (bom, exporters, the reference importers) | `tools/AGENTS.md` | its recipe |
| changed any geometry, mass or placement | — | **Recipe C** (below, the regeneration checklist) |
| change a shared dimension | `lib/AGENTS.md` Shared dimensions (its own table) | Recipe C |
| touch the URDF / links / inertials | `robot/AGENTS.md` | Recipe C steps 7–9 |
| print list, buy list, STLs to print | `parts/AGENTS.md` Printed vs. bought | — |
| bump cadgen / build123d / OCP / Python, or a pull changed `pyproject.toml` / `uv.lock` | `docs/toolchain.md` | — |
| work on the cycloidal drive / the forearm roll drive / the wrist roll drive | `docs/cycloidal_drive.md` / `docs/forearm_roll.md` / `docs/wrist_roll.md` | — |
| make a design choice (how far to shrink a part, which fastener, what moves) | Design choices (below) | the preview first, then Recipe C |
| know what is unsettled (fit problems, estimates, unmodelled hardware) | `docs/open_issues.md` | add / remove rows as you go |
| something behaves oddly | Gotchas (below), then the Gotchas of the folder's AGENTS.md and `docs/toolchain.md` | — |

## Recipe C — the regeneration checklist, after ANY geometry / mass / placement change, in this order
1. `./cadtool stop` when `pyproject.toml` / `uv.lock` changed (the engine's workers keep the old packages; a source
   edit needs no restart: the next run names what changed).
2. `shasum -a 256 parts/*/*.step assemblies/*.step robot/links/*.step > /tmp/before.txt` (the hash gate); a fresh
   worktree has no STEPs: run it in the main checkout's `cad/` (same machine, so the same bytes).
3. `./cadtool gen assemblies/arm.py` (every child part runs with it - replayed where nothing it reaches changed - and
   writes its STEP);
   a part whose vendor file is NEW needs `./cadtool gen parts/<group>/<name>.py --force` first (see Gotchas below).
4. `shasum -a 256 -c /tmp/before.txt` on the SAME machine (STEP bytes differ per machine: Two machines, below) —
   only the STEPs you meant to change may fail; a file can still come back with numerical-zero terms (≤ 1e-17, seen
   after a `lib/reference.py` edit): settle float noise with `./cadtool inspect diff <old.step> <new.step>` (the old
   one from the main checkout's `cad/`, or a copy kept before step 3).
5. `./cadtool pytest -m "not slow"`, then `./cadtool pytest`; bump the locks the failures name (`tests/AGENTS.md`
   "Where the locks live"; a drive's `EXPECTED` (`assemblies/<drive>.py`) from
   `./cadtool python -c "from assemblies.<drive> import totals; print(totals(), totals('stator'))"`).
6. A mounted part's geometry changed → `./cadtool python tools/reference/mount_placements.py` (its records carry
   solids / volume / bbox); a SolidWorks-derived change →
   `./cadtool python tools/reference/extract_placements.py --monolith <full assembly> --no-pancake`.
7. `./cadtool python tools/robot/derive.py --check robot/arm.urdf robot/arm.sdf`; for every link it names, copy that
   link's `<inertial>` block from `--urdf-draft` / `--sdf-draft` into `robot/arm.urdf` / `arm.sdf` (never a generator).
8. `./cadtool python tools/robot/export_link_meshes.py --links <those links>` and `./cadtool gen robot/links/<link>.py`
   (a mesh left stale fails `tests/test_robot.py`'s slow lane).
9. `./cadtool validate robot/arm.urdf --strict` (and `arm.srdf --strict`, `arm.sdf --gz-check never`); `--check` again.
10. `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly` (+ each changed
    module's `assemblies/<module>.step`, `robot/arm.urdf`) and LOOK at them.
11. Docs: the doc that owns the changed rule (Docs, below), `docs/open_issues.md`
    rows added / closed. Then commit on the branch, the message being the record — why, and every number that
    changed (root `AGENTS.md` Git workflow).

## Running things (always via `./cadtool`, from `cad/`)
Every command, its flags and what it prints: `./cadtool help` (for people: the `README.md` table). The rules:
- **Never call bare `python`** — the system Python has no build123d (`./cadtool python` is the venv's).
- **Never add `cadquery-ocp`** (the VTK build) to `pyproject.toml` (why: `docs/toolchain.md` Gotchas). The `cadgen`
  package is the whole runtime — decorators, CLI, engine, viewer, snapshots; the `text-to-cad@earthtojake` plugin ships
  the `/text-to-cad:*` skill docs and CAD's MCP server `cad` (its own uvx-pinned cadgen: `docs/toolchain.md` Pins).
- **A model is a script you run.** `./cadtool gen <model.py>` hands it to this venv's warm cadgen engine
  (`docs/toolchain.md` Pins), which builds it and writes its sibling STEP (git-ignored; `@step(out="<name>.step")`); a
  second run prints `took … nothing ran` — the engine keys every body by the code it reaches and the files it reads,
  and replays what an edit did not reach; a run that rebuilt says what changed (`rebuilt: ../lib/assembly.py:
  label_text changed`). Everything else a build derives — documents, tessellations — lives in the engine and its
  cache `~/.cache/cadgen/documents`, never in the tree.
- **An assembly calls its child models**: `./cadtool gen assemblies/arm.py` runs every part with it (each one's body
  replayed when unchanged, its STEP written once a build) and writes its own STEP; a child's result reaches the parent
  as a copy the parent owns, which the arm re-tints (`assemblies/AGENTS.md`). Pull semantics: a rebuilt part does not
  update the arm until the arm is rebuilt.
- **Look at what you built**: show every created / updated STEP (or URDF) in the CAD Viewer — from a Claude session
  with the `cad` MCP server's `cad_show` (an absolute path; it answers with a link: share it once, the page follows
  the file), from a terminal with `./cadtool viewer <file>` (its link is good for a day). Snapshot review
  (`./cadtool snapshot …`) is mandatory
  after visible geometry changes; a change inside a housing (a bearing seat, a fit) also gets a cut, the cut faces
  capped — the XZ plane at Y = `<mm>` seen head on: `--display '{"clip":{"axis":"y","at":<mm>,"invert":true}}'
  --camera front` (`axis` `x` / `y` / `z`, `at` the plane's coordinate; without `invert` this camera sees the kept
  half's uncut outside) — and `--display '{"isolated": ["<ref>", …]}'` draws those occurrences alone,
  `{"hidden": [...]}` leaves them out (a ref is an occurrence path, `arm/wrist/gripper/mg996r_servo`: `./cadtool
  inspect` lists them). No `ocp-vscode`.
- `./cadtool inspect` is a local tool (`tools/step_facts.py`: every leaf's ref and measures, `diff`);
  `./cadtool cadgen inspect <file.step> "<ref>"` says what one ref names (the refs the viewer copies). Anything else is
  Python over the model's body (`parts.build(name)`, `lib.models.raw(model)`) or a saved STEP's tree
  (`cadgen.read_step(path)`: the instance names, each node's location in its parent's frame) + `cadgen.geometry`
  (`closest_points`, `overlap_volume`, `topology_errors`), kept as a test when it is worth re-running.
- Run pytest only through `./cadtool pytest` (rootdir `cad/`; `software/control/tests/` is the unrelated motor-control
  suite); the suite: `tests/AGENTS.md`.
- A lint fix in a model's import closure rebuilds the model like any source edit — rebuild and hash-gate it
  (Recipe C 2–4); fix the hook's findings before the rebuild, not after. The lint rules: root `AGENTS.md` Toolchain.

## Rules for every folder
- A part's NAME (its module stem, unique across groups) is its key everywhere — the manifest,
  `reference/<origin>/<name>.step`, `placements.json`, `robot/frames.py LINKS` (the URDF links' occurrences). Code
  reaches a part only through `parts.names()` / `parts.load(name)` / `parts.model(name)` / `parts.build(name)` — never
  `from parts import <name>` (`parts/AGENTS.md`).
- Every model file — each part, each assembly, each `robot/links/<link>.py` — ends with
  `if __name__ == "__main__": <name>()` — that call IS the build (`./cadtool gen` runs the file; without it `gen`
  silently builds nothing — source-checked for every part, every assembly and every `robot/links/<link>.py`); no `show()` in the
  file, no import side effects.
- **Belts**: every GT2 belt is a stock length (`lib/belts.py STANDARD_2GT_LENGTHS`: what the belts are bought as,
  never an odd length ordered on its own) and every belt gets a **tensioner** - an idler on its back taking up what the
  stock length has over its open loop (`idler_on_run()`), adjustable; so a belt joint's centre distance is a design
  value, not the belt's. A belt still without its tensioner is a `docs/open_issues.md` row.
- Import `lib` / `parts` plainly — **no `sys.path` shim**: `cadtool` exports `PYTHONPATH=cad/`, pytest
  has `pythonpath = ["."]`, `.env` covers VS Code. **Never add `cad/__init__.py`** (the package root would become the
  repo root). The one `sys.path` line in `cad/` is `tools/cycloidal/export_cadquery.py` (`tools/AGENTS.md`).

### Layering
Locked by `tests/test_layering.py` (an AST scan — function-local imports count):
`lib ← parts ← assemblies ← robot ← tools ← tests`; a package imports only itself and the ones to its
left. So `assemblies/` never imports `robot/` (the base frame both need is `lib/datum.py`), `robot/`
builds on `assemblies/_occurrences.py` (`world_rows` reads a designed module's `OCCURRENCES` / `BODIES`),
and inside `lib/` no package or leaf imports `lib/params.py` (`lib/AGENTS.md`).

### Lazy kernel
**Model files never load the CAD kernel at import** (`tests/test_lazy_kernel.py`, fast lane): a model run hands its
build to the engine, and a run the engine finds current never loads the kernel in the calling process — unless the
model's import closure does, and then every run pays the kernel's import.
**Keep the kernel lazy** in every model and its import closure: `import build123d as bd` (cadgen's import hook,
`cadgen-lazy-kernel.pth` in the venv, binds a module the kernel loads into on its first attribute — never
`from build123d import …`, which reads an attribute and loads it) and use `bd.<name>` **inside function bodies only**. No kernel object in a module-level constant, a class body, a decorator or
an argument default (`align=bd.Align.MIN` as a default is eager — `lib/geom.align_min()` is a
function for that reason); kernel types in annotations need `from __future__ import annotations`. The rule
covers the model's whole import closure (`lib/`, `assemblies/_occurrences.py`, `robot/`). Frames a module
declares are **data** — `(position mm, rotation_xyz_deg)`, `lib.datum.IDENTITY` for none — turned into a
`Location` by `lib.datum.to_location()` inside a body; `lib.datum.base_frame()` and
`assemblies.arm.arm_from_w()` are functions, the drives' `OCCURRENCES` rows carry positions.

### Never call a model outside a build
Never call a model to get its geometry outside a build: with no cadgen build on the thread the
call is a build (it starts this venv's engine and writes the STEP) and returns a document handle, not a shape.
`parts.build(name)` / `lib.models.raw(model)` run the body in-process (tests, tools, previews);
`lib.models.geometry(model)` is what assembly bodies call for a child — the child model's result (`model().shape`)
while a build runs, the raw body otherwise. `tests/conftest.py` makes an accidental top-level call under pytest fail
loudly.

### Design choices
A change that is a design choice — how far to shrink a part, which fastener, what moves — is the user's to make: ask,
with 2–4 concrete options, each with its measured numbers and explained simply (AskUserQuestion). Then SHOW the chosen
geometry before the branch is touched: build it from the part builders with the proposed config in a scratch script
(nothing in the repo) and publish a page with matched before / after renders and a to-scale overlay, close-ups where
the difference is subtle; the work starts once the user has seen it. Renders at one scale: Gotchas (`snapshot` fits
every view).

## Two machines (Fedora Linux PC + arm64 Mac)
Push before leaving a machine, pull on arrival — git is the only sync channel (root `AGENTS.md` "Two development
machines"; CI is the third machine: root `AGENTS.md` "CI").
- **Per-machine state git does not carry** — after a pull that changes `pyproject.toml` / `uv.lock`, follow
  `docs/toolchain.md`.
- **What is committed:** only the inputs — `reference/**/*.step`, `vendor/*.step` — and `robot/meshes/*.stl`, plain git
  files (older commits hold them as Git LFS pointers - `git log -S filter=lfs -- :/.gitattributes` finds where that
  ends - so checking one of those out to build it needs `git-lfs`). **No generated STEP is committed**, part
  STEPs included (the root `.gitignore`'s `*.step`: `git check-ignore -v cad/parts/<group>/<name>.step` from the repo
  root must print that rule - on the Mac, with `core.ignorecase=true`, its `*.STEP` twin): cadgen writes deterministic
  bytes per kernel AND per machine — the other machine writes the same geometry with other float noise (≤ 2e-10 mm),
  and the STEP header names the OCCT version — so each machine builds its own; a fresh clone has none until
  `./cadtool gen assemblies/arm.py`. Nothing else lands in the tree: no `__cadgen__/`, no sidecar.
- **Written once, on one machine:** the committed files a tool generates — `robot/meshes/*.stl` (re-exported per
  change, never as a side effect), `vendor/*.step` (`vendor/AGENTS.md`), `reference/native/` (`reference/AGENTS.md`).
- `cadtool` runs on bash 3.2 (root `AGENTS.md`): under `set -u` an empty array is unset — expand optional arrays as
  `${arr[@]+"${arr[@]}"}`. No exact-equality asserts on floating-point results (tessellations of spline geometry
  differ per architecture). Changes must work on both machines by construction; there is no per-machine sign-off to
  track or report.

## Gotchas (all verified)
- The engine keys a body by the files it READ, not the ones it only checked for: a part whose vendor file appears
  after its last build still reads `took … nothing ran` (its body never opened that file: `lib/cots.hybrid()` asks
  `exists()` first) — `./cadtool gen <part> --force` once (its next run says `changed`); the assemblies then follow.
  Verified on cadgen 0.8.0: `bearing_625` built with its vendor file set aside kept its envelope after the file came
  back, until forced.
- Don't compare large STEP artifacts with `git diff`; compare source, `inspect` output and snapshots. A STEP edited
  (or deleted) by anything but its model comes back on the model's next run, byte for byte (`wrote …`).
- A model run takes no options: `./cadtool gen` refuses any flag but its own `--force` (`CADGEN_ENGINE_MEMO=0`: every
  kernel call runs again); for anything else (`--totals`, a preview) use `./cadtool python -c`.
- cadgen writes equal parts as ONE STEP product named after the first of them (the yaw, elbow and roll idler screws
  are one product, `yaw_idler_screw:base_yaw`); each instance keeps its own name. Read a STEP's tree with
  `cadgen.read_step` (or `./cadtool inspect`), which gives the instance names - build123d's `import_step` gives the
  product's.
- `./cadtool snapshot` fits every view to its model: the `--camera` JSON's `orthographicHalfHeight` is ignored, whatever
  `--help` says (verified again on cadgen 0.8.0: `bearing_625` and `base` both fill the frame at
  `orthographicHalfHeight` 100). To render versions at one scale, give each the same bounding box — tiny marker cubes
  at fixed corners, in the scratch build — and crop.
- A snapshot taken right after a rebuild once drew the STEP's previous version (the same command a minute later drew
  the new one): when a render does not show a change you made, run it again before looking for the cause.

## Docs
- A rule lives in the ONE AGENTS.md / README of the folder it governs; a rule that spans folders lives here. Change it
  there — never restate it in a second file (point at it instead).
- Docs name constants, never numbers, and point at code; they never list inventories (parts, counts) or quote totals
  (leaves / solids / bought pieces / pinned children) — `./cadtool help`, `parts.names()`, the tests and the locks
  (`tests/AGENTS.md` "Where the locks live") are those lists; their history is `git log -p` on the lock files.
- `docs/open_issues.md`, the commit message as the record, no dated history: root `AGENTS.md` Docs / Git workflow.
