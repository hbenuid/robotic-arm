# robotic-arm

3-motor MKS SERVO42D/57D arm controlled over CAN bus via a CANable / slcan-compatible USB adapter. Runs on macOS, Linux, and Windows.

**Last updated:** 2026-09-24 — see `CHANGELOG.md` for dated changes.

## Git workflow
- Work on a branch — `cad/<topic>` for CAD work, `<area>/<topic>` otherwise — and push the **branch**. `main` is
  fast-forwarded to it only when the user says so; never commit to or push `main` directly (an approved plan
  that says "commit + push" means the branch). Every commit that changes behaviour, layout or tooling gets a
  dated `CHANGELOG.md` entry naming the branch and the commit.
- Case matters: git and the `*.step` / `*.stl` rules are case-sensitive. `.gitignore` and `.gitattributes` also
  match `*.STEP` / `*.STP` now, but keep every file in the tree lowercase — the tests and tools assume it.

## Two development machines
Worked on from a **Fedora Linux PC and an arm64 Mac**; git is the only sync channel, venvs / plugin / caches are
per machine. In `cad/` no generated STEP is committed (its bytes differ per machine; each machine builds its
own) — rules in `cad/CLAUDE.md` "Two machines". Scripts must run on macOS's bash 3.2 without GNU coreutils.

## The SolidWorks inputs are in the repo; the raw exports are not
The CAD reads only committed files: `cad/reference/solidworks/*.step` (every part's SolidWorks export, renamed),
`cad/reference/cycloidal/*.step`, `cad/vendor/*.step`, `reference/placements.json` and `manifest.json` — all Git
LFS. The raw exports they were derived from (the full-assembly monolith, the per-part exports, the motor-kit
exports) are **not kept anywhere in git** and need not exist on a machine; they are only needed to re-run the
derivation tools (`cad/tools/reference/extract_placements.py --monolith …`, `import_solidworks.py --src …`,
`split_mks_motor.py --src …`; the default directory is `cad/lib/reference.py DEFAULT_SOURCE_DIR`, overridable
with `ARM_REFERENCE_SRC`). A new export the user hands over is fed to those tools from wherever it sits (lowercase
`.step`, outside the tree — never committed raw); its sha256 goes into `cad/reference/README.md`. Recipe E in
`cad/CLAUDE.md` "Start here".

## Toolchain
- Always use `uv` — never `pip install` directly. `uv add <pkg>` for new deps;
  commit `pyproject.toml` and `uv.lock`.
- Run the CLI with `uv run launcher` (entry point defined in `[project.scripts]`).
  After adding or renaming entry points, run `uv sync` to refresh `.venv/bin/`.
- Lint with ruff (a locked dev dependency in both uv projects, config in each `pyproject.toml` `[tool.ruff]`:
  ruff's default set + `E F I UP B SIM` in full, the ignores commented):
  `uv run ruff check` here (it skips `cad/`), `./cadtool lint` in `cad/`. Both are clean; keep them clean.
  Lint only — `ruff format` is not adopted (it would re-flow `cad/`'s hand-aligned tables).
  It runs by itself at three points: a Claude Code PostToolUse hook (`.claude/hooks/ruff-check.sh`) reports
  findings on every `.py` file Claude edits — fix them in the same turn. It is registered twice, in
  `.claude/settings.json` AND `cad/.claude/settings.json`: a session reads the shared settings file of the
  directory it starts in only (not inherited like CLAUDE.md), so keep the two hook blocks identical;
  the git pre-commit hook (`.githooks/pre-commit`, installed per machine by `./cadtool setup`) blocks a commit
  with findings in the staged files; VS Code fixes / sorts imports on save (workspace settings, Ruff extension).
  `F401` is never auto-fixed (`unfixable`): an import written before its first use must survive a save.

## CI (GitHub Actions)
`.github/workflows/ci.yml` runs **only by hand** (Actions tab → CI → Run workflow, or
`gh workflow run ci.yml --ref <branch>`; `gh run watch` follows it): a fresh `ubuntu-24.04` runner clones the repo,
pulls the LFS objects, lints both projects, imports the CLI, runs `cad/`'s fast lane and slow tests and `./cadtool
gen`s both arms (~13 min, from the private repo's monthly Actions minutes). It is the **clean-clone check** neither
machine can give — no `~/.cache/cadgen`, no raw exports, no generated STEPs — so a test or model that quietly needs
a file outside git fails there first. Everyday pushes do not need it (the pre-commit hook and the local suite cover
them); run it on the branch before fast-forwarding `main` after: a cadgen / build123d / OCP bump, a new SolidWorks
or vendor export or vendor STEP, a change to `lib/reference.py` or to how parts read their inputs, or any large
change. Suggest it then; never trigger it unasked. It checks and never writes (no hash gate, no snapshots, no
commits). The runner is a third machine (x86_64 Linux): the "no exact float equality" rule of `cad/CLAUDE.md` "Two
machines" holds for it too. The root `tests/` stay out until they are fixed.

## Code layout
- `launcher.py` auto-detects the CANable via `pyserial.tools.list_ports` and calls
  `motor_control.main()`.
- `motor_control.py` is the interactive CLI. Near the top it does
  `sys.path.insert(0, "src")` so `src/can_interface.py`, `src/motor_driver.py`,
  `src/config.py` import without an `arctos.` prefix. **Don't** convert `src/`
  into a real Python package without also rewriting those imports — the sys.path
  hack is load-bearing.
- Joint table and CAN settings live in `src/config.py` (`JOINTS`, `CAN_CHANNEL`, etc).
- `firmware/` holds all firmware, one subfolder per board. `firmware/stm32/` is the archived Nucleo-F446RE
  PlatformIO projects (a blink test and three SERVO42D CAN test programs) from before the Python CLI; each is a
  self-contained PlatformIO project, part of neither uv project.

## Known issues
- `tests/` still imports `from arctos.*` and is broken. CLI runs fine without it.

## CAD (`cad/`)
- `cad/` is a **separate uv project** (Python 3.12, build123d) — the motor-control
  project above never depends on it, and `launch.bat` never installs it. Never run
  CAD code with the root venv.
- Work from `cad/` via `./cadtool …` (`setup|doctor|gen|step|why|inspect|snapshot|export|validate|viewer|parts|skill|cadgen|store|daemon|pytest|lint|python|clean`; `step` = `gen`);
  it runs the `cadgen` 0.6 toolchain (the `cad@text-to-cad` plugin v0.6.x's PyPI runtime, locked in
  `cad/pyproject.toml`) inside the CAD venv with `PYTHONPATH=cad/`. A model is a plain script with one
  `@step def <name>()`; `./cadtool gen <model.py>` runs it. Conventions, the wrapper → parametric
  conversion workflow and the reference-match tests: `cad/CLAUDE.md`.
- Printed vs. bought is one label per part — `COTS = True` in the part module = bought, anything else = printed
  (`parts.bought(name)`); the print list, the buy list (`./cadtool python tools/bom.py`), the grey of purchased parts
  in `arm.step` / `gripper.step` / `cycloidal_drive.step` (one STEP per assembly, never a make/buy copy) and the STL export (`tools/export_printables.py` → git-ignored `cad/print/`) are all
  generated from it. Never sort parts into make/buy folders or keep a second list by hand.
- The 20:1 cycloidal shoulder drive was imported from the `cycloidal_drive` repo (history kept
  via a subtree merge) and ported to build123d: `cad/lib/cycloidal/`, `cad/parts/cycloidal/`,
  `cad/assemblies/cycloidal_drive.py`; spec + port notes in `cad/docs/cycloidal_drive.md`.
- Parts are grouped by subsystem (`cad/parts/{base,joints,wrist,gripper,cycloidal}/`) and reached
  only through `parts.load(name)`; references sit in `cad/reference/{solidworks,cycloidal}/`; the
  committed STEP/STL files (the inputs in `reference/` + `vendor/`, and `robot/meshes/`) are Git LFS objects
  (`git lfs pull` if a checkout shows pointer files) — part STEPs are generated and git-ignored.
- The robot description (`cad/robot/`: `arm.urdf` is the source of truth, `arm.srdf`, `arm.sdf`, per-link
  meshes) is derived from `cad/robot/frames.py` — links `base_link, shoulder_link, upper_arm_link,
  elbow_link, forearm_link, wrist_pitch_link, wrist_roll_link, jaw_a_link, jaw_b_link, tool0`; joints `base_yaw,
  shoulder_pitch` (the cycloidal drive: stator in `shoulder_link`, rotor in `upper_arm_link`),
  `elbow_pitch, forearm_roll` (the belt-driven roll drive, `cad/assemblies/forearm_roll_drive.py`: stator in
  `elbow_link` - its block is also the elbow's output flange, the SolidWorks `j3_coupler#1` is retired -, rotor - the
  hollow roll shaft - in `forearm_link`; `cad/docs/forearm_roll.md`), `wrist_pitch, wrist_roll, jaw_a, jaw_b`. Which MKS motor (`src/config.py` J1..J3, all `gear_ratio` 1.0) drives which joint is
  unconfirmed — `CYCLOIDAL_RATIO` = 20 applies to `shoulder_pitch`, `FOREARM_ROLL_RATIO` = 4.5 to `forearm_roll`, and the
  arm now carries five kits for three configured CAN ids. The motors themselves are placed: MKS SERVO42D kits on `base`
  (48 mm, `nema17_48mm#1`) / `j1_link` / `j2_link` (40 mm, `nema17_40mm#2..3`) with `mks_servo42d#1..3`
  (`cad/lib/mounts.py`), the drive's 48 mm kit and the roll drive's 40 mm kit (their boards are module rows).

## Docs
- `CHANGELOG.md` is the dated record of changes: add an entry (date, what changed, commit) with
  every commit that changes behaviour, layout or tooling, and bump the `Last updated` line of any
  README/CLAUDE.md you touch.
- `cad/docs/open_issues.md` is the ONE list of what is not settled (fit problems, estimates to confirm on
  hardware, unmodelled hardware, unconfirmed mappings): add a row when you flag something, remove it when you
  close it. `cad/CLAUDE.md` opens with a "Start here" task index and the regeneration checklist.
