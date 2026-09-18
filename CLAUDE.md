# robotic-arm

3-motor MKS SERVO42D/57D arm controlled over CAN bus via a CANable / slcan-compatible USB adapter. Runs on macOS, Linux, and Windows.

**Last updated:** 2026-09-18 — see `CHANGELOG.md` for dated changes.

## Toolchain
- Always use `uv` — never `pip install` directly. `uv add <pkg>` for new deps;
  commit `pyproject.toml` and `uv.lock`.
- Run the CLI with `uv run launcher` (entry point defined in `[project.scripts]`).
  After adding or renaming entry points, run `uv sync` to refresh `.venv/bin/`.

## Code layout
- `launcher.py` auto-detects the CANable via `pyserial.tools.list_ports` and calls
  `motor_control.main()`.
- `motor_control.py` is the interactive CLI. Near the top it does
  `sys.path.insert(0, "src")` so `src/can_interface.py`, `src/motor_driver.py`,
  `src/config.py` import without an `arctos.` prefix. **Don't** convert `src/`
  into a real Python package without also rewriting those imports — the sys.path
  hack is load-bearing.
- Joint table and CAN settings live in `src/config.py` (`JOINTS`, `CAN_CHANNEL`, etc).

## Known issues
- `tests/` still imports `from arctos.*` and is broken. CLI runs fine without it.

## CAD (`cad/`)
- `cad/` is a **separate uv project** (Python 3.12, build123d) — the motor-control
  project above never depends on it, and `launch.bat` never installs it. Never run
  CAD code with the root venv.
- Work from `cad/` via `./cadtool …` (`setup|doctor|gen|step|show|why|inspect|snapshot|export|validate|viewer|parts|skill|cadgen|store|daemon|pytest|python|clean`; `step` = `gen`);
  it runs the `cadgen` 0.6 toolchain (the `cad@text-to-cad` plugin v0.6.x's PyPI runtime, locked in
  `cad/pyproject.toml`) inside the CAD venv with `PYTHONPATH=cad/`. A model is a plain script with one
  `@step def <name>()`; `./cadtool gen <model.py>` runs it. Conventions, the wrapper → parametric
  conversion workflow and the reference-match tests: `cad/CLAUDE.md`.
- The 20:1 cycloidal shoulder drive was imported from the `cycloidal_drive` repo (history kept
  via a subtree merge) and ported to build123d: `cad/lib/cycloidal/`, `cad/parts/cycloidal/`,
  `cad/assemblies/cycloidal_drive.py`; spec + port notes in `cad/docs/cycloidal_drive.md`.
- Parts are grouped by subsystem (`cad/parts/{base,joints,wrist,gripper,cycloidal}/`) and reached
  only through `parts.load(name)`; references sit in `cad/reference/{solidworks,cycloidal}/`; the
  committed STEP/STL files are Git LFS objects (`git lfs pull` if a checkout shows pointer files).
- The robot description (`cad/robot/`: `arm.urdf` is the source of truth, `arm.srdf`, `arm.sdf`, per-link
  meshes) is derived from `cad/robot/frames.py` — links `base_link, shoulder_link, upper_arm_link,
  forearm_link, wrist_pitch_link, wrist_roll_link, jaw_a_link, jaw_b_link, tool0`; joints `base_yaw,
  shoulder_pitch` (the cycloidal drive: stator in `shoulder_link`, rotor in `upper_arm_link`),
  `elbow_pitch, wrist_pitch, wrist_roll, jaw_a, jaw_b`. Which MKS motor (`src/config.py` J1..J3, all
  `gear_ratio` 1.0) drives which joint is unconfirmed — `CYCLOIDAL_RATIO` = 20 applies to `shoulder_pitch`.

## Docs
- `CHANGELOG.md` is the dated record of changes: add an entry (date, what changed, commit) with
  every commit that changes behaviour, layout or tooling, and bump the `Last updated` line of any
  README/CLAUDE.md you touch.
