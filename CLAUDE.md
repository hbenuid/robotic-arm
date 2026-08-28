# robotic-arm

3-motor MKS SERVO42D/57D arm controlled over CAN bus via a CANable / slcan-compatible USB adapter. Runs on macOS, Linux, and Windows.

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

## Known issue
- `tests/` still imports `from arctos.*` and is broken. CLI runs fine without it.

## CAD (`cad/`)
- `cad/` is a **separate uv project** (Python 3.12, build123d) — the motor-control
  project above never depends on it, and `launch.bat` never installs it. Never run
  CAD code with the root venv.
- Work from `cad/` via `./cadtool …` (`setup|gen|export|inspect|snapshot|validate|parts|skill|viewer|pytest|python`);
  it runs the `cad@text-to-cad` plugin (v0.4.x) CLIs inside the CAD venv. Conventions, the
  wrapper → parametric conversion workflow and the reference-match tests: `cad/CLAUDE.md`.
- The 20:1 cycloidal shoulder drive was imported from the `cycloidal_drive` repo (history kept
  via a subtree merge) and ported to build123d: `cad/lib/cycloidal/`, `cad/parts/cycloidal_*.py`,
  `cad/assemblies/cycloidal_drive.py`; spec + port notes in `cad/docs/cycloidal_drive.md`.

## Docs
- `CHANGELOG.md` is the dated record of changes: add an entry (date, what changed, commit) with
  every commit that changes behaviour, layout or tooling, and bump the `Last updated` line of any
  README/CLAUDE.md you touch.
