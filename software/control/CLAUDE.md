# software/control/ — the motor-control CLI

Loads when you work in `software/control/`. The repo-wide rules (git, uv, ruff and the hooks, CI, docs) are the root
`CLAUDE.md`; setup, configuring the motors and running the CLI are `README.md`. The software is picked up after the
CAD: until then it stays as it is.

## Running
- Run the CLI with `uv run launcher` from `software/control/` (entry point defined in `[project.scripts]`). After
  adding or renaming entry points, run `uv sync` to refresh `.venv/bin/`.
- Lint with `uv run ruff check` here; it is clean — keep it clean.
- Never run CAD code with this venv: `cad/` is its own uv project.

## Code layout
- `launcher.py` auto-detects the CANable via `pyserial.tools.list_ports` and calls `motor_control.main()`.
- `motor_control.py` is the interactive CLI. Near the top it does `sys.path.insert(0, "src")` so
  `src/can_interface.py`, `src/motor_driver.py`, `src/config.py` import without an `arctos.` prefix. **Don't** convert
  `src/` into a real Python package without also rewriting those imports — the sys.path hack is load-bearing. `"src"`
  is relative to the current directory, so the CLI runs from `software/control/` (`launch.bat` changes into its own
  folder first).
- Joint table and CAN settings live in `src/config.py` (`JOINTS`, `CAN_CHANNEL`, etc). What they must match — the
  joints, the reductions and which way round `gear_ratio` reads, which motor sits on which joint — is
  `cad/robot/CLAUDE.md`; which CAN id drives which joint is unconfirmed (`cad/docs/open_issues.md`).

## Known issues
- `tests/` still imports `from arctos.*` and is broken, and `pytest` is not in the dev group; the CLI runs fine
  without it. CI leaves these tests out (root `CLAUDE.md` "CI").
