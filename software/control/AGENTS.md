# software/control/ — the arm's motor control

Loads when you work in `software/control/`. The repo-wide rules (git, uv, ruff and the hooks, CI, docs) are the root
`AGENTS.md`; setup, configuring the boards and running the CLIs are `README.md`; the layers and the plan for the next
ones are `docs/architecture.md`; the MKS CAN protocol is `docs/mks_can_protocol.md`.

## Running
- `uv run pytest` and `uv run ruff check` from `software/control/`; both are clean, keep them clean. CI runs both.
- The CLIs: `uv run launcher` (finds the CANable), `uv run motor-control …`, `uv run motor-control --fake` (simulated
  boards), `uv run dashboard` (the browser dashboard; `--fake` for simulated boards that move in time). After adding
  or renaming an entry point (`[project.scripts]`), `uv sync`.
- The tests never reach hardware: they use python-can's virtual bus and `robotic_arm.fake` only.
  `dashboard --fake --no-browser --port <n>` serves the page on simulated boards (to drive it with a headless
  browser, say).
- Never run CAD code with this venv: `cad/` is its own uv project.

## Code layout
- `src/robotic_arm/` is the package (hatchling, `packages = ["src/robotic_arm"]`). It imports nothing outside
  itself but python-can and pyserial: other projects install it (`uv add --editable`), so keep it that way.
- Layers, each calling only those below it: `can_interface` → `motor_driver` → `joint` → `arm`; `config` holds the
  arm's constants, `fake` simulates boards. The CLIs (`motor_control`, `launcher`, `command_sweep`) and the
  dashboard (`dashboard/`) sit on top.
- The dashboard (`dashboard/`): `bus.py` is the only code there that touches the CAN bus, on its own thread (a
  CANInterface is not thread-safe), and it never blocks for long, so a stop goes out at once; it polls every board
  and hands every reply and tick to every motor's listener. `bench.py` (one motor) owns every check and safety
  rule; `rig.py` holds the motors on the page - which is active, STOP for all, moves together (every target checked
  before any is sent). The page (`static/`) is trusted with none of them: a move the page asks for is checked in
  `bench.check` (the limits, the speed cap), and the jog keep-alive, the page-silence stop and the run's limit stop
  run on the bus thread's tick. Limits are kept in encoder counts (`Limits.count_lo` / `count_hi`), not in the page's
  units. A shaped (S-curve) move's `shaping.Shaper` sends only under the bench's lock after checking the shaping
  generation, and every stop bumps it (`Bench._stop_shaper`) and purges the queued motion frames (`BusWorker.purge`):
  nothing queued before a stop goes out after it.
  Keep it that way. A new motion command goes through `Bench._start_motion`. A new code that moves the motor goes in
  `RAW_REFUSED`. The server binds 127.0.0.1 only, and checks the page's token and the Host header on every API call.
  It uses the standard library only (`http.server`, no web framework).
- `config.py` imports only the standard library: `cad/tests/test_robot.py` loads it by its path and checks `JOINTS`'
  ratios, limits and home angles against `cad/lib/params.py` (`tests/test_config.py` checks the imports). What the
  table must match - the joints, the reductions and which way round `gear_ratio` reads (output turns per motor
  turn), the home angles - is `cad/robot/AGENTS.md`. Which board carries which CAN id, each `sign` and the boards'
  firmware are unconfirmed (`cad/docs/open_issues.md`).
- Every MKS byte follows the manual V1.0.9, checked against it (never from memory); `tests/test_motor_driver.py`
  holds the manual's example frames. The boards' firmware is unknown: a command or byte the version notes date goes
  in its docstring, and the driver takes either reply where the manual shows they differ.
- An experiment's limits are its own: `JointLimits` passed in by the caller. `config.JOINTS` holds only the arm's
  hard limits.
- `command_sweep` moves a motor: it takes one `--can-id` from the command line (never the table's), reads only
  unless `--motion`, asks for `MOVE` at a terminal, and has no console script. Keep it that way.
