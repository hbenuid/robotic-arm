# software/control — the motor-control CLI

An interactive CLI that drives MKS SERVO42D / 57D stepper drivers over a CAN bus through a CANable /
slcan-compatible USB adapter, on macOS, Linux and Windows. Its own uv project; it never depends on the CAD
([`cad/`](../../cad/README.md)). The software is picked up after the CAD — see the [repo README](../../README.md).

## Hardware

- MKS SERVO42D stepper drivers. [`src/config.py`](src/config.py) names three (J1..J3 = CAN ids 1..3); the arm the CAD
  describes carries more MKS boards, and which joint each CAN id drives is not confirmed yet:
  [`cad/docs/open_issues.md`](../../cad/docs/open_issues.md). The joints and the motor on each:
  [`cad/robot/CLAUDE.md`](../../cad/robot/CLAUDE.md).
- CANable / slcan-compatible USB-to-CAN adapter
- 500 kbit/s CAN bus (MKS factory default)

## Setup

### Windows

1. Clone the repo.
2. Double-click `software\control\launch.bat`. That's it.

The first run installs uv and project dependencies (~30s); every subsequent
run goes straight to the motor controller menu.

### macOS / Linux

Install `uv`:

- macOS: `brew install uv`
- Linux / macOS: `curl -LsSf https://astral.sh/uv/install.sh | sh`

Then:

```
git clone <repo-url>
cd robotic-arm/software/control
uv sync
```

`uv sync` creates `software/control/.venv/`, installs `python-can` + `pyserial`, and registers
the `launcher` console script in `.venv/bin/`.

## Configure your motors

Edit [`src/config.py`](src/config.py):

- `JOINTS` — three `(name, can_id, gear_ratio)` tuples. Defaults are placeholders
  (CAN IDs `0x01`/`0x02`/`0x03`, gear ratio `1.0`). Set CAN IDs to match what
  you've programmed into each motor's on-board menu. The CAD's reductions are
  `cad/lib/params.py` `CYCLOIDAL_RATIO` (20:1, shoulder pitch), `GT2_RATIO` (4.5:1, elbow and wrist-pitch belts)
  and `FOREARM_ROLL_RATIO` (4.5:1, forearm roll); `gear_ratio` counts output turns per motor turn, so it takes their
  reciprocal ([`cad/robot/CLAUDE.md`](../../cad/robot/CLAUDE.md)). Which motor drives which joint is not confirmed
  yet, so the ratios here stay `1.0`.
- `CAN_CHANNEL` — only needed if auto-detect fails. The launcher overrides this
  on every run.
- `CAN_BITRATE`, `DEFAULT_SPEED`, `DEFAULT_ACC`, `ENCODER_COUNTS_PER_REV` —
  usually leave alone.

To set or check a motor's CAN ID, use the buttons + OLED on the SERVO42D/57D
itself (Menu → CAN → ID).

## Run

From `software/control/` (the CLI finds its `src/` modules relative to the current directory):

```
cd software/control
uv run launcher
```

(Windows: just double-click `software\control\launch.bat` — same thing, no terminal needed.)

`launcher` auto-detects the CANable via `pyserial` — works on macOS
(`/dev/cu.usbmodem*`), Linux (`/dev/ttyACM*`), and Windows (`COM*`). It opens
the CAN bus at the configured bitrate and drops you into the interactive menu.

Override config from the command line if needed:

```
uv run launcher --channel /dev/cu.usbmodemXXXX    # macOS
uv run launcher --channel /dev/ttyACM0            # Linux
uv run launcher --channel COM3                    # Windows
uv run launcher --bitrate 250000                  # change bitrate
```

**Safety**: `Ctrl+C` at any time sends emergency-stop to ALL motors before
exiting. A safe first command (no motion): `[0] Select motor → [1] Read /
Status → [4] Read motor status (0xF1)` — returns `Stopped` for a healthy motor.

## Layout

```
launcher.py              auto-detect CAN port + run motor_control
motor_control.py         interactive CLI (menus, command dispatch)
launch.bat               Windows double-click launcher
src/
  config.py              joint table, CAN bus settings, motion defaults
  can_interface.py       thin python-can wrapper
  motor_driver.py        MKS CAN protocol (CRC, encode/decode, commands)
tests/                   pytest suite (currently broken — see Known issues)
pyproject.toml, uv.lock  uv-managed project metadata
```

## Development

Lint with `uv run ruff check` here. The working rules — branches, commit messages, uv only, the hooks, CI — are the
root [`CLAUDE.md`](../../CLAUDE.md), this folder's rules [`CLAUDE.md`](CLAUDE.md). The git pre-commit hook (ruff on
the staged files, both projects) is installed per machine by `cad/cadtool setup`.

## Known issues

- [`tests/`](tests/) still imports `from arctos.*` and won't run until updated to match the current `src/` layout
  (and `pytest` is not in the dev group yet). The CLI works without the test suite.
