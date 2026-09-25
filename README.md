# robotic-arm

Robot arm control software for a 3-motor MKS SERVO42D/57D arm over CAN bus, and the arm's CAD — which has
since outgrown the three motors (see [CAD](#cad)).

## Hardware

- 3× MKS SERVO42D/57D stepper drivers (`software/control/src/config.py` J1..J3 = CAN ids 1..3; which arm joint each one
  drives is still to be confirmed: [`cad/docs/open_issues.md`](cad/docs/open_issues.md))
- CANable / slcan-compatible USB-to-CAN adapter
- 500 kbit/s CAN bus (MKS factory default)

## CAD

The arm's mechanical design lives in [`cad/`](cad/README.md): parametric [build123d](https://github.com/gumyr/build123d)
code in its own uv project, which the control software never depends on. It also produces the arm's robot description
— URDF / SRDF / SDF, the joints and the motor on each ([`cad/robot/`](cad/robot/CLAUDE.md)). Its STEP / STL files are
Git LFS objects: install `git-lfs` before cloning. Setup, commands and layout: [`cad/README.md`](cad/README.md).

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

Edit [software/control/src/config.py](software/control/src/config.py):

- `JOINTS` — three `(name, can_id, gear_ratio)` tuples. Defaults are placeholders
  (CAN IDs `0x01`/`0x02`/`0x03`, gear ratio `1.0`). Set CAN IDs to match what
  you've programmed into each motor's on-board menu. The CAD's reductions are
  `cad/lib/params.py` `CYCLOIDAL_RATIO` (20:1, shoulder pitch), `GT2_RATIO` (4.5:1, elbow and wrist-pitch belts)
  and `FOREARM_ROLL_RATIO` (4.5:1, forearm roll); `gear_ratio` counts output turns per motor turn, so it takes their
  reciprocal ([cad/robot/CLAUDE.md](cad/robot/CLAUDE.md)). Which motor drives which joint is not confirmed yet, so
  the ratios here stay `1.0`.
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

## Project structure

The repo splits into the CAD and everything else:

```
cad/                         parametric build123d CAD, separate uv project (see cad/README.md); STEP/STL via Git LFS
software/
  control/                   the motor-control CLI, its own uv project
    launcher.py              auto-detect CAN port + run motor_control
    motor_control.py         interactive CLI (menus, command dispatch)
    launch.bat               Windows double-click launcher
    src/
      config.py              joint table, CAN bus settings, motion defaults
      can_interface.py       thin python-can wrapper
      motor_driver.py        MKS CAN protocol (CRC, encode/decode, commands)
    tests/                   pytest suite (currently broken — see Known issues)
    pyproject.toml, uv.lock  uv-managed project metadata
  firmware/
    stm32/                   archived STM32 (Nucleo-F446RE) PlatformIO test firmware for the SERVO42D
```

The software will be worked on later; for now the work is the CAD. Both stay in one repo on purpose: when the
software is picked up it has to match the CAD's robot description (`cad/robot/arm.urdf`, joint table, gear ratios).

## Development

The working rules — branches (never `main`), commit messages, uv only (never `pip install`), lint, the git and Claude
Code hooks, when to run CI — are in [`CLAUDE.md`](CLAUDE.md). Claude Code reads that file; the rules hold for people too.

## Known issues

- [software/control/tests/](software/control/tests/) still imports `from arctos.*` and won't run until updated to
  match the current `src/` layout. The CLI works without the test suite.
