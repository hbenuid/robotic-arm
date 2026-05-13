# robotic-arm

Robot arm control software for a 3-motor MKS SERVO42D/57D arm over CAN bus.

## Hardware

- 3× MKS SERVO42D/57D stepper drivers (one per joint: J1, J2, J3)
- CANable / slcan-compatible USB-to-CAN adapter
- 500 kbit/s CAN bus (MKS factory default)
- Host: macOS

## Setup

Prerequisite: `uv` (`brew install uv` on macOS).

```
git clone <repo-url>
cd robotic-arm
uv sync
```

`uv sync` creates `.venv/`, installs `python-can` + `pyserial`, and registers the
`launcher` console script in `.venv/bin/`.

## Configure your motors

Edit [src/config.py](src/config.py):

- `JOINTS` — three `(name, can_id, gear_ratio)` tuples. Defaults are placeholders
  (CAN IDs `0x01`/`0x02`/`0x03`, gear ratio `1.0`). Set CAN IDs to match what
  you've programmed into each motor's on-board menu.
- `CAN_CHANNEL` — only needed if auto-detect fails or you're not on macOS.
- `CAN_BITRATE`, `DEFAULT_SPEED`, `DEFAULT_ACC`, `ENCODER_COUNTS_PER_REV` —
  usually leave alone.

To set or check a motor's CAN ID, use the buttons + OLED on the SERVO42D/57D
itself (Menu → CAN → ID).

## Run

```
uv run launcher
```

`launcher` auto-detects the CANable at `/dev/tty.usbmodem*`, opens the CAN bus
at the configured bitrate, and drops you into the interactive menu.

Override config from the command line if needed:

```
uv run launcher --bitrate 250000 --channel /dev/tty.usbmodemXXXX --interface slcan
```

**Safety**: `Ctrl+C` at any time sends emergency-stop to ALL motors before
exiting. A safe first command (no motion): `[0] Select motor → [1] Read /
Status → [4] Read motor status (0xF1)` — returns `Stopped` for a healthy motor.

## Project structure

```
launcher.py              auto-detect CAN port + run motor_control
motor_control.py         interactive CLI (menus, command dispatch)
src/
  config.py              joint table, CAN bus settings, motion defaults
  can_interface.py       thin python-can wrapper
  motor_driver.py        MKS CAN protocol (CRC, encode/decode, commands)
stm32_tests/             STM32 firmware experiments (separate from Python)
tests/                   pytest suite (currently broken — see Known issues)
pyproject.toml, uv.lock  uv-managed project metadata
```

## Development

- Use uv exclusively: `uv add <pkg>` to add deps, never `pip install`. Commit
  both `pyproject.toml` and `uv.lock`.
- `motor_control.py` does `sys.path.insert(0, "src")` so the helpers under
  `src/` import without a package prefix (`from can_interface import …`). This
  is intentional — don't convert `src/` into a real Python package without
  rewriting all those imports.
- Adding a new entry-point script: add a `[project.scripts]` line in
  `pyproject.toml`, then `uv sync`.

## Known issues

- [tests/](tests/) still imports `from arctos.*` and won't run until updated to
  match the current `src/` layout. The CLI works without the test suite.
