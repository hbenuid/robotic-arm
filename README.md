# robotic-arm

**Last updated:** 2026-09-21 — see the root `CHANGELOG.md` for dated changes.

Robot arm control software for a 3-motor MKS SERVO42D/57D arm over CAN bus.

## Hardware

- 3× MKS SERVO42D/57D stepper drivers (`src/config.py` J1..J3 = CAN ids 1..3; which arm joint each one
  drives is still to be confirmed against the CAD's robot description below)
- CANable / slcan-compatible USB-to-CAN adapter
- 500 kbit/s CAN bus (MKS factory default)

## CAD

The arm's mechanical design lives in [`cad/`](cad/README.md) as parametric
[build123d](https://github.com/gumyr/build123d) code — a **separate uv project**
(Python 3.12) that the control software never depends on, driven by the
[`text-to-cad`](https://github.com/earthtojake/text-to-cad) CAD skills plugin (v0.6.x, runtime `cadgen`). It is being converted
part-by-part from the original SolidWorks STEP exports (kept in `cad/reference/`);
until a part is converted it is an import wrapper around its reference geometry, so
the whole arm already assembles and renders:

```
cd cad
./cadtool setup                      # one-time: venv + Playwright Chromium (snapshots)
./cadtool gen assemblies/arm.py      # build the arm STEP (rebuilds any stale part and its committed STEP)
./cadtool viewer                     # CAD Viewer: http://127.0.0.1:3245/?file=assemblies/arm.step (or ?file=robot/arm.urdf: joint sliders)
./cadtool pytest                     # convention + reference-match tests
./cadtool clean                      # drop __pycache__/pytest caches (cadgen's store lives in ~/.cache/cadgen)
```

The committed CAD binaries (`cad/**/*.step`, `cad/**/*.stl`) are **Git LFS** objects: install
`git-lfs` and run `git lfs install` before cloning (or `git lfs pull` afterwards).

The same CAD also produces the arm's robot description — [`cad/robot/arm.urdf`](cad/robot/arm.urdf)
(+ SRDF for MoveIt2, SDF for Gazebo) with per-link meshes, validated by cadgen's checkers. Its chain:
`base_link → base_yaw → shoulder_link → shoulder_pitch → upper_arm_link → elbow_pitch → forearm_link →
wrist_pitch → wrist_pitch_link → wrist_roll → wrist_roll_link → jaw_a / jaw_b (+ tool0)`, where
`shoulder_pitch` **is** the 20:1 cycloidal drive (its housing turns with the base-yaw holder, its output
hub carries the upper arm), `elbow_pitch` / `wrist_pitch` are the GT2 belt joints, `wrist_roll` the
NEMA17 pancake and the jaws the MG996R gripper. The CAD now places the three belt-joint motors (MKS SERVO42D kits on the pads
the links carry, `cad/lib/mounts.py`: a 48 mm NEMA 17 under the base, 40 mm ones at the elbow and wrist) and the drive's own 48 mm kit; which CAN id
(`src/config.py` J1..J3) drives which joint is not confirmed yet.

The 20:1 cycloidal shoulder drive (formerly the separate `cycloidal_drive` CadQuery repo) is fully
parametric build123d here — see [`cad/docs/cycloidal_drive.md`](cad/docs/cycloidal_drive.md).

See [`cad/README.md`](cad/README.md) for the workflow and [`cad/CLAUDE.md`](cad/CLAUDE.md)
for the conventions.

## Setup

### Windows

1. Clone the repo.
2. Double-click `launch.bat`. That's it.

The first run installs uv and project dependencies (~30s); every subsequent
run goes straight to the motor controller menu.

### macOS / Linux

Install `uv`:

- macOS: `brew install uv`
- Linux / macOS: `curl -LsSf https://astral.sh/uv/install.sh | sh`

Then:

```
git clone <repo-url>
cd robotic-arm
uv sync
```

`uv sync` creates `.venv/`, installs `python-can` + `pyserial`, and registers
the `launcher` console script in `.venv/bin/`.

## Configure your motors

Edit [src/config.py](src/config.py):

- `JOINTS` — three `(name, can_id, gear_ratio)` tuples. Defaults are placeholders
  (CAN IDs `0x01`/`0x02`/`0x03`, gear ratio `1.0`). Set CAN IDs to match what
  you've programmed into each motor's on-board menu. The CAD's reductions are
  `cad/lib/params.py` `CYCLOIDAL_RATIO` (20:1, shoulder pitch) and `GT2_RATIO` (4.5:1, belts) —
  which motor drives which joint is not confirmed yet, so the ratios here stay `1.0`.
- `CAN_CHANNEL` — only needed if auto-detect fails. The launcher overrides this
  on every run.
- `CAN_BITRATE`, `DEFAULT_SPEED`, `DEFAULT_ACC`, `ENCODER_COUNTS_PER_REV` —
  usually leave alone.

To set or check a motor's CAN ID, use the buttons + OLED on the SERVO42D/57D
itself (Menu → CAN → ID).

## Run

```
uv run launcher
```

(Windows: just double-click `launch.bat` — same thing, no terminal needed.)

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

```
launcher.py              auto-detect CAN port + run motor_control
motor_control.py         interactive CLI (menus, command dispatch)
src/
  config.py              joint table, CAN bus settings, motion defaults
  can_interface.py       thin python-can wrapper
  motor_driver.py        MKS CAN protocol (CRC, encode/decode, commands)
cad/                     parametric build123d CAD, separate uv project (see cad/README.md); STEP/STL via Git LFS
old_stm32_tests/         archived STM32 firmware experiments (separate from Python)
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
