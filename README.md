# robotic-arm

Robot arm control software for a 3-motor MKS SERVO42D/57D arm over CAN bus (the CAD's arm is now a 6-axis one that needs five boards - see the CAD section).

## Hardware

- 3× MKS SERVO42D/57D stepper drivers (`software/control/src/config.py` J1..J3 = CAN ids 1..3; which arm joint each one
  drives is still to be confirmed against the CAD's robot description below - which now places five kits:
  base yaw, elbow pitch, wrist pitch, the shoulder's cycloidal drive and the forearm roll)
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
./cadtool setup                      # per machine: venv + git pre-commit hook (ruff) + Playwright Chromium (snapshots)
./cadtool gen assemblies/arm.py      # build the arm STEP (rebuilds any stale part and its committed STEP)
./cadtool viewer                     # CAD Viewer: http://127.0.0.1:3245/?file=assemblies/arm.step (or ?file=robot/arm.urdf: joint sliders)
./cadtool pytest                     # convention + reference-match tests
./cadtool lint                       # ruff check
./cadtool clean                      # drop __pycache__/pytest caches (cadgen's store lives in ~/.cache/cadgen)
```

The committed CAD binaries (`cad/**/*.step`, `cad/**/*.stl`) are **Git LFS** objects: install
`git-lfs` and run `git lfs install` before cloning (or `git lfs pull` afterwards).

The same CAD also produces the arm's robot description — [`cad/robot/arm.urdf`](cad/robot/arm.urdf)
(+ SRDF for MoveIt2, SDF for Gazebo) with per-link meshes, validated by cadgen's checkers. Its chain:
`base_link → base_yaw → shoulder_link → shoulder_pitch → upper_arm_link → elbow_pitch → elbow_link →
forearm_roll → forearm_link → wrist_pitch → wrist_pitch_link → wrist_roll → wrist_roll_link → jaw_a / jaw_b
(+ tool0)` — six revolute joints, the last three concurrent at the wrist centre — where
`shoulder_pitch` **is** the 20:1 cycloidal drive (its housing turns with the base-yaw holder, its output
hub carries the upper arm), `elbow_pitch` / `wrist_pitch` are the GT2 belt joints, `forearm_roll` the
belt-driven roll drive in the elbow block ([`cad/docs/forearm_roll.md`](cad/docs/forearm_roll.md): one printed block
that is also the elbow's output flange, a hollow printed shaft crossing the elbow axis with a 90T ring in two 6808
bearings, the forearm bolted to its end spigot 48 mm from the elbow axis), `wrist_roll` the
NEMA17 pancake and the jaws the MG996R gripper. The CAD places the three belt-joint motors (MKS SERVO42D kits on the pads
the links carry, `cad/lib/mounts.py`: a 48 mm NEMA 17 under the base, 40 mm ones at the elbow and wrist), the drive's own
48 mm kit and the roll drive's 40 mm kit; which CAN id (`software/control/src/config.py` J1..J3 - three of the five) drives which joint is
not confirmed yet.

The 20:1 cycloidal shoulder drive (formerly the separate `cycloidal_drive` CadQuery repo) is fully
parametric build123d here — see [`cad/docs/cycloidal_drive.md`](cad/docs/cycloidal_drive.md).

See [`cad/README.md`](cad/README.md) for the workflow and [`cad/CLAUDE.md`](cad/CLAUDE.md)
for the conventions.

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
  `cad/lib/params.py` `CYCLOIDAL_RATIO` (20:1, shoulder pitch) and `GT2_RATIO` (4.5:1, belts) —
  which motor drives which joint is not confirmed yet, so the ratios here stay `1.0`.
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

- Work in `software/control/`. Use uv exclusively: `uv add <pkg>` to add deps,
  never `pip install`. Commit both `pyproject.toml` and `uv.lock`.
- `motor_control.py` does `sys.path.insert(0, "src")` so the helpers under
  `src/` import without a package prefix (`from can_interface import …`). This
  is intentional — don't convert `src/` into a real Python package without
  rewriting all those imports.
- Adding a new entry-point script: add a `[project.scripts]` line in
  `pyproject.toml`, then `uv sync`.
- Lint with `uv run ruff check` (`--fix` for the safe fixes; ruff is a dev
  dependency, config in `pyproject.toml` `[tool.ruff]`; `cad/` has its own,
  `./cadtool lint`). Lint only; `ruff format` is not used. It also runs
  by itself: a git pre-commit hook (`cd cad && ./cadtool setup` installs it on
  a machine) blocks commits with findings, a Claude Code hook checks every file
  Claude edits, and VS Code (Ruff extension) fixes and sorts imports on save.
- CI: GitHub Actions ([.github/workflows/ci.yml](.github/workflows/ci.yml))
  runs only by hand (Actions tab → CI → Run workflow, or
  `gh workflow run ci.yml --ref <branch>`), ~13 min on a fresh Ubuntu runner:
  lint (both projects), the CAD test suite and a build of the arm from a clean
  clone. Worth running before fast-forwarding `main` after a dependency bump,
  a new CAD export or a large change; everyday pushes don't need it.

## Known issues

- [software/control/tests/](software/control/tests/) still imports `from arctos.*` and won't run until updated to
  match the current `src/` layout. The CLI works without the test suite.
