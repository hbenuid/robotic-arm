# software/control — the arm's motor control

`robotic_arm`, a Python package that drives the arm's MKS SERVO42D boards over CAN, plus its CLIs. It works through
a CANable (slcan firmware: any OS; candleLight firmware: socketcan on Linux) or python-can's virtual bus for tests.
It is its own uv project and never depends on the CAD ([`cad/`](../../cad/README.md)). Other projects install it as
a package (`uv add --editable <path>/software/control`; [`docs/architecture.md`](docs/architecture.md)).

It grew out of the BU Robotics Club's Arctos control code. The layer design and the CLI come from there; the code is
rewritten for this arm and checked against the MKS manual V1.0.9 ([`docs/mks_can_protocol.md`](docs/mks_can_protocol.md)).

## Hardware

- Six MKS SERVO42D boards, one per joint: base_yaw, shoulder_pitch, elbow_pitch, forearm_roll, wrist_pitch,
  wrist_roll (the joints and the motor on each: [`cad/robot/AGENTS.md`](../../cad/robot/AGENTS.md)). The gripper's
  MG996R is not on the CAN bus.
- A CANable USB-to-CAN adapter.
- A 500 kbit/s CAN bus (the MKS factory default).

Not confirmed yet ([`cad/docs/open_issues.md`](../../cad/docs/open_issues.md)): which board carries which CAN id,
which way each joint's encoder counts (`sign`), and which MKS firmware the boards run.

## Setup

### Windows

1. Clone the repo.
2. Double-click `software\control\launch.bat`. That's it.

The first run installs uv and the project's dependencies (~30 s); every later run goes straight to the motor
controller menu.

### macOS / Linux

Install `uv` (macOS: `brew install uv`; Linux / macOS: `curl -LsSf https://astral.sh/uv/install.sh | sh`), then:

```
git clone <repo-url>
cd robotic-arm/software/control
uv sync
```

`uv sync` creates `software/control/.venv/`, installs `python-can`, `pyserial` and the package, and registers the
`launcher` and `motor-control` commands.

## Configure the boards and the joint table

Each board, once (on its menu or by command; the list: [`docs/mks_can_protocol.md`](docs/mks_can_protocol.md)
"Setting up a board"): Mode **SR_vFOC**, CanRate 500K, and CanID its joint's row in `config.JOINTS`, 1 (base_yaw)
to 6 (wrist_roll). Set the ids with one new board on the bus at a time: they all ship as id 1.

[`src/robotic_arm/config.py`](src/robotic_arm/config.py):

- `JOINTS`: one `JointConfig` per joint, holding its CAN id, `gear_ratio`, `sign`, hard limits and home angle. The
  ratios, limits and home angles are the CAD's (`cad/lib/params.py`), and `cad/tests/test_robot.py` fails if they
  drift. `gear_ratio` is **output turns per motor turn**, the reciprocal of the CAD's reductions (`1 / 22.5` for
  the elbow). A joint with no board on the bus gets `can_id=None`.
- `CAN_INTERFACE`, `CAN_CHANNEL`, `CAN_BITRATE`: the CLI's defaults. `uv run launcher` passes the serial port it
  finds.
- `DEFAULT_SPEED` (motor RPM), `DEFAULT_ACC`, `REPLY_TIMEOUT_S`, `ENCODER_COUNTS_PER_REV`: usually left alone.

## Run

```
cd software/control
uv run launcher                                     # finds the CANable (slcan) and opens the menu
uv run launcher --channel /dev/ttyACM0              # a port of your choice (macOS /dev/cu.usbmodem*, Windows COM3)
uv run motor-control --interface socketcan --channel can0   # candleLight firmware (Linux; `ip link` sets the bitrate)
uv run motor-control --fake                         # simulated boards, no hardware
```

(Windows: double-click `software\control\launch.bat`; it runs the same thing.)

**Safety**: `Ctrl+C` at any time sends an emergency stop to every board in `JOINTS` before exiting. A board whose
CAN id is not there is not stopped. Disabling a motor frees its joint: support the arm first. A safe first command
(no motion): `[0] Select motor → [1] Read / Status → [8] Read version (0x40)`, which also tells you the board's
firmware.

### The dashboard

A page in your browser for driving and testing the boards on the bus, served from your own machine. It starts with
`--joint`'s board and adds every other board whose CAN id is a joint's in `config.JOINTS` (`--no-scan` skips that;
the page's **+** card adds one by hand). Each motor is a card across the top: its position, speed and state, and a
small jog. Click a card (or press 1-9) to make it the active one, which the panels below drive, graph and test.
**Move together** gives each motor a target and starts them in the same instant, timed to arrive together; if one
move is refused, nothing moves. STOP stops them all.

```
uv run dashboard                                    # finds the CANable (slcan), opens http://127.0.0.1:8765/
uv run dashboard --joint elbow_pitch                # the joint to start on (the page can change it)
uv run dashboard --channel /dev/cu.usbmodem…        # a port of your choice
uv run dashboard --fake                             # simulated boards that move in time, no hardware
```

It shows the board's readouts live (position, speed, status, enable, stall, the home switch on IN_1, voltage,
current, angle error), with position and speed graphed, and every CAN frame in a log. From it you can:

- enable or disable the motor; step it with ← / → (or the Step buttons: one press, one step of the Step amount);
  jog it with Shift + ← / → (or the ◀ Jog ▶ buttons: it moves while held and stops when you let go); change the
  speed with ↑ / ↓, go to an angle, set the ramp;
- set limits: a min and a max, from where the motor is now (Here) or typed, and a switch to turn them off;
- experiment with smoothness (Smoothing in the left panel, the Tuning tab): the board's own straight ramp, or an
  S-curve the dashboard shapes by streaming speed updates to the board (curve linear / smoothstep / sine / quintic,
  ease time, updates per second, how hard the board follows), with Snappy / Balanced / Smooth presets; the board's
  work mode, microsteps, interpolation, current and holding current; its PID gains (firmware V1.0.8+); the graph's
  speed from the encoder or the board, the commanded speed over it, and 20 / 10 / 5 ms sampling;
- home it (0x90 + 0x91), set its zero, release a stall, calibrate it (bench mode only);
- run the tests: repeat moves between two angles, a step response, a speed sweep;
- run the bring-up checks: which CAN ids answer, the board's firmware, which way the encoder counts (the joint's
  `sign`), and where the home switch turns on and off. It lists what it measured as text for `config.py` or
  `open_issues.md`; it never edits files;
- read and write the board's settings (work mode, current, microsteps, stall protection, CAN id, heartbeat), send a
  raw command, and record the readouts to CSV.

It has two modes. **Joint mode** works in joint degrees through the joint's ratio and sign, at no more than 10 °/s
(`--max-joint-speed` raises the cap). With the joint's reference (the joint angle at the encoder's zero: home the
joint, or tell the page where it is) the angles are the joint's; without it they count from the encoder's zero.
**Bench mode** works in degrees and RPM of the motor's shaft.

**Limits** are on by default and start open. Each side is what you set (Here, or typed); × or an empty box opens that
side again. Use arm range sets both to the arm's own from `config.JOINTS` once joint mode has a reference - a
reference never sets them by itself. Every move and jog is checked against them before anything is sent. They follow
the physical position through a change of mode, sign or reference. Set zero shifts them with the zero; homing or a
power loss clears them. Switched off, nothing limits the position: only the speed cap and STOP remain.

**Set zero when a board comes on**: after power-up a board's targets count from where it switched on, while its
encoder reads the magnet's own angle, so every move would land that far off until Set zero or homing (base_yaw:
1551 counts). So when a board comes on - the dashboard starts, or it answers again after a silence - the dashboard
presses Set zero for it once it reports stopped, and refuses moves until then. The Set zero button stays for any
other time.

**Safety**: STOP (the button, Esc, or Space outside a text box) sends F7 and an immediate F5 stop. These go ahead of
anything else queued. A jog stops when you let go: the page repeats it every 0.1 s, and the server stops the motor 0.4 s
after the last one. A jog aims at most 2 s of travel ahead (never past a limit), and its keep-alives move that target
on, so a crashed dashboard leaves at most that much travel. Any motion or test stops if the page goes quiet for 2 s (tab
closed, laptop asleep). The constant-speed run is bench mode's only, and it stops at a limit. A board that goes silent
and comes back loses its reference and your limits, in case it lost power. The raw command box refuses the commands
that move the motor. Ctrl+C in the terminal, or killing the process (SIGTERM, SIGHUP), stops a moving motor and quits.

### Checking one board

`command_sweep` runs the club's command check on **one** board, named by its CAN id:

```
uv run python -m robotic_arm.command_sweep --interface slcan --channel /dev/ttyACM0 --can-id 1
uv run python -m robotic_arm.command_sweep --interface slcan --channel /dev/ttyACM0 --can-id 1 --motion
```

Without `--motion` it only reads. With it, after you type `MOVE` at a terminal, it enables and disables the motor,
moves it 90° of the motor out and back (`--move-motor-deg`), spins it briefly, stops it and disables it. Before
asking, it shows what that move is at each joint.

## Layout

```
launch.bat                     Windows double-click launcher
src/robotic_arm/
  config.py                    the joint table, CAN bus defaults, motion defaults
  can_interface.py             layer 1: python-can frames with timestamps
  motor_driver.py              layer 2: the MKS protocol, one board
  joint.py                     layer 3: degrees <-> counts, limits
  arm.py                       layer 4: every board - send all, collect, e-stop all, checked moves
  fake.py                      simulated boards on the virtual bus
  motor_control.py             the interactive CLI (`motor-control`)
  launcher.py                  finds the adapter, runs the CLI (`launcher`)
  command_sweep.py             one board's command check
  dashboard/                   the dashboard (`dashboard`): every board on the bus, in the browser
    bus.py                       the one thread on the CAN bus: queue, polls, replies, log
    bench.py                     one motor: units, limits, the safety rules
    rig.py                       every motor on the page: the active one, STOP all, moves together
    routines.py                  the tests and bring-up checks
    shaping.py                   S-curve moves: the speed streamed to the board as F5 updates
    server.py                    the local HTTP server and the command
    static/                      the page (HTML, CSS, JS)
tests/                         pytest, on the virtual bus and the fake boards
docs/architecture.md           the layers, using the package, the plan for the next layers
docs/mks_can_protocol.md       the CAN commands, checked against the MKS manual V1.0.9
pyproject.toml, uv.lock        uv-managed project metadata
```

## Development

`uv run pytest` runs the tests (no hardware: python-can's virtual bus), `uv run ruff check` lints and
`uv run ty check` checks the types. The working rules (branches, commit messages, uv only, the hooks, CI) are the
root [`AGENTS.md`](../../AGENTS.md); this folder's rules are [`AGENTS.md`](AGENTS.md). The git pre-commit hook (ruff
on the staged files, ty on their project, both projects) is installed per machine by `cad/cadtool setup`.
