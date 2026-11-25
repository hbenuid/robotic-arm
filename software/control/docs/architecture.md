# The motor-control software's layers

How `src/robotic_arm/` is built: what each layer does now, and the plan for the ones still to come. The layer design is
the BU Robotics Club Arctos software's (its `docs/ARCHITECTURE.md`). Its joint, arm and kinematics files were stubs,
so everything above the motor driver here is new, and its Arctos numbers (joint table, DH parameters) do not apply to
this arm.

```
  user interface        motor_control.py (CLI), command_sweep.py, dashboard/ (one board in the browser);
                        another project's code
  kinematics            - planned: numeric FK / IK from cad/robot/arm.urdf
  arm (layer 4)         arm.py        every board on one bus: send all, collect, e-stop all, checked moves
  joint (layer 3)       joint.py      one joint: degrees <-> counts, the arm's limits and the caller's
  motor driver (2)      motor_driver.py  the MKS protocol for one board: frames, check bytes, replies
  CAN interface (1)     can_interface.py python-can: frames with receive timestamps
  config                config.py     the arm's constants: bus defaults, the joint table
  fake boards           fake.py       simulated boards on python-can's virtual bus, for tests
```

Each layer calls only the ones below it; nothing imports from outside the package except python-can and pyserial.

## The layers today

**1. CAN interface** (`CANInterface`, `Frame`). Opens one python-can bus. The interface is a parameter:
`slcan` (a CANable with slcan firmware: a serial port and a bitrate), `socketcan` (candleLight firmware on Linux:
a netdev like `can0`, with the bitrate set by `ip link`), or `virtual` (in-process, for tests). `receive()` returns a
`Frame(can_id, data, timestamp)` with python-can's receive timestamp. A frame one `receive()` passes over is kept
for a later one, so replies from several boards can be picked up in any order. One thread per instance.

**2. Motor driver** (`MotorDriver`, `Reply`, the `*_request` builders). One instance per board. It appends the
check byte, sends, checks and parses replies (`Reply(can_id, data, timestamp)`, `data[0]` the code), and wraps the
commands in methods (`docs/mks_can_protocol.md`). Every blocking method waits for its own reply. The non-blocking
pair is `send()` and `parse()`. No gear ratios or angles at this layer.

**3. Joint** (`Joint`, `JointLimits`, `LimitError`). Converts joint degrees to the board's encoder counts through
`gear_ratio` (output turns per motor turn), `sign`, and `zero_deg` (the joint angle at the encoder's zero:
`home_deg` once the board has homed). Converts deg/s to motor RPM. Checks a move against the arm's hard limits
(`config.JOINTS`) narrowed by the caller's `JointLimits` (angles, deg/s, ramp) before anything is sent.

**4. Arm** (`Arm`). The `config.JOINTS` joints on one bus. A joint with `can_id=None` has no board and is skipped.
Every multi-board operation sends all its frames first and only then waits, so a silent board delays nobody:
`send_all` / `collect` / `request_all`, `emergency_stop` (`F7` to every mapped board), `read_positions` (angles with
timestamps), `move_joints` (all targets checked first: one bad target and nothing moves).

**Fake boards** (`FakeBoards`, `FakeBoard`). A thread on a virtual channel answering the driver's commands from a
small state model. It records every frame it saw. Tests and the CLI's `--fake` use it; it is not the firmware.

## Using the package from another project

```
uv add --editable <path>/software/control        # imports as robotic_arm
```

```python
from robotic_arm import Arm, CANInterface, JointLimits, Cmd

with CANInterface("can0", "socketcan", bitrate=None) as can:          # or ("/dev/ttyACM0", "slcan")
    arm = Arm(can)
    replies = arm.request_all(bytes([Cmd.READ_VERSION]), timeout=0.2)  # name -> Reply, each with .timestamp
    sent = arm.send_all(bytes([Cmd.READ_ENCODER]))                     # send now ...
    replies = arm.collect(Cmd.READ_ENCODER, sent, timeout=0.05)        # ... collect later
    arm.move_joints({"elbow_pitch": -30.0}, speed_dps=10.0, acc=2,
                    limits={"elbow_pitch": JointLimits(min_deg=-41.0, max_deg=-20.0, max_speed_dps=15.0)})
    arm.emergency_stop()
```

The caller owns its limits: `config.JOINTS` holds only the arm's own. In tests, run `FakeBoards(channel, ids)` and a
`CANInterface(channel, "virtual", bitrate=None)` on the same channel name, in one process.

## Design rules

1. **Layers call down, never up.** A lower layer reports through return values, not imports.
2. **The arm's constants live in `config.py`**, which mirrors the CAD's numbers (`cad/tests/test_robot.py` checks).
   The protocol's own constants (codes, ranges) live in `motor_driver.py`, beside the bytes they describe.
3. **The emergency stop works at every layer**: `MotorDriver.emergency_stop`, `Arm.emergency_stop`, the CLIs'
   Ctrl+C. It goes to every mapped board before waiting for any reply.
4. **Check before sending.** Limits are checked before the first frame of a move goes out; a bad argument raises.
5. **Fail safe, never silently.** A missing or bad reply is `None` / `False`, never a guess; the CLIs report it, and
   `command_sweep` stops the motor at a failed motion check.
6. **Every layer has tests**, on the fake boards or python-can's virtual bus. Only hardware checks need the arm.

## The plan for what is not built yet

From the club's design, adapted to this arm:

- **Joint:** `start_move(deg)` / `wait_until_done()` (the second `F5` reply or `F1` polling), `is_moving()`,
  `start_home()`: `90` with hm_mode 0 for the joint's KY-003 switch, then `91`, then `3B` until it succeeds. After
  that, `zero_deg = home_deg`.
- **Arm:** `home_all()` in a safe order (each switch's zone lies past its joint's lower end, `cad/robot/AGENTS.md`),
  `enable_all()`, `wait_all()`, a configured heartbeat (`98`) so a crashed host stops the arm.
- **Kinematics:** forward and inverse kinematics from `cad/robot/arm.urdf`'s joint origins. The IK has to be
  **numeric**: the elbow and the wrist carry offsets, so the last three axes do not meet (`cad/docs/open_issues.md`).
  The club's DH table is the Arctos's and does not apply. A planner here also keeps the poses the joint limits allow
  but the arm cannot reach: the gripper into the upper arm past `ELBOW_GRIPPER_FREE_DEG`, and the table under the
  base.
- **Trajectories:** the boards ramp each move themselves (`acc`), so single moves need no planner. Straight-line or
  multi-waypoint paths would add one between kinematics and the user interface. `F5` takes a new target mid-move.
- **Threading:** single-threaded today; `CANInterface`'s kept frames let one thread interleave boards. A background
  receive thread dispatching to per-board queues would fit under `CANInterface` without changing the layers above.
- **The gripper:** an MG996R hobby servo, not an MKS board. Its microcontroller firmware goes in
  `software/firmware/<board>/`.

## A move, top to bottom

1. The caller asks `arm.move_joints({"elbow_pitch": -30.0}, 10.0, 2, limits)`.
2. `Joint.check_move` checks −30° against the arm's −47..144 and the caller's limits, then turns it into counts
   (`(−30 − zero_deg) × 1024` at 22.5:1) and 10 deg/s into 38 RPM.
3. `move_to_axis_request` builds `F5`, speed, acc, int24 position; `MotorDriver.send` adds the check byte.
4. `CANInterface.send` puts the frame on the bus; the board starts the move and replies `F5 01`.
5. `Arm.collect` picks up each board's reply with its timestamp; the board's second reply (`F5 02`) says it arrived.
