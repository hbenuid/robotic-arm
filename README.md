# robotic-arm

A six-joint desktop robot arm with a parallel gripper: its CAD, and the software that drives its MKS SERVO42D
stepper motors over CAN.

```
cad/                 the mechanical design: parametric build123d code, its own uv project; also the robot
                     description (URDF / SRDF / SDF)
software/
  control/           robotic_arm, the motor-control package and its CLIs (CAN via a CANable), its own uv project
  firmware/          microcontroller firmware (archived STM32 test projects)
```

The main work is the CAD. The software is the arm's motor control: its basic commands are in place, homing and
kinematics are planned, and other projects install it as a package. Both stay in one repo on purpose: the software
has to match the CAD's robot description (`cad/robot/arm.urdf`, the joints, the reductions, the home angles; a CAD
test checks it), and one repo keeps both sides in one commit.

## Getting started

- **CAD** — [`cad/README.md`](cad/README.md): setup, `./cadtool`, the layout.
- **Motor control** — [`software/control/README.md`](software/control/README.md): hardware, setup (Windows /
  macOS / Linux), configuring the boards, running the CLIs, using the package.
- **Firmware** — [`software/firmware/README.md`](software/firmware/README.md).

## Development

The working rules — branches (never `main`), commit messages, uv only (never `pip install`), lint, the git and Claude
Code hooks, when to run CI — are in [`AGENTS.md`](AGENTS.md). Claude Code reads that file; the rules hold for people too.
Each part of the repo has its own guide next to its code (`cad/AGENTS.md` and its folders, `software/control/AGENTS.md`).
What is not settled yet — fits, estimates, unmodelled hardware, unconfirmed mappings: [`cad/docs/open_issues.md`](cad/docs/open_issues.md).
