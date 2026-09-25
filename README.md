# robotic-arm

A six-joint desktop robot arm with a parallel gripper: its CAD, and the software that drives its MKS SERVO42D
stepper motors over CAN.

```
cad/                 the mechanical design: parametric build123d code, its own uv project; also the robot
                     description (URDF / SRDF / SDF). STEP / STL inputs are Git LFS objects
software/
  control/           the motor-control CLI (CAN via a CANable / slcan adapter), its own uv project
  firmware/          microcontroller firmware (archived STM32 test projects)
```

The current work is the CAD; the software is picked up later. Both stay in one repo on purpose: when the software is
picked up it has to match the CAD's robot description (`cad/robot/arm.urdf`, the joints, the reductions), and one
repo keeps both sides in one commit.

## Getting started

- **CAD** — install `git-lfs` before cloning, then [`cad/README.md`](cad/README.md): setup, `./cadtool`, the layout.
- **Motor control** — [`software/control/README.md`](software/control/README.md): hardware, setup (Windows /
  macOS / Linux), configuring the motors, running the CLI.
- **Firmware** — [`software/firmware/README.md`](software/firmware/README.md).

## Development

The working rules — branches (never `main`), commit messages, uv only (never `pip install`), lint, the git and Claude
Code hooks, when to run CI — are in [`CLAUDE.md`](CLAUDE.md). Claude Code reads that file; the rules hold for people too.
Each part of the repo has its own guide next to its code (`cad/CLAUDE.md` and its folders, `software/control/CLAUDE.md`).
What is not settled yet — fits, estimates, unmodelled hardware, unconfirmed mappings: [`cad/docs/open_issues.md`](cad/docs/open_issues.md).
