# robot/ — the robot description (URDF / SRDF / SDF)

Loads when you work in `robot/`. After any change here: Recipe C steps 7–9 (`cad/AGENTS.md`).

```
robot/
├── frames.py          # THE kinematic decomposition: LINKS (which occurrences move together) + JOINTS
│                      # (axis point/direction, parent/child, limits from lib/params.py)
├── _links.py          # link_rows() / build_link(): one link's occurrences placed in the link frame (models, meshes, tests)
├── links/<link>.py    # a @step model per rigid link, in the link's own frame (./cadtool gen robot/links/shoulder_link.py)
├── meshes/<link>.stl  # per-link meshes in mm (committed) - tools/robot/export_link_meshes.py
├── arm.urdf           # SOURCE OF TRUTH (hand-edited ledger + numbers from tools/robot/derive.py)
├── arm.srdf           # MoveIt2 semantics: chain base_link->tool0, gripper group, home/open/closed states
└── arm.sdf            # model-level SDF 1.12 derived from the URDF
```

## Robot description
- `robot/arm.urdf` is the **source of truth** (hand-authored XML, ledger comment on top);
  `arm.srdf` pairs by colocation + robot name; `arm.sdf` is derived from the URDF. Never build a
  Python generator for them - `tools/robot/derive.py --urdf-draft/--sdf-draft` only prints scaffolding
  to copy numbers from, and `--check` (run by `tests/test_robot.py`) catches drift.
- `robot/frames.py` is the kinematic SSOT: `LINKS` (occurrence keys per rigid link; a designed-module
  key may carry a `:<body>` suffix — `cycloidal_drive#1:stator` / `:rotor` from
  `assemblies/cycloidal_drive.py BODIES`, expanded by `_occurrences.world_rows`) and `JOINTS` (axis
  point/direction in the SolidWorks capture frame — the points beyond a shortened link moved with it,
  `lib/placements.py SHIFTS` via `P.shifted()` — limits from `lib/params.py`). Frames are REP-103 (`base_link` on
  the base's bottom face at the base_yaw axis, Z up, X forward). Joint frame: Z on the axis, X along the child link
  (`forearm_roll`: X = N, its child lies along the axis); child link frame = joint frame at capture, so **all joints
  are 0 at the capture pose**, mesh origins are identity and the URDF at zero reproduces `assemblies/arm.py` (emitted
  in the same `base_link` frame — `assemblies/AGENTS.md`). Moving an occurrence between links or changing an axis =
  edit `frames.py`, re-export the meshes, re-derive the affected `<origin>` / `<inertial>` values from the drafts (the
  checked-in XML stays canonical), re-check.
- `robot/links/<link>.py` are `@step` models of each rigid link (`robot/_links.build_link`, in the link
  frame; `robot/links/<link>.step` git-ignored). Meshes: `tools/robot/export_link_meshes.py` (mm STL per
  link via build123d's `export_stl`, `scale 0.001` in the XML); never hand-edit.
  Inertials come from OCP `BRepGProp` (printed parts at `PETG_DENSITY`, COTS at `MASS_G`).
- Validate (`--strict`) and snapshot after every edit; hand `.urdf` files to the viewer (`?file=robot/arm.urdf`) —
  Commands below.
- Placeholders to confirm before real use: the joint limits / effort / velocity, the axis signs and the jaw travel (all
  `[ESTIMATE]` in `lib/params.py`, `*_LIMIT_DEG …` — confirm with viewer sweeps and hardware) and the link-membership
  assumptions the URDF ledger lists. Which CAN id (`software/control/src/config.py` J1..J3, fewer ids than the arm has
  MKS boards) drives which joint is unconfirmed (`docs/open_issues.md`); wrist_roll and the jaws are not driven by
  `software/control/src/config.py`.
- **The reductions the software must match** (`lib/params.py`): `BASE_YAW_RATIO` (base_yaw: its 120T is placed, the
  motor's 20T and the belt are not, `docs/open_issues.md`), `CYCLOIDAL_RATIO` (shoulder_pitch: the drive's shell
  turns, the same way as its motor - the port's hub turned against it), `GT2_RATIO` (the wrist_pitch and the
  elbow_pitch belts, one stage each: the elbow motor's 20T straight to the 90T, `docs/open_issues.md`),
  `FOREARM_ROLL_RATIO` (forearm_roll). They are motor revolutions per output revolution, while
  `software/control/src/config.py`'s `gear_ratio` is output revolutions per motor revolution — the software's value
  is the reciprocal (`1 / CYCLOIDAL_RATIO`, …).

## Joints and links
`base_link → base_yaw → shoulder_link → shoulder_pitch → upper_arm_link → elbow_pitch → elbow_link →
forearm_roll → forearm_link → wrist_pitch → wrist_pitch_link → wrist_roll → wrist_roll_link → jaw_a / jaw_b
(prismatic, jaw_b mimics jaw_a) + tool0` (frame-only) — six revolute joints, the last three concurrent at the
wrist centre (`forearm_roll`'s axis runs along the forearm through `WRIST_CENTRE`; `docs/forearm_roll.md`). The two
drives ARE joints: each one's stator stays in the parent link, its rotor moves with the child (`BODIES` in
`assemblies/cycloidal_drive.py` / `forearm_roll_drive.py`; the URDF ledger, `docs/cycloidal_drive.md` §12). The belt
joints' motors are mounted occurrences (`lib/mounts.py`, `assemblies/AGENTS.md`); the drives' motors are module rows.

| joint | type | parent → child | actuator | notes |
|---|---|---|---|---|
| `base_yaw` | revolute, world up | `base_link → shoulder_link` | GT2 120T belt (`BASE_YAW_RATIO`), NEMA 17 x 48 + MKS SERVO42D (`nema17_48mm#1` + `mks_servo42d#1` under the plate of the base's bolt-on motor mount, `BASE_MOTOR_TABLE_CLEAR` above the base's bottom face) | the holder `j1_coupler` turns on the base, standing on the thrust bearing in its groove, held down by the pulley bolted to its stub's end under the lower bearing |
| `shoulder_pitch` | revolute, `N` | `shoulder_link → upper_arm_link` | the 21:1 cycloidal drive, its own NEMA 17 (`CYCLOIDAL_RATIO`) | stator (the held carrier + motor) in the holder's fork, rotor (the turning shell) with `j1_link` |
| `elbow_pitch` | revolute, `N` | `upper_arm_link → elbow_link` | GT2 90T belt, NEMA 17 x 40 + MKS SERVO42D (`nema17_40mm#2` + `mks_servo42d#2` on the web across `j1_link`'s motor hole, on the arm's +N side, `ELBOW_MOTOR_CENTRES` from the elbow) | the pulley carries the roll drive's block, which is the elbow coupler (`j3_coupler#1` retired) |
| `forearm_roll` | revolute, along the forearm through the wrist centre | `elbow_link → forearm_link` | GT2 90T ring on the hollow roll shaft, NEMA 17 x 40 + MKS SERVO42D on the motor mount bolted to the elbow block's top (`assemblies/forearm_roll_drive.py`) | the shaft's end spigot bolts to `j2_link`'s wall 48 mm from the elbow axis; hard stop ±`FOREARM_ROLL_LIMIT_DEG`; specs `docs/forearm_roll.md` §0 |
| `wrist_pitch` | revolute, `N` | `forearm_link → wrist_pitch_link` | GT2 90T belt, NEMA 17 x 40 + MKS SERVO42D (`nema17_40mm#3` + `mks_servo42d#3` on `j2_link`'s web slots) | pulley + J3 coupler on the wrist body |
| `wrist_roll` | revolute, `F` | `wrist_pitch_link → wrist_roll_link` | NEMA17 pancake + 20T pulley (not CAN-driven) | |
| `jaw_a`, `jaw_b` (mimic, −1) | prismatic | `wrist_roll_link → jaw_*_link` | MG996R crank linkage | `open` / `closed` SRDF states |
| `tool0_joint` | fixed | `wrist_roll_link → tool0` | — | fingertip midpoint, Z = approach |

| link | occurrences (`robot/frames.py LINKS`) |
|---|---|
| `base_link` | `base` + `bearing_6806#1`, `#2` (the base_yaw pair in its bore) + `washer_as6590#1`, `bearing_axk6590#1` (the thrust bearing's lower washer and cage in its groove) + `base_motor_mount#1` (the motor's box, bolted to its end) + its M4 screws + nuts (`base_motor_mount_screws#1`, `base_motor_mount_nuts#1`) + `nema17_48mm#1`, `mks_servo42d#1` (the base_yaw 48 mm motor + board under the mount's plate) |
| `shoulder_link` | `j1_coupler` + `washer_as6590#2` (the thrust bearing's upper washer, under its seat) + `gt2_pulley_120t#1` (the base_yaw 120T on its stub's end, inside the base) + its M4 screws + nuts (`yaw_pulley_screws#1`, `yaw_pulley_nuts#1`) + the drive's **stator** (`cycloidal_drive#1:stator`: the held carrier - motor plate, output hub + its pins + the 625 - NEMA 17 + its MKS board, the gear train) |
| `upper_arm_link` | the drive's **rotor** (`cycloidal_drive#1:rotor`: the shell ring, ring pins, housing bolts/nuts, both 6814s) + `j1_link` (the arm rising off the shell, printed with the shell's body) + `bearing_6806#3`, `#4` (the elbow pair in its bore) + `nema17_40mm#2`, `mks_servo42d#2`, `gt2_pulley_20t#2` (the elbow_pitch motor + board on its plate, the 20T on its shaft) |
| `elbow_link` | `gt2_pulley_90t#3` (re-seated, `lib/mounts.py`) + its M4 screws + nuts (`elbow_pulley_screws#1`, `elbow_pulley_nuts#1`) + the roll drive's **stator** (`forearm_roll_drive#1:stator`: the elbow block — the elbow coupler and the housing in one, `j3_coupler#1` retired —, 2× 6808, the end cap, the bolt-on motor mount + its M3 screws and nuts, its NEMA 17 x 40 + MKS board, the 20T) |
| `forearm_link` | the roll drive's **rotor** (`forearm_roll_drive#1:rotor`: the hollow roll shaft with its 90T ring and end spigot) + `j2_link` + `bearing_6806#5`, `#6` (the wrist pair in its boss) + `nema17_40mm#3`, `mks_servo42d#3` (the wrist_pitch motor + board on the web) |
| `wrist_pitch_link` | `wrist_link`, `gripper_clamp_bracket`, `nema17_pancake`, `gt2_pulley_90t#4` (re-seated) + its M4 screws + nuts (`wrist_pulley_screws#1`, `wrist_pulley_nuts#1`), `j3_coupler#2` |
| `wrist_roll_link` | `gt2_pulley_20t` + the gripper base (connector, servo holder, servo + horn, cover, rails, crank links) |
| `jaw_a_link` / `jaw_b_link` | slider + two fingers + end, each side |

## Commands
```bash
./cadtool python tools/robot/derive.py                 # joint origins + link inertials (m, kg, rad)
./cadtool python tools/robot/derive.py --check robot/arm.urdf robot/arm.sdf   # files vs CAD (tests run this)
./cadtool python tools/robot/export_link_meshes.py           # regenerate meshes after converting a part
./cadtool validate robot/arm.urdf --strict             # also .srdf / .sdf --gz-check never
./cadtool snapshot robot/arm.urdf snapshots/arm_urdf.png --joint-values '{"shoulder_pitch": 45}'   # posed stills
./cadtool viewer                                       # then ?file=robot/arm.urdf: meshes + joint sliders (base_yaw, shoulder_pitch, elbow_pitch, forearm_roll, wrist_pitch, wrist_roll, jaw_a)
```

The `shoulder_pitch` slider turns the drive's rotor with `j1_link` while its housing stays with `j1_coupler`; the
`forearm_roll` slider turns the forearm, wrist and gripper about the forearm while the elbow block stays with the elbow
pulley. The drives on their own: `?file=assemblies/cycloidal_drive.step` (see `docs/cycloidal_drive.md`, "Viewing the
drive") and `?file=assemblies/forearm_roll_drive.step` (`docs/forearm_roll.md`).
