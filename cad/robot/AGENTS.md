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
  point/direction in the SolidWorks capture frame — the points beyond a link the design changed moved with it,
  `lib/placements.py SHIFTS` via `P.shifted()` — limits from `lib/params.py`). Frames are REP-103 (`base_link` on
  the base's bottom face at the base_yaw axis, Z up, X forward). Joint frame: Z on the axis, X along the child link
  (`forearm_roll`: X = N, its child lies along the axis); child link frame = joint frame at capture, so **all joints
  are 0 at the capture pose**, mesh origins are identity and the URDF at zero reproduces `assemblies/arm.py` (emitted
  in the same `base_link` frame — `assemblies/AGENTS.md`). The capture pose is the design's: the SHIFTS applied and
  the forearm turned half a turn about its roll axis (`lib/placements.py TURNS`), so the directions beyond the roll
  (`WRIST_PITCH_AXIS` = the capture's −N, `WRIST_ROLL_AXIS`, `JAW_TRAVEL`) are turned with it and at zero the gripper
  is rolled `FOREARM_ROLL_TURN_DEG` from the SolidWorks capture. Moving an occurrence between links or changing an axis =
  edit `frames.py` (a moved occurrence also in `assemblies/arm.py GROUPS`, which mirrors `LINKS` - `test_assembly.py`;
  a mounted one's `Mount.link` in `lib/mounts.py`; a capture record in the `lib/placements.py SHIFTS` lists -
  `test_placements.py`), re-export the meshes, re-derive the affected `<origin>` / `<inertial>` values from the
  drafts (the checked-in XML stays canonical), re-check.
- `robot/links/<link>.py` are `@step` models of each rigid link (`robot/_links.build_link`, in the link
  frame; `robot/links/<link>.step` git-ignored). Meshes: `tools/robot/export_link_meshes.py` (mm STL per
  link via build123d's `export_stl`, `scale 0.001` in the XML); never hand-edit.
  Inertials come from OCP `BRepGProp` (printed parts at `PETG_DENSITY`, COTS at `MASS_G`).
- Validate (`--strict`) and snapshot after every edit; hand `.urdf` files to the viewer (`?file=robot/arm.urdf`) —
  Commands below.
- What is still an estimate or unconfirmed here - the `[ESTIMATE]` limits (`BASE_YAW_LIMIT_DEG`, the shoulder's lower
  one), efforts / velocities and jaw travel (`lib/params.py`), the axis signs (`robot/frames.py`), the link-membership
  assumptions the URDF ledger lists, which CAN id drives which joint: `docs/open_issues.md`; wrist_roll and the jaws
  are not driven by `software/control/src/config.py`.
- **The reductions the software must match** (`lib/params.py`): `BASE_YAW_RATIO` (base_yaw: the 120T, the
  motor's 20T and the belt), `CYCLOIDAL_RATIO` (shoulder_pitch: the drive's shell
  turns, the same way as its motor - the port's hub turned against it), `GT2_RATIO` (the wrist_pitch and the
  elbow_pitch belts, one stage each: the elbow motor's 20T straight to the 90T, `docs/open_issues.md`),
  `FOREARM_ROLL_RATIO` (forearm_roll), `WRIST_ROLL_RATIO` (wrist_roll). They are motor revolutions per output revolution, while
  `software/control/src/config.py`'s `gear_ratio` is output revolutions per motor revolution — the software's value
  is the reciprocal (`1 / CYCLOIDAL_RATIO`, …).

## Joints and links
`base_link → base_yaw → shoulder_link → shoulder_pitch → upper_arm_link → elbow_pitch → elbow_link →
forearm_roll → forearm_link → wrist_pitch → wrist_pitch_link → wrist_roll → wrist_roll_link → jaw_a / jaw_b
(prismatic, jaw_b mimics jaw_a) + tool0` (frame-only) — six revolute joints with an elbow and a wrist offset: the
last three are NOT concurrent (`forearm_roll`'s axis runs along the forearm through `WRIST_CENTRE` on the `wrist_pitch`
axis, `docs/forearm_roll.md`; `wrist_roll`'s runs `WRIST_ROLL_OFFSET` across it, `docs/wrist_roll.md`) - an inverse
kinematics must be numeric. The drives ARE joints: each one's stator stays in the parent link, its rotor moves with the child (`BODIES` in
`assemblies/cycloidal_drive.py` / `forearm_roll_drive.py` / `wrist_roll_drive.py`; the URDF ledger, `docs/cycloidal_drive.md` §12). The belt
joints' motors are mounted occurrences (`lib/mounts.py`, `assemblies/AGENTS.md`); the drives' motors are module rows.

| joint | type | parent → child | actuator | notes |
|---|---|---|---|---|
| `base_yaw` | revolute, world up | `base_link → shoulder_link` | GT2 120T belt (`BASE_YAW_RATIO`), NEMA 17 x 48 + MKS SERVO42D (`nema17_48mm#1` + `mks_servo42d#1` under the plate of the base's bolt-on motor mount, `BASE_MOTOR_TABLE_CLEAR` above the base's bottom face) | the holder `j1_coupler` turns on the base, standing on the thrust bearing in its groove, held down by the pulley bolted to its stub's end under the lower bearing |
| `shoulder_pitch` | revolute, `N` | `shoulder_link → upper_arm_link` | the cycloidal drive (`CYCLOIDAL_RATIO`), its own NEMA 17 | stator (the held carrier + motor) in the holder's fork, rotor (the turning shell) with `j1_link` |
| `elbow_pitch` | revolute, `N` | `upper_arm_link → elbow_link` | GT2 90T belt, NEMA 17 x 40 + MKS SERVO42D (`nema17_40mm#2` + `mks_servo42d#2` on the web across `j1_link`'s motor hole, on the arm's +N side, `ELBOW_MOTOR_CENTRES` from the elbow) | the pulley carries the roll drive's frame, whose web is the elbow coupler (`j3_coupler#1` retired) |
| `forearm_roll` | revolute, along the forearm through the wrist centre, `ELBOW_ROLL_OFFSET` across the elbow axis (an elbow offset: the axes do not cross) | `elbow_link → forearm_link` | GT2 90T ring integral to the roll pulley (the output), NEMA 17 x 40 + MKS SERVO42D on the elbow axis, in the roll frame's pocket on its plate (`assemblies/forearm_roll_drive.py`) | `j2_link`'s wall bolts onto the pulley's ring, its faces `FOREARM_WALL_X` along the roll axis from the elbow axis' station; the printed hard stop just past ±`FOREARM_ROLL_LIMIT_DEG`; specs `docs/forearm_roll.md` §0 |
| `wrist_pitch` | revolute, `WRIST_PITCH_AXIS` (the capture's −N) | `forearm_link → wrist_pitch_link` | GT2 90T belt, NEMA 17 x 40 + MKS SERVO42D (`nema17_40mm#3` + `mks_servo42d#3` on `j2_link`'s web slots) | pulley + J3 coupler on the wrist body |
| `wrist_roll` | revolute, `WRIST_ROLL_AXIS` (the capture's `F`, turned), `WRIST_ROLL_OFFSET` across the `wrist_pitch` axis (a wrist offset) | `wrist_pitch_link → wrist_roll_link` | GT2 belt (`WRIST_ROLL_RATIO`), the NEMA17 pancake + MKS SERVO42D ON the `wrist_pitch` axis, in `wrist_link`'s pocket, the board in the frame's rear window (`assemblies/wrist_roll_drive.py`) | the gripper's connector bolts to the roll pulley's face; the printed hard stop just past ±`WRIST_ROLL_LIMIT_DEG`; specs `docs/wrist_roll.md` §0 |
| `jaw_a`, `jaw_b` (mimic, −1) | prismatic | `wrist_roll_link → jaw_*_link` | MG996R crank linkage | `open` / `closed` SRDF states |
| `tool0_joint` | fixed | `wrist_roll_link → tool0` | — | fingertip midpoint, Z = approach |

| link | occurrences (`robot/frames.py LINKS`) |
|---|---|
| `base_link` | `base` + `bearing_6806#1`, `#2` (the base_yaw pair in its bore) + `washer_as6590#1`, `bearing_axk6590#1` (the thrust bearing's lower washer and cage in its groove) + `base_motor_mount#1` (the base's other half, bolted to its end) + its M4 screws + nuts (`base_motor_mount_screws#1`, `base_motor_mount_nuts#1`) + `nema17_48mm#1`, `mks_servo42d#1` (the base_yaw 48 mm motor + board under the mount's plate) + `gt2_pulley_20t#3`, `yaw_belt#1` (its 20T and the belt to the 120T) + `gt2_idler_20t#2`, `yaw_idler_spacer#1`, `yaw_idler_screw#1`, `yaw_idler_nut#1` (the belt's tensioner on the mount's plate) |
| `shoulder_link` | `j1_coupler` + `j1_motor_leg#1` (its motor-side leg, a ring round the drive's motor sleeve, bolted to its disc and to the sleeve) + `washer_as6590#2` (the thrust bearing's upper washer, under its seat) + `gt2_pulley_120t#1` (the base_yaw 120T on its stub's end, inside the base) + its M4 screws + nuts (`yaw_pulley_screws#1`, `yaw_pulley_nuts#1`) + the fork's M4 screws + nuts that hold the drive's hub and sleeve in it (`fork_hub_screws#1`, `fork_hub_nuts#1`, `fork_foot_screws#1`, `fork_foot_nuts#1`, `fork_sleeve_screws#1`, `fork_sleeve_nuts#1`) + the drive's **stator** (`cycloidal_drive#1:stator`: the held carrier - motor plate, output hub + its pins + the 625 - NEMA 17 + its MKS board, the gear train) |
| `upper_arm_link` | the drive's **rotor** (`cycloidal_drive#1:rotor`: the shell ring, ring pins, housing bolts/nuts, both 6814s) + `j1_link` (the arm rising off the shell, printed with the shell's body) + `bearing_6806#3`, `#4` (the elbow pair in its bore) + `nema17_40mm#2`, `mks_servo42d#2`, `gt2_pulley_20t#2`, `elbow_motor_screws#1`, `elbow_belt#1` (the elbow_pitch motor + board on the web across `j1_link`'s motor hole, the 20T on its shaft, the motor's 4x M3 up through the web, the belt to the elbow 90T) + `gt2_idler_20t#1`, `elbow_idler_spacer#1`, `elbow_idler_screw#1`, `elbow_idler_nut#1` (the belt's tensioner under the slab) |
| `elbow_link` | `gt2_pulley_90t#3` (re-seated, `lib/mounts.py`) + its M4 screws + nuts (`elbow_pulley_screws#1`, `elbow_pulley_nuts#1`) + the roll drive's **stator** (`forearm_roll_drive#1:stator`: the roll frame — one body round the motor, its underside the elbow coupler, `j3_coupler#1` retired —, 2× 6806, its NEMA 17 x 40 + MKS board on the elbow axis and its 4x M3 (`roll_motor_screws`), the 20T, the roll belt and its tensioner) |
| `forearm_link` | the roll drive's **rotor** (`forearm_roll_drive#1:rotor`: the pulley - its 90T ring the output, its spigot in the wall - and the shaft clamped to it) + `j2_link` + `bearing_6806#5`, `#6` (the wrist pair in its boss) + `nema17_40mm#3`, `mks_servo42d#3`, `gt2_pulley_20t#4`, `wrist_motor_screws#1`, `wrist_belt#1` (the wrist_pitch motor + board on the web, its 20T, the motor's 4x M3 up through the web, the belt) + `gt2_idler_20t#3`, `wrist_idler_screw#1`, `wrist_idler_nut#1` (the belt's tensioner under the web) |
| `wrist_pitch_link` | `wrist_link` (the wrist roll drive's frame) + `gt2_pulley_90t#4` (re-seated) + its M4 screws + nuts (`wrist_pulley_screws#1`, `wrist_pulley_nuts#1`), `j3_coupler#2` + the wrist roll drive's **stator** (`wrist_roll_drive#1:stator`: the pancake + its MKS board on the pitch axis, its 20T, the belt, 2× 6806) |
| `wrist_roll_link` | the wrist roll drive's **rotor** (`wrist_roll_drive#1:rotor`: the pulley - its 40T band, the face the connector bolts to - and the shaft clamped to it) + the gripper base (connector, servo holder, servo + horn, cover, rails, crank links) |
| `jaw_a_link` / `jaw_b_link` | slider + two fingers + end, each side |

## Commands
```bash
./cadtool python tools/robot/derive.py                 # joint origins + link inertials (m, kg, rad)
./cadtool python tools/robot/derive.py --check robot/arm.urdf robot/arm.sdf   # files vs CAD (tests run this)
./cadtool python tools/robot/export_link_meshes.py --links <link> ...   # the meshes of the links a change touched (Recipe C step 8)
./cadtool validate robot/arm.urdf --strict             # also .srdf / .sdf --gz-check never
./cadtool snapshot robot/arm.urdf snapshots/arm_urdf.png --joint-values '{"shoulder_pitch": 45}'   # posed stills
./cadtool viewer                                       # then ?file=robot/arm.urdf: meshes + joint sliders (base_yaw, shoulder_pitch, elbow_pitch, forearm_roll, wrist_pitch, wrist_roll, jaw_a)
```

The `shoulder_pitch` slider turns the drive's rotor (the turning shell) with `j1_link` while its stator (the held
carrier + motor) stays with `j1_coupler`; the `forearm_roll` slider turns the forearm, wrist and gripper about the
forearm while the roll frame stays with the elbow pulley; the `wrist_roll` slider turns the gripper about the axis
beside the pancake. The drives on their own:
`?file=assemblies/cycloidal_drive.step` (see `docs/cycloidal_drive.md`, "Viewing the drive"),
`?file=assemblies/forearm_roll_drive.step` (`docs/forearm_roll.md`) and `?file=assemblies/wrist_roll_drive.step`
(`docs/wrist_roll.md`; its frame is `wrist_link`).
