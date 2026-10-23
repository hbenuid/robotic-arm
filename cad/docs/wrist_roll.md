# The wrist roll drive — the pancake on the wrist-pitch axis

**Purpose:** spec and attachment of the belt-driven wrist roll (`wrist_roll`, between `wrist_pitch` and the gripper),
the forearm roll drive's pattern (`forearm_roll.md`) one joint further out. Code: `lib/wrist/` (`params.py
WristRollParams` + `ConnectorParams`, `layout.py stack_positions`, `roll.py`, `connector.py`, `link.py` - the frame on
`wrist_link`'s plate), `assemblies/wrist_roll_drive.py`, `parts/wrist/wrist_roll_*.py`, `parts/wrist/gripper_j3_connector.py`;
tests `tests/wrist/`. Numbers below name the constants; the values live in `lib/wrist/params.py`.

## 0. Specifications at a glance
| item | value | where |
|---|---|---|
| joint | `wrist_roll`, revolute, `wrist_pitch_link → wrist_roll_link`; axis R′ parallel to the SolidWorks roll axis, `WRIST_ROLL_OFFSET` (the belt's centre distance) across the `wrist_pitch` axis in the wrist's swing plane and `WRIST_ROLL_AXIS_H` over the plate's coupler face - **a wrist offset: the last three axes are not concurrent** | `robot/frames.py WRIST_ROLL_ORIGIN`, `lib/placements.py SHIFTS` |
| range | ±`WRIST_ROLL_LIMIT_DEG`, 2° short of the printed hard stop (a lug on the shaft's flange, a post in the frame's bay - the forearm roll's) | `lib/params.py`, `WristRollParams stop_*`, `tests/test_sweeps.py` |
| drive | the NEMA 17 pancake **on the `wrist_pitch` axis** (centred on it along R′) with its **MKS SERVO42D** on its rear face (a CAN id of its own), its 20T on the shaft, the 40T band **integral to the pulley - the output**, **2 : 1** (`WRIST_ROLL_RATIO`), the stock 2GT × 6 mm belt `WRIST_ROLL_BELT_LENGTH` at its centre distance with the pancake at its slots' middle - **no idler yet** (`open_issues.md`) | `WristRollParams`, `lib/belts.py` |
| bearings | 2× 6806-2RS back to back on a 2 mm lip in the frame's tower, bearing 2 right under the band (the forearm roll's and the belt joints' bearing, `lib/bearings.py`) | `WristRollParams bearing_*` |
| pulley (rotor, PETG - the output) | its hub a journal through bearing 2 to the lip's middle, the Ø33 shoulder on bearing 2's inner ring, the 40T band (two flanges) on a core inside its root, the face `gripper_j3_connector` bolts to (4× M3 into nuts in pockets from the face's rim); the rotor clamp's 4× M3 heads sunk in the face | `wrist_roll_pulley` |
| shaft (rotor, PETG) | its hub through bearing 1 to the pulley's, the Ø33 shoulder, the flange with the clamp's 4 nuts (pockets open to its rear face) and the stop lug; the cable bore through both pieces | `wrist_roll_shaft` |
| rotor clamp | 4× M3 × `clamp_screw_len` on `clamp_circle_dia` inside the band's core, from the pulley's face through both hubs into the shaft's nuts: the pulley, both inner rings and the shaft one block | `WristRollParams clamp_*` |
| frame (stator, PETG) | `wrist_link` itself: its plate (j3_coupler#2's seat on the pitch axis) and on the plate's underside - the side away from the forearm - one body: the pancake's **pocket** centred on the pitch axis (its floor the plate, open on the far face; no wall behind it: the board's **window** - its housing square + the tension travel + `board_clear` - runs out through the frame's rear, open on the far face too), its front wall the **motor plate** the pancake bolts to (`pad_t`; slotted along Y: the belt's tension, the shaft's slit out to the open face), the **tower** round R′ beside it (the 6806 seats, the lip, the **bearing boss** on the front face), the shaft's **bay** behind bearing 1 open on the far face (the shaft goes in there, the cable comes out), the stop's **post** in it; the coupler nut the tower covers drops in from the bay through a hex channel | `lib/wrist/roll.py build_frame`, `lib/wrist/link.py` |
| gripper interface | `gripper_j3_connector` (converted): its plate (the servo holder's and the cover's holes) on a hub bolted to the pulley's face - a key reaches every head past the plate's flats, notches in the flange over the plate's holes; the cable bore through it | `ConnectorParams` |
| clearances held by tests | the bearings' presses on the journals (0.9–1.0 × the nominal `journal_add` press), every other pair in the module and against `wrist_link` < 1 mm³, the rotor `run_gap` off the frame, the pancake's and its board's way in from the open face (`motor_drop` back, the slit, the window) and their slots' travel (the board `board_clear` off the window), the board's screws through the pancake's own (`KIT_OVERLAP`), its plugs' room (`MKS_PLUG_ROOM`) at every pose, the rotor's walls round the clamp and the bore, the gripper 1 mm off the wrist and the forearm while it rolls, the wrist-pitch limits | `tests/wrist/test_roll_drive.py`, `tests/test_sweeps.py` |
| link masses (URDF) | `wrist_pitch_link` carries `wrist_link` (the frame) and the stator (the pancake + its board, its 20T, the belt, both bearings), `wrist_roll_link` the rotor + the gripper | `robot/arm.urdf` |
| purchased per drive | what to order, with quantities: `./cadtool python tools/bom.py --module wrist_roll_drive` (its BUY list, and BUY, NOT MODELLED from `tools/bom.py EXTRAS`) | `tools/bom.py` |
| printed per drive | the pulley, the shaft; the frame is `wrist_link` | `print/` |

## 1. Why, and where
The SolidWorks wrist hung the pancake and `gripper_clamp_bracket` on the roll axis past `wrist_link`'s tower, the gripper
beyond them, all cantilevered off the pitch axis. The elbow already showed the other way: the forearm roll's motor sits ON
the elbow axis, belted to a roll axis beside it. Done the same at the wrist, the pancake centres on the pitch axis, on
the plate's free face (the forearm's boss and the coupler fill the other side), and belts to R′ beside it; the gripper
rolls on R′, `WRIST_ROLL_OFFSET` across the pitch axis and `WRIST_ROLL_AXIS_H` off the plate. The gripper comes nearer the
pitch axis and the wrist side's centre of mass with it, so the wrist-pitch motor carries less (the commit that made it
quotes the numbers); in exchange the wrist is not spherical - the wrist roll axis misses the pitch axis and the
forearm roll's - and an inverse kinematics in `software/control` must be numeric (`open_issues.md`).

**Which side.** R′ sits on `wrist_link`'s −Y (`WristRollParams.side`): the outer side, away from the upper arm when the
elbow folds (the forearm turned), so the wrist pitch's long range bends the gripper down. The frame is the other side's
mirror image; the rotor and every bought part are placed, not mirrored. Bending up, the frame's tower soon meets the
wrist-pitch motor on the forearm; bent far down, the gripper reaches back toward the elbow - the forearm roll's frame
there, and the upper arm once the elbow folds - so `WRIST_PITCH_LIMITS_DEG` keeps the gripper off those at the elbow's
zero, and folded further its clearance is the planner's (`ELBOW_GRIPPER_FREE_DEG`, `tests/test_sweeps.py`,
`open_issues.md`). The first build had R′ on +Y, the inner side: its long range bent the gripper up.

## 2. Layout (module frame)
Module frame (`lib/wrist/layout.py module_frame_in_host`, `lib/mounts.py MODULE_MOUNTS` on `wrist_link#1`, turned with
it): origin on R′ at the pitch axis' station; **+Z along R′ toward the gripper** (wrist_link's +X); **+X away from the
plate** (the open face, wrist_link's −Z); **+Y = wrist_link's +Y** (R′ `side` of the pancake: −Y); the pitch axis runs
along X at `y_motor`.
Every station: `stack_positions()`.

| station | what | fixed by |
|---|---|---|
| `z_board_back` … `z_motor_rear` | the MKS board on the pancake's rear face, turned with it (its motor plug at the open face), in `board_window()` - from the frame's rear `z_rear` (the tower's) to `board_clear` in front of that face | `board_stack`, `board_clear` |
| `z_motor_rear` … `z_motor_face` | the pancake, centred on the pitch axis' station, its plug spun to +X | the pitch axis |
| `z_motor_face` … `z_front` | the motor plate: the 4 bolt slots and the pilot slot (±`pad_slot_len`/2 along Y), the shaft's slit | `pad_t` |
| `z_20t` … | the 20T, its hub `pulley_seat` over the pilot, its band at `z_ring_mid` | `t20` |
| `z_ring` … `z_ring_end` | the 40T band between its flanges, level with the 20T's | the 20T |
| `z_ring_end` … `z_face` | the pulley's face: the connector's nuts, the clamp's heads | `face_t` |
| `z_boss` | bearing 2's front face, the bearing boss's top, `run_gap` under the band's flange | the band |
| `z_bearing_2`, `z_lip`, `z_bearing_1` | the seats and the lip; the hubs meet at `z_meet` | the bearings |
| `z_shaft_end` … `z_shoulder_1` | the shaft's flange: the clamp's nuts, the stop lug | the clamp's tips |
| `z_bay` … `z_bearing_1` | the bay, open on +X, the stop post on the tower's rear face | the shaft's length + `bay_clear` |
| `x_coupler`, `x_free` | the plate's coupler face, its free face (the pocket's floor) | `axis_h` |

## 3. Fits and printing
PETG, the forearm roll's rules: the seats the bearing OD + `seat_add`, the journals `journal_add` interference (print a
fit gauge first, `cycloidal_drive.md` §6). `wrist_link` prints plate down (its coupler face on the bed: the frame stands
on it, the pocket and the bay open at the top) - the tower's seats then print as horizontal bores, their tops bridged:
unchecked (`open_issues.md`, a `dfam-check` before the first print); the pulley face down; the shaft rear face down (its
nut pockets open to the bed).

## 4. Assembly sequence
1. The coupler nut under the tower into its hex channel from the bay; `j3_coupler#2` onto the plate's seat as before.
2. Bearing 2 onto the pulley's hub, bearing 1 onto the shaft's. The pulley + bearing 2 into the tower from the front; the
   shaft + bearing 1 into the bay through the open face, then forward into its seat.
3. Four M3 nuts into the shaft's flange from its rear face (in the bay); 4× M3 × `clamp_screw_len` from the pulley's
   face through both hubs into them: the rotor turns, whole.
4. The MKS board onto the pancake's rear face - its screws in place of the motor's own rear four (their length on a
   pancake: `open_issues.md`) - and its motor cable into the pancake's connector; the pancake into its pocket from the
   open face `motor_drop` back - its pilot behind the motor plate, its shaft along the slit, the board through the window
   behind - then forward into the pilot slot; 4× M3 from the motor plate's front face. The 20T on its shaft, its hub
   on a `pulley_seat` shim on the pilot; the belt round it and the 40T; slide the pancake along its slots until the belt
   is tight, tighten.
5. The gripper servo's cable through the connector's bore, the rotor's bore, out of the bay.
6. Four M3 nuts into the pulley face's pockets from its rim; the connector onto the face, 4× M3 × `bolt_len` from
   between its flange and its plate; then the servo holder and the cover onto its plate (their screws through the
   flange's notches).

## 5. Attachment to the arm (`robot/`)
`wrist_pitch_link` = `wrist_link#1` (the frame) + `gt2_pulley_90t#4` and its bolts + `j3_coupler#2` +
`wrist_roll_drive#1:stator`; `wrist_roll_link` = `wrist_roll_drive#1:rotor` + the gripper (the connector, the servo
holder, …). `Joint("wrist_roll")` keeps its direction (the capture's roll axis, turned with the forearm); its origin is the
capture's on that axis carried by the gripper's shift onto R′ (`robot/frames.py WRIST_ROLL_ORIGIN`). `nema17_pancake#1` and
`gt2_pulley_20t#1` are retired (`lib/placements.py RETIRED`), `gripper_clamp_bracket` dropped (`lib/reference.py
SKIPPED_PRODUCTS`); the gripper moves by `lib/placements.py SHIFTS`' `wrist_roll_drive` shift (along, across, and off
`wrist_link`'s plate).

## 6. Not modelled / to confirm
`open_issues.md`: the belt's tensioner (none yet: the pancake's slots tension it), the frame's flex under the gripper's
load and the belt's pull, the 6806 fits in PETG, the cable's route through the rotor, a home sensor (as the forearm roll's,
not designed), the pancake's exact model and mass, the board's screw length on it, the wrist offset in a future IK.
