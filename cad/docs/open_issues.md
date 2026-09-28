# Open issues — what the CAD knows is not settled

**Purpose:** the ONE list of unsettled things a session should know before trusting a number or a fit: fit problems
the model carries knowingly, `[ESTIMATE]` values waiting for a measurement, hardware not modelled yet, mappings
not confirmed, parts not converted yet. **Rule:** when you flag something in a commit, add a row (issue, where it lives, what closes it,
the commit that raised it); when you close it, delete the row and say so in the commit message. A number here is
a fit value or an estimate, quoted beside the constant or test that holds it — the code wins if they drift; counts
and totals are never quoted (`cad/CLAUDE.md` Docs).

## Fit problems the model carries knowingly
| issue | where it lives | what closes it | raised |
|---|---|---|---|
| The base_yaw joint is not held down: `j1_coupler` stands on the thrust bearing (its seat `THRUST_STACK` above the groove's floor, the upper 6806 free under it - no shoulder by design, the stack sets the height), but nothing pulls it down onto it until the base_yaw 90T is modelled: bolted to the stub's end on the lip's lower face (its ring on the lower inner ring, the lip on the outer), it would close the loop, and the lower 6806 holds nothing until then; the stack's height against that loop wants a shim on assembly | `lib/mounts.py` `bearing_6806#1` / `#2`, `THRUST_MOUNTS`; `tests/test_mounts.py test_bearing_stacks` checks only the lip at the base, `test_thrust_bearing_carries_j1_coupler` the stack; the stub is ready (`lib/yaw_coupler/params.py` DEFAULT: Ø`BEARING_6806_BORE` to the lip's lower face, drilled for the 90T at the diagonals) | model the base_yaw pulley (Not modelled, below) | 2026-09-25 (`git log --grep 6806-2RS`) |
| The cycloidal drive is **not fastened to `j1_coupler`**: the cheek's 3 nut pockets and holes (the bottom housing bolt and those at +/-45 degrees, on the housing's bolt circle) are empty - the drive's housing bolts end in its own nuts in the ring gear body, 0.5 short of the cheek; the housing only rests in the cradle, kept from turning by the V-grooves round its pillars, and its axis sits 0.21 off the cradle's (placements.json), past the 0.2 a side an M4 has in the cheek's Ø4.4 holes | `lib/yaw_coupler/params.py YokeParams` (`bolt_*`, `nut_*`), `lib/cycloidal/params.py HousingParams.bolt_length`, `reference/placements.json cycloidal_drive#1` | those 3 bolts long enough to reach the cheek's nuts (M4 x 60: 0.2 proud of its outer face) with the drive re-seated on the cradle's axis - or another way to hold the housing down | 2026-09-27 (`git log --grep yaw_coupler`) |
| The drive motor's tie rods are not modelled (the MKS kit's M3x30 replace them); bolt shanks overlap the export's tapped holes in the model (46 / 159 mm³, thread engagement) | `tests/cycloidal/test_assembly.py` interference budget | nothing — a modelling representation; re-measure if the vendor file changes | 2026-09-21 (`3e667e0`) |
| The roll motor's plate (`pad_t`, the mount's one thickness) is braced only by its root in the motor mount's base (the cheeks that stiffened it are gone): the belt's pull, `t20` in front of the plate, and the motor hanging behind it bend that root | `lib/forearm/params.py RollDriveParams pad_t`, `lib/forearm/roll.py build_motor_mount` | check the plate for flex / creep under belt tension on the print; thicken `pad_t` or add a root fillet / gussets outside the motor's footprint if it gives | 2026-09-27 (`git log --grep forearm_roll_motor_mount`) |
| `j3_coupler`'s pulley-bolt holes are Ø4.1 (the SolidWorks pattern) for the wrist 90T's M4 shanks: 0.05 mm a side in PETG, and printed holes come out small (the 90T's own were opened to `M4_CLEAR`) | `lib/coupler/params.py CouplerParams.pulley_bolt_dia` | drill them Ø4.2 or open DEFAULT's to `M4_CLEAR` | 2026-09-26 (`git log --grep wrist_pulley_screws`) |

## Estimates to confirm on the hardware (`[ESTIMATE]` in `lib/params.py` unless noted)
| value | where | how to confirm |
|---|---|---|
| The thrust bearing's fits in PETG: the seat ring `RING_CLEAR` a side inside the cage's and washers' bore, the coupler's recess `THRUST_CLEAR` round them; its masses (`THRUST_CAGE_MASS_G`, `THRUST_WASHER_MASS_G`: distributor listings) | `lib/base/params.py`, `lib/yaw_coupler/params.py`, `lib/bearings.py` | print a fit gauge first (docs/cycloidal_drive.md §6); weigh them, then Recipe C step 7 |
| `NEMA17_40_MASS_G` 280, `MKS_SERVO42D_MASS_G` 35, `CYCLOIDAL_MOTOR_MASS_G` 400 (`[DATASHEET]` class value), `PANCAKE_MASS_G` 180 | `lib/params.py` | weigh; then Recipe C step 7 (inertials) |
| The 40 mm kit motors' real shaft length (the export said 23; the model carries the drive's 22) and the 48 mm one's (datasheet 17HS19-2004S1: 24 ± 1 with a 15 mm D-cut; the drive assumes 22 / 18 by ruling — `reference/cycloidal/nema17_48mm.step`) | `lib/cycloidal/params.py MotorParams`, `tools/reference/split_mks_motor.py` | measure from the mounting face; a 24 mm shaft bottoms the eccentric shaft's D-bore (`d_bore_depth` 14 → 16) |
| Connector / cable direction of each mounted motor (the spin about its axis) | `lib/mounts.py` frames, `note` fields | decide on the bench; change the `rz`, run `mount_placements.py`, Recipe C |
| The wrist-pitch motor's position on `j2_link`'s slide, `J2_MOTOR_SLIDE_X` = −136.37: what a stock **264-2GT** belt sets (`lib/forearm/params.py RollEndParams.wrist_belt`, `lib/belts.py`) - the belt length itself is the estimate | `lib/forearm/params.py` | confirm the belt on the hardware; 260-2GT would put it at −138.5 |
| Joint limits, efforts, velocities, axis signs, jaw travel | `lib/params.py` `*_LIMIT_DEG`, `ARM_JOINT_*`, `JAW_*` | viewer sweeps + hardware; `robot/arm.urdf` follows via `derive.py --check` |
| The roll belt, 240-2GT (`RollDriveParams.roll_belt`) - it sets the roll motor's centre distance (60.9) and so its height above the block (its MKS board 3.4 mm above the top - the motor mount's, level with the block's - at the nominal slot position, 0.9 at the slot's low end) | `lib/forearm/params.py` | confirm on the hardware; the plate's slots give +/- 2.5 mm |
| `FOREARM_ROLL_LIMIT_DEG` = 170 (the hard stop: the shaft's lug on its neck against the end cap's post - printed, 10° wide each, 1 mm of axial overlap) | `lib/params.py`, `lib/forearm/params.py stop_*` | the overlap and widths on the print; the lugs are small - a steel pin if PETG shears |
| 6808-2RS: mass 33 g, inner-race OD ≈ 44.5 (the shaft's Ø44 shoulders must not touch the outer race) | `lib/params.py BEARING_6808_MASS_G`, `RollDriveParams.inner_race_od` | datasheet / calipers on the bearing in hand |
| 6806-2RS: mass 26 g (the 6808's, scaled by the ring area); the inner ring's outer edge against the Ø33 shoulders on the stubs and the 90T's Ø34.76 ring (both must bear on the inner ring only) | `lib/bearings.py` | datasheet / calipers on the bearing in hand |
| The 6806 seats in PETG: Ø42.2 (the base, `j1_link`'s lower seat, `j2_link`'s two), `j1_link`'s upper seat Ø42.0 (line-to-line); the Ø30 stubs and hubs in the inner rings (line-to-line); the M4 nuts' hex channels / pockets, 6.85 across flats for a 7.0 nut (the elbow block's, `j3_coupler`'s - the modelled nuts carry the press, `tests/test_mounts.py _press`) | `lib/base/params.py BoreParams`, `lib/upper_arm/params.py ElbowParams`, `lib/forearm/params.py WristBossParams` / `RollDriveParams nut_af`, `lib/coupler/params.py nut_af` | print a fit gauge first (docs/cycloidal_drive.md §6) |
| The vendor 20T's tooth-band centre 10.95 from its hub face (`t20_hub`) and the 0.5 mm lift above the pad | `lib/forearm/params.py` | the pulley slides on the motor shaft (set screw): align it with the ring on assembly |
| The roll shaft's Ø40 end spigot + 4x M3 self-tapped into an 8 mm PETG wall carry the forearm's bending moment (no separate flange: bearing 2 must slide over the end) | `lib/forearm/params.py RollEndParams` | check for creep on the print; heat-set inserts or a bolted steel flange if it moves |
| The elbow block prints with its coupler features (lip / boss / journal / stub) off its side faces: supports either way (front face down keeps the bores vertical; the mount's nut pockets then lie on their side, corner up); the motor mount prints base down | `docs/forearm_roll.md` §3 | a `dfam-check` pass on `print/forearm_roll_block.stl` and `print/forearm_roll_motor_mount.stl`, then the first print |
| The roll motor mount's fits in PETG: its base in the block's pocket (`mount_fit` round it), the M3 nuts' hex pockets (`mount_nut_pocket_af` 5.35 for a 5.5 nut, a press so the nut stays when its screw is out - the modelled nuts carry it, `tests/forearm/test_roll_drive.py`), the 90° countersinks at the head's own size | `lib/forearm/params.py RollDriveParams mount_*` | print a fit gauge first (docs/cycloidal_drive.md §6) |
| The roll shaft's journals: +0.3 mm interference in the 6808 inner races, the seat +0.15 (PETG, like the drive's) | `RollDriveParams.journal_add / seat_add` | print a fit gauge first (docs/cycloidal_drive.md §6) |
| The base motor mount's joint in PETG: the M4 nuts' hex pockets in the base's posts (`nut_pocket_af`, a press so the nuts stay when the mount is off - the modelled nuts carry it, `tests/test_mounts.py`), the M4 clearance holes along X through the mount's ears and the posts (printed on their side), the ears' faces flat enough to close on the posts under 4 bolts | `lib/base/params.py JointParams`, `MountParams` | print a fit gauge first (docs/cycloidal_drive.md §6); the joint on the first print |
| The base_yaw belt, `YAW_BELT` (280-2GT from the stock list) - it sets the 48 mm motor's centre distance on the base motor mount; the slots give ± `MOTOR_TRAVEL`; the motor is held there by its 4 M3s' clamp alone (no tension screw) against the belt's pull toward the axis | `lib/base/params.py YAW_BELT`, `MOTOR_TRAVEL` | confirm the belt on the hardware; watch the motor for creep under tension - a tension screw through the mount's end wall if it slips |
| The base prints bottom down with supports under its cap and the plate's neck; its motor mount prints plate down, without: its truss walls' struts stand at ≥ 45°, the side triangles' table edges bridge (`MountParams.strut`, `truss_panels()`: ~21 / ~26 mm) | `lib/base/body.py`, `lib/base/layout.py`, `parts/base/base_motor_mount.py` | a `dfam-check` pass on `print/base.stl` and `print/base_motor_mount.stl`, then the first print |

## Not modelled yet
| item | note |
|---|---|
| Belt-side hardware of the elbow_pitch and wrist_pitch belts: the GT2 20T pulleys on their motors' shafts and the elbow belt (the wrist belt is an `EXTRAS` row); the elbow drive's second stage through `j1_link`'s x 128 seats (`lib/upper_arm/params.py BearingParams`: a pulley on an 8 mm shaft in two 608s?), whose belt plane must follow the elbow 90T - re-seated `PULLEY_SEAT_SHIFT` out on its bearing; what drives base_yaw — a third 90T under `j1_coupler` (the 90T's SolidWorks name says J1), its hub in the lower base bearing and bolted to the stub's end (a Recipe-B mount turned 45 degrees onto the stub's pattern, its M4 screws - x 40 from the 90T's outer face ends 1.3 past the nuts - and nuts), a 20T on the 48 mm motor, and the belt, `YAW_BELT` (a stock 2GT length): the base motor mount's slots put `BASE_MOTOR_PATTERN_CENTRE` at its centre distance, ± `MOTOR_TRAVEL` to tension it; the motor's 4 M3s through the slots | candidate `tools/bom.py EXTRAS` rows until modelled |
| The arm's own fasteners and the electronics (CAN adapter, wiring) - the 90T pulley bolts excepted (`parts/joints/{elbow,wrist}_pulley_{screws,nuts}`), e.g. `j3_coupler`'s flange M4s into `wrist_link` | not in `tools/bom.py EXTRAS` yet (it holds the drives' fasteners, the belts and the home sensor) |
| Simplified collision primitives in the URDF (visual meshes are reused for collision) | `robot/arm.urdf` TODO |
| The forearm roll's **home sensor** (on the end cap's outer face, a magnet in the shaft's stop lug, to the MKS board's limit input) | `tools/bom.py EXTRAS`; the lug and the post are modelled |
| The roll belt (240-2GT) and the wrist belt (264-2GT) | `tools/bom.py EXTRAS`; their lengths set the roll motor's centre distance / `J2_MOTOR_SLIDE_X` |
| The base_yaw motor's cables out of the base: they leave the motor mount into the base through the window between its posts (`lib/base/body.py _posts`), but the base has no way out for them (the lobe's cable notch went with the lobe) | decide where (a notch at the foot of the base's round or a side wall), then add it to `build_base` |
| The roll drive's cable route: through the shaft's Ø24 bore, out of the block's rear end wall on the axis (Ø26 `cable_exit`), then over the elbow to the upper arm | only the exit (`cable_exit`) is modelled |

## Not confirmed
| item | where |
|---|---|
| The elbow_pitch reduction: `robot/CLAUDE.md` gives it `GT2_RATIO` (90/20, one stage), but `j1_link`'s x 128 seats are for a second stage of the elbow drive - the ratio is provisional until that stage is designed | `lib/belts.py GT2_RATIO`, `robot/CLAUDE.md` reductions, `lib/upper_arm/params.py BearingParams` |
| Which CAN id (`software/control/src/config.py` J1..J3) drives which joint; `software/control/src/config.py` gear ratios still 1.0 while `CYCLOIDAL_RATIO` = 20 and `GT2_RATIO` = 4.5 | `software/control/README.md` Configure your motors, `robot/arm.urdf` ledger, `robot/frames.py` joint notes |
| The wrist-roll pancake motor's exact model | `parts/wrist/nema17_pancake.py PURCHASE_NOTE` |
| Link-membership assumptions (90T pulleys + J3 couplers with the driven links, the gripper linkage merged into `wrist_roll_link`) | `robot/frames.py LINKS` comments, the URDF ledger |
| A **4th CAN id** for the forearm roll: `software/control/src/config.py` J1..J3 name three MKS boards, the arm now carries five (base_yaw, shoulder_pitch, elbow_pitch, forearm_roll, wrist_pitch) - the control side is out of the CAD's scope | `software/control/src/config.py`, `software/control/README.md` Hardware |
| Whether the elbow 90T pulley is the driven side (it carries the roll drive's stator - the block that replaced `j3_coupler#1` - in `elbow_link`) | `robot/frames.py LINKS` [ASSUMPTION] |

## Not converted yet (SolidWorks geometry, not build123d)
The part's model still returns its SolidWorks export (`CONVERTED = False`, `parts/_templates/wrapper.py`), so its
geometry can be moved but not re-sized by a parameter. Converting one: `parts/CLAUDE.md` "Converting a part", then
Recipe C; the same commit deletes its row. `tools/bom.py`'s print list gives every part's state.

| part | link (`robot/frames.py LINKS`) | note |
|---|---|---|
| `gripper_clamp_bracket` | wrist_pitch_link | (`assemblies/arm.py`) |
| `gripper_j3_connector` | wrist_roll_link | two solids in one part (`tests/test_parts_convention.py MULTI_BODY`) |
| `servo_holder` | wrist_roll_link | holds the MG996R (`assemblies/gripper.py`) |
| `gripper_cover` | wrist_roll_link | |
| `gripper_link_1` | wrist_roll_link | the servo's crank linkage, merged into the link |
| `gripper_link_2` | wrist_roll_link | likewise |
| `gripper_slider` | jaw_a_link, jaw_b_link | |
| `gripper_finger_left` | jaw_a_link | |
| `gripper_finger_right` | jaw_b_link | the mirror of `gripper_finger_left` (its export is still named "…Hand Left") |
| `gripper_end` | jaw_a_link, jaw_b_link | |

## Known-broken / pending
| item | where |
|---|---|
| The motor-control `software/control/tests/` imports `arctos.*` and does not run | `software/control/CLAUDE.md` "Known issues" |
| The drive's motor envelope cuts the D-flat at `shaft_dcut_flat / 2` (flat-to-round 4.75) — ruled correct 2026-09-21 (it is what `reference/cycloidal/nema17_48mm.step` defines and the eccentric shaft's D-bore matches); noted here only because the parameter's name reads like 4.5 | `lib/cycloidal/motor.py flat_offset()` |
