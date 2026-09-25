# Open issues — what the CAD knows is not settled

**Last updated:** 2026-09-25 — see the root `CHANGELOG.md` for dated changes.

**Purpose:** the ONE list of unsettled things a session should know before trusting a number or a fit: fit problems
the model carries knowingly, `[ESTIMATE]` values waiting for a measurement, hardware not modelled yet, mappings
not confirmed. **Rule:** when you flag something in a commit, add a row (issue, where it lives, what closes it,
the CHANGELOG entry that raised it); when you close it, delete the row and say so in the CHANGELOG. Numbers here
are quoted from the code they live in — the code wins if they drift.

## Fit problems the model carries knowingly
| issue | where it lives | what closes it | raised |
|---|---|---|---|
| The base_yaw motor (48 mm) + its MKS board reach **6.1 mm below the base's bottom face** (62.1 mm stack, 56 mm of depth under the plate) | `lib/params.py BASE_MOTOR_STACK_PROUD`, `tests/test_mounts.py`, `test_params_invariants.py` | feet / a cut-out ≥ 6.1 mm under the base, or the plate moved up when `base` is converted | 2026-09-21 (`d9662a3`) |
| The drive motor's tie rods are not modelled (the MKS kit's M3x30 replace them); bolt shanks overlap the export's tapped holes in the model (46 / 159 mm³, thread engagement) | `tests/cycloidal/test_assembly.py` interference budget | nothing — a modelling representation; re-measure if the vendor file changes | 2026-09-21 (`3e667e0`) |

## Estimates to confirm on the hardware (`[ESTIMATE]` in `lib/params.py` unless noted)
| value | where | how to confirm |
|---|---|---|
| `NEMA17_40_MASS_G` 280, `MKS_SERVO42D_MASS_G` 35, `CYCLOIDAL_MOTOR_MASS_G` 400 (`[DATASHEET]` class value), `PANCAKE_MASS_G` 180 | `lib/params.py` | weigh; then Recipe C step 7 (inertials) |
| The 40 mm kit motors' real shaft length (the export said 23; the model carries the drive's 22) and the 48 mm one's (datasheet 17HS19-2004S1: 24 ± 1 with a 15 mm D-cut; the drive assumes 22 / 18 by ruling — `reference/cycloidal/nema17_48mm.step`) | `lib/cycloidal/params.py MotorParams`, `tools/reference/split_mks_motor.py` | measure from the mounting face; a 24 mm shaft bottoms the eccentric shaft's D-bore (`d_bore_depth` 14 → 16) |
| Connector / cable direction of each mounted motor (the spin about its axis) | `lib/mounts.py` frames, `note` fields | decide on the bench; change the `rz`, run `mount_placements.py`, Recipe C |
| The wrist-pitch motor's position on `j2_link`'s slide, `J2_MOTOR_SLIDE_X` = −136.37: what a stock **264-2GT** belt sets (`lib/forearm/params.py RollEndParams.wrist_belt`, `lib/belts.py`) - the belt length itself is the estimate | `lib/forearm/params.py` | confirm the belt on the hardware; 260-2GT would put it at −138.5 |
| Joint limits, efforts, velocities, axis signs, jaw travel | `lib/params.py` `*_LIMIT_DEG`, `ARM_JOINT_*`, `JAW_*` | viewer sweeps + hardware; `robot/arm.urdf` follows via `derive.py --check` |
| The roll belt, 240-2GT (`RollDriveParams.roll_belt`) - it sets the roll motor's centre distance (60.9) and so its height above the block (its body 1.9 mm above the top at the nominal slot position) | `lib/forearm/params.py` | confirm on the hardware; the plate's slots give +/- 2.5 mm |
| `FOREARM_ROLL_LIMIT_DEG` = 170 (the hard stop: the shaft's lug on its neck against the end cap's post - printed, 10° wide each, 1 mm of axial overlap) | `lib/params.py`, `lib/forearm/params.py stop_*` | the overlap and widths on the print; the lugs are small - a steel pin if PETG shears |
| 6808-2RS: mass 33 g, inner-race OD ≈ 44.5 (the shaft's Ø44 shoulders must not touch the outer race) | `lib/params.py BEARING_6808_MASS_G`, `RollDriveParams.inner_race_od` | datasheet / calipers on the bearing in hand |
| The vendor 20T's tooth-band centre 10.95 from its hub face (`t20_hub`) and the 0.5 mm lift above the pad | `lib/forearm/params.py` | the pulley slides on the motor shaft (set screw): align it with the ring on assembly |
| The roll shaft's Ø40 end spigot + 4x M3 self-tapped into an 8 mm PETG wall carry the forearm's bending moment (no separate flange: bearing 2 must slide over the end) | `lib/forearm/params.py RollEndParams` | check for creep on the print; heat-set inserts or a bolted steel flange if it moves |
| The elbow's torque path is 4x M4 in **heat-set inserts** in the elbow block's PETG underside (the elbow 90T bolts up into them, 8 mm of insert) | `lib/forearm/params.py RollDriveParams insert_*`, `tools/bom.py EXTRAS` | check for creep on the print; a bolted steel insert plate under the boss is the fallback |
| The elbow block prints with its coupler features (lip / boss / journal / stub) and the motor plate off its side faces: supports either way (front face down keeps the bores vertical) | `docs/forearm_roll.md` §3 | a `dfam-check` pass on `print/forearm_roll_block.stl`, then the first print |
| The roll shaft's journals: +0.3 mm interference in the 6808 inner races, the seat +0.15 (PETG, like the drive's) | `RollDriveParams.journal_add / seat_add` | print a fit gauge first (docs/cycloidal_drive.md §6) |

## Not modelled yet
| item | note |
|---|---|
| Belt-side hardware of the three belt joints: 3 × GT2 20T pulleys on the motor shafts, the belts, the base-yaw driven pulley / what `j1_coupler` is driven by | candidate `tools/bom.py EXTRAS` rows until modelled |
| The arm's own fasteners and the electronics (CAN adapter, wiring) | `tools/bom.py EXTRAS` lists only the drive's arm-mount bolts, nuts and grease |
| Simplified collision primitives in the URDF (visual meshes are reused for collision) | `robot/arm.urdf` TODO |
| The forearm roll's **home sensor** (on the end cap's outer face, a magnet in the shaft's stop lug, to the MKS board's limit input) | `tools/bom.py EXTRAS`; the lug and the post are modelled |
| The roll belt (240-2GT) and the wrist belt (264-2GT) | `tools/bom.py EXTRAS`; their lengths set the roll motor's centre distance / `J2_MOTOR_SLIDE_X` |
| The roll drive's cable route: through the shaft's Ø24 bore, out of the block's rear end wall on the axis (Ø26 `cable_exit`), then over the elbow to the upper arm | nothing modelled; the exit is |
| The elbow's own bearings between the block's Ø30 stub and `j1_link`'s Ø42 bore (6702-class rings fit the SolidWorks geometry; the capture never had them either) | `lib/forearm/params.py RollDriveParams stub_*`; candidate `tools/bom.py EXTRAS` rows |

## Not confirmed
| item | where |
|---|---|
| Which CAN id (`software/control/src/config.py` J1..J3) drives which joint; `software/control/src/config.py` gear ratios still 1.0 while `CYCLOIDAL_RATIO` = 20 and `GT2_RATIO` = 4.5 | root `README.md`, `robot/arm.urdf` ledger, `robot/frames.py` joint notes |
| The wrist-roll pancake motor's exact model | `parts/wrist/nema17_pancake.py PURCHASE_NOTE` |
| Link-membership assumptions (90T pulleys + J3 couplers with the driven links, the gripper linkage merged into `wrist_roll_link`) | `robot/frames.py LINKS` comments, the URDF ledger |
| A **4th CAN id** for the forearm roll: `software/control/src/config.py` J1..J3 name three MKS boards, the arm now carries five (base_yaw, shoulder_pitch, elbow_pitch, forearm_roll, wrist_pitch) - the control side is out of the CAD's scope | `software/control/src/config.py`, root `README.md` |
| Whether the elbow 90T pulley is the driven side (it carries the roll drive's stator - the block that replaced `j3_coupler#1` - in `elbow_link`) | `robot/frames.py LINKS` [ASSUMPTION] |

## Known-broken / pending
| item | where |
|---|---|
| The motor-control `software/control/tests/` imports `arctos.*` and does not run | root `CLAUDE.md` "Known issues" |
| 15 of the 17 custom parts are still SolidWorks wrappers (`CONVERTED = False`; the links are parametric: `j2_link` since 2026-09-22, `lib/forearm/`, `j1_link` since 2026-09-25, `lib/upper_arm/`); the fit problems above are fixed at conversion | `parts/<group>/*.py`, `cad/CLAUDE.md` "Part states" |
| The drive's motor envelope cuts the D-flat at `shaft_dcut_flat / 2` (flat-to-round 4.75) — ruled correct 2026-09-21 (it is what `reference/cycloidal/nema17_48mm.step` defines and the eccentric shaft's D-bore matches); noted here only because the parameter's name reads like 4.5 | `lib/cycloidal/motor.py flat_offset()` |
