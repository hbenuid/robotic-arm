# Open issues — what the CAD knows is not settled

**Purpose:** the ONE list of unsettled things a session should know before trusting a number or a fit: fit problems
the model carries knowingly, `[ESTIMATE]` values waiting for a measurement, hardware not modelled yet, mappings
not confirmed. **Rule:** when you flag something in a commit, add a row (issue, where it lives, what closes it,
the commit that raised it); when you close it, delete the row and say so in the commit message. A number here is
a fit value or an estimate, quoted beside the constant or test that holds it — the code wins if they drift; counts
and totals are never quoted (`cad/CLAUDE.md` Docs).

## Fit problems the model carries knowingly
| issue | where it lives | what closes it | raised |
|---|---|---|---|
| The base_yaw motor (48 mm) + its MKS board reach **6.1 mm below the base's bottom face** (62.1 mm stack, 56 mm of depth under the plate) | `lib/params.py BASE_MOTOR_STACK_PROUD`, `tests/test_mounts.py`, `test_params_invariants.py` | feet / a cut-out ≥ 6.1 mm under the base, or the plate moved up (`lib/base/params.py PlateParams`) | 2026-09-21 (`d9662a3`) |
| The base_yaw bearing stack is not clamped: `j1_coupler` has no inner-ring shoulder (its flat underside stops 1.5 mm above the upper 6806 and 0.5 above the base's seat ring, so tightening the base_yaw pulley's bolts would pull it onto the ring), and the lower 6806 holds nothing until the base_yaw pulley is modelled | `lib/mounts.py` `bearing_6806#1` / `#2`; `tests/test_mounts.py test_bearing_stacks` checks only the lip at the base | convert `j1_coupler` like `j3_coupler`'s DEFAULT - a Ø`BEARING_6806_SHOULDER_DIA` shoulder and a stub on through the lip to the pulley's hub, bolted flat onto it; model the base_yaw pulley (Not modelled, below) | 2026-09-25 (`git log --grep 6806-2RS`) |
| The drive motor's tie rods are not modelled (the MKS kit's M3x30 replace them); bolt shanks overlap the export's tapped holes in the model (46 / 159 mm³, thread engagement) | `tests/cycloidal/test_assembly.py` interference budget | nothing — a modelling representation; re-measure if the vendor file changes | 2026-09-21 (`3e667e0`) |
| The roll motor's 3 mm plate is braced only by its root in the motor mount's base (the cheeks that stiffened it are gone): the belt's pull, `t20` in front of the plate, and the motor hanging behind it bend that root | `lib/forearm/params.py RollDriveParams pad_t`, `lib/forearm/roll.py build_motor_mount` | check the plate for flex / creep under belt tension on the print; thicken `pad_t` or add a root fillet / gussets outside the motor's footprint if it gives | 2026-09-27 (`git log --grep forearm_roll_motor_mount`) |
| `j3_coupler`'s pulley-bolt holes are Ø4.1 (the SolidWorks pattern) for the wrist 90T's M4 shanks: 0.05 mm a side in PETG, and printed holes come out small (the 90T's own were opened to `M4_CLEAR`) | `lib/coupler/params.py CouplerParams.pulley_bolt_dia` | drill them Ø4.2 or open DEFAULT's to `M4_CLEAR` | 2026-09-26 (`git log --grep wrist_pulley_screws`) |

## Estimates to confirm on the hardware (`[ESTIMATE]` in `lib/params.py` unless noted)
| value | where | how to confirm |
|---|---|---|
| `NEMA17_40_MASS_G` 280, `MKS_SERVO42D_MASS_G` 35, `CYCLOIDAL_MOTOR_MASS_G` 400 (`[DATASHEET]` class value), `PANCAKE_MASS_G` 180 | `lib/params.py` | weigh; then Recipe C step 7 (inertials) |
| The 40 mm kit motors' real shaft length (the export said 23; the model carries the drive's 22) and the 48 mm one's (datasheet 17HS19-2004S1: 24 ± 1 with a 15 mm D-cut; the drive assumes 22 / 18 by ruling — `reference/cycloidal/nema17_48mm.step`) | `lib/cycloidal/params.py MotorParams`, `tools/reference/split_mks_motor.py` | measure from the mounting face; a 24 mm shaft bottoms the eccentric shaft's D-bore (`d_bore_depth` 14 → 16) |
| Connector / cable direction of each mounted motor (the spin about its axis) | `lib/mounts.py` frames, `note` fields | decide on the bench; change the `rz`, run `mount_placements.py`, Recipe C |
| The wrist-pitch motor's position on `j2_link`'s slide, `J2_MOTOR_SLIDE_X` = −136.37: what a stock **264-2GT** belt sets (`lib/forearm/params.py RollEndParams.wrist_belt`, `lib/belts.py`) - the belt length itself is the estimate | `lib/forearm/params.py` | confirm the belt on the hardware; 260-2GT would put it at −138.5 |
| Joint limits, efforts, velocities, axis signs, jaw travel | `lib/params.py` `*_LIMIT_DEG`, `ARM_JOINT_*`, `JAW_*` | viewer sweeps + hardware; `robot/arm.urdf` follows via `derive.py --check` |
| The roll belt, 240-2GT (`RollDriveParams.roll_belt`) - it sets the roll motor's centre distance (60.9) and so its height above the block (its MKS board 3.4 mm above the top - the motor mount's, level with the block's - at the nominal slot position, 0.9 at the slot's low end) | `lib/forearm/params.py` | confirm on the hardware; the plate's slots give +/- 2.5 mm |
| `FOREARM_ROLL_LIMIT_DEG` = 170 (the hard stop: the shaft's lug on its neck against the end cap's post - printed, 10° wide each, 1 mm of axial overlap) | `lib/params.py`, `lib/forearm/params.py stop_*` | the overlap and widths on the print; the lugs are small - a steel pin if PETG shears |
| 6808-2RS: mass 33 g, inner-race OD ≈ 44.5 (the shaft's Ø44 shoulders must not touch the outer race) | `lib/params.py BEARING_6808_MASS_G`, `RollDriveParams.inner_race_od` | datasheet / calipers on the bearing in hand |
| 6806-2RS: mass 26 g (the 6808's, scaled by the ring area); the inner ring's outer edge against the Ø33 shoulders on the stubs and the 90T's Ø34.76 ring (both must bear on the inner ring only) | `lib/bearings.py` | datasheet / calipers on the bearing in hand |
| The 6806 seats in PETG: Ø42.2 (the base, `j1_link`'s lower seat, `j2_link`'s two), `j1_link`'s upper seat Ø42.0 (line-to-line); the Ø30 stubs and hubs in the inner rings (line-to-line; `j1_coupler`'s Ø29.8); the M4 nuts' hex channels / pockets, 6.85 across flats for a 7.0 nut (the elbow block's, `j3_coupler`'s - the modelled nuts carry the press, `tests/test_mounts.py _press`) | `lib/base/params.py BoreParams`, `lib/upper_arm/params.py ElbowParams`, `lib/forearm/params.py WristBossParams` / `RollDriveParams nut_af`, `lib/coupler/params.py nut_af` | print a fit gauge first (docs/cycloidal_drive.md §6) |
| The vendor 20T's tooth-band centre 10.95 from its hub face (`t20_hub`) and the 0.5 mm lift above the pad | `lib/forearm/params.py` | the pulley slides on the motor shaft (set screw): align it with the ring on assembly |
| The roll shaft's Ø40 end spigot + 4x M3 self-tapped into an 8 mm PETG wall carry the forearm's bending moment (no separate flange: bearing 2 must slide over the end) | `lib/forearm/params.py RollEndParams` | check for creep on the print; heat-set inserts or a bolted steel flange if it moves |
| The elbow block prints with its coupler features (lip / boss / journal / stub) off its side faces: supports either way (front face down keeps the bores and the mount's nut channels vertical); the motor mount prints base down | `docs/forearm_roll.md` §3 | a `dfam-check` pass on `print/forearm_roll_block.stl` and `print/forearm_roll_motor_mount.stl`, then the first print |
| The roll motor mount's fits in PETG: its base in the block's step (`mount_fit` at the riser only - the countersunk screws locate it), the M3 nut channels (the nut + `mount_channel_add`, a slide), the 90° countersinks at the head's own size | `lib/forearm/params.py RollDriveParams mount_*` | print a fit gauge first (docs/cycloidal_drive.md §6) |
| The roll shaft's journals: +0.3 mm interference in the 6808 inner races, the seat +0.15 (PETG, like the drive's) | `RollDriveParams.journal_add / seat_add` | print a fit gauge first (docs/cycloidal_drive.md §6) |

## Not modelled yet
| item | note |
|---|---|
| Belt-side hardware of the elbow_pitch and wrist_pitch belts: the GT2 20T pulleys on their motors' shafts and the elbow belt (the wrist belt is an `EXTRAS` row); the elbow drive's second stage through `j1_link`'s x 128 seats (`lib/upper_arm/params.py BearingParams`: a pulley on an 8 mm shaft in two 608s?), whose belt plane must follow the elbow 90T - re-seated `PULLEY_SEAT_SHIFT` out on its bearing; what drives base_yaw — a driven pulley under `j1_coupler` (the 90T's SolidWorks name says J1: probably a third copy, its hub in the lower base bearing) | candidate `tools/bom.py EXTRAS` rows until modelled |
| The base_yaw **thrust bearing** in the base's annular groove round the seat ring (Ø65.1 / Ø90.2, 3.0 deep): no bearing is chosen or modelled, and nothing rests on it - `j1_coupler`'s underside stands 3.9 above the groove floor over it but sits flush on the base's top face outside it, so as modelled the coupler turns on the base's face, not on a bearing | `lib/base/params.py CapParams groove_r / groove_y0`; choose the bearing (the groove fits a 65 x 90 axial one), add it (Recipe A, a `lib/mounts.py` row) and make `j1_coupler` clear the top face when it is converted (the 6806 row above) |
| The arm's own fasteners and the electronics (CAN adapter, wiring) - the 90T pulley bolts excepted (`parts/joints/{elbow,wrist}_pulley_{screws,nuts}`), e.g. `j3_coupler`'s flange M4s into `wrist_link` | not in `tools/bom.py EXTRAS` yet (it holds the drives' fasteners, the belts and the home sensor) |
| Simplified collision primitives in the URDF (visual meshes are reused for collision) | `robot/arm.urdf` TODO |
| The forearm roll's **home sensor** (on the end cap's outer face, a magnet in the shaft's stop lug, to the MKS board's limit input) | `tools/bom.py EXTRAS`; the lug and the post are modelled |
| The roll belt (240-2GT) and the wrist belt (264-2GT) | `tools/bom.py EXTRAS`; their lengths set the roll motor's centre distance / `J2_MOTOR_SLIDE_X` |
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

## Known-broken / pending
| item | where |
|---|---|
| The motor-control `software/control/tests/` imports `arctos.*` and does not run | `software/control/CLAUDE.md` "Known issues" |
| Most custom parts are still SolidWorks wrappers (`CONVERTED = False`; the parametric ones so far are the links `j2_link`, `lib/forearm/`, and `j1_link`, `lib/upper_arm/`, the `base`, `lib/base/`, and `j3_coupler`, `lib/coupler/`; `gt2_pulley_90t` is the SolidWorks body with its bolt holes opened to `M4_CLEAR`, a diverged part; the print list of `tools/bom.py` gives every part's state); the fit problems above are fixed at conversion | `parts/<group>/*.py`, `cad/parts/CLAUDE.md` "Part states" |
| The drive's motor envelope cuts the D-flat at `shaft_dcut_flat / 2` (flat-to-round 4.75) — ruled correct 2026-09-21 (it is what `reference/cycloidal/nema17_48mm.step` defines and the eccentric shaft's D-bore matches); noted here only because the parameter's name reads like 4.5 | `lib/cycloidal/motor.py flat_offset()` |
