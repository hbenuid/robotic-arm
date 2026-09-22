# Open issues — what the CAD knows is not settled

**Last updated:** 2026-09-21 — see the root `CHANGELOG.md` for dated changes.

**Purpose:** the ONE list of unsettled things a session should know before trusting a number or a fit: fit problems
the model carries knowingly, `[ESTIMATE]` values waiting for a measurement, hardware not modelled yet, mappings
not confirmed. **Rule:** when you flag something in a commit, add a row (issue, where it lives, what closes it,
the CHANGELOG entry that raised it); when you close it, delete the row and say so in the CHANGELOG. Numbers here
are quoted from the code they live in — the code wins if they drift.

## Fit problems the model carries knowingly
| issue | where it lives | what closes it | raised |
|---|---|---|---|
| The base_yaw motor (48 mm) + its MKS board reach **6.1 mm below the base's bottom face** (62.1 mm stack, 56 mm of depth under the plate) | `lib/params.py BASE_MOTOR_STACK_PROUD`, `tests/test_mounts.py`, `test_params_invariants.py` | feet / a cut-out ≥ 6.1 mm under the base, or the plate moved up when `base` is converted | 2026-09-21 (`d9662a3`) |
| `j2_link`'s Ø20 central slot vs the wrist-pitch motor's **Ø22 pilot boss** (24.7 mm³ overlap budgeted; physically the boss cannot enter, the motor would sit 2 mm proud) | `tests/test_mounts.py` budget `("nema17_40mm#3", "j2_link#1"): 30` | widen the slot to ≥ Ø22.3 when `j2_link` is converted, or a 2 mm spacer | 2026-09-21 (`2abdb0b`) |
| `j1_link`'s pad holes are 0.38 mm off the shoulder axis and uneven (30.8 / 31.2 mm); the elbow motor is placed on the axis, not on the holes | `lib/mounts.py` note on `nema17_40mm#2` | fix the pattern when `j1_link` is converted | 2026-09-21 (`2abdb0b`) |
| The drive motor's tie rods are not modelled (the MKS kit's M3x30 replace them); bolt shanks overlap the export's tapped holes in the model (46 / 159 mm³, thread engagement) | `tests/cycloidal/test_assembly.py` interference budget | nothing — a modelling representation; re-measure if the vendor file changes | 2026-09-21 (`3e667e0`) |

## Estimates to confirm on the hardware (`[ESTIMATE]` in `lib/params.py` unless noted)
| value | where | how to confirm |
|---|---|---|
| `NEMA17_40_MASS_G` 280, `MKS_SERVO42D_MASS_G` 35, `CYCLOIDAL_MOTOR_MASS_G` 400 (`[DATASHEET]` class value), `PANCAKE_MASS_G` 180 | `lib/params.py` | weigh; then Recipe C step 7 (inertials) |
| The 40 mm kit motors' real shaft length (the export said 23; the model carries the drive's 22) and the 48 mm one's (datasheet 17HS19-2004S1: 24 ± 1 with a 15 mm D-cut; the drive assumes 22 / 18 by ruling — `reference/cycloidal/nema17_48mm.step`) | `lib/cycloidal/params.py MotorParams`, `tools/reference/split_mks_motor.py` | measure from the mounting face; a 24 mm shaft bottoms the eccentric shaft's D-bore (`d_bore_depth` 14 → 16) |
| Connector / cable direction of each mounted motor (the spin about its axis) | `lib/mounts.py` frames, `note` fields | decide on the bench; change the `rz`, run `mount_placements.py`, Recipe C |
| The wrist-pitch motor's position on `j2_link`'s slide, `J2_MOTOR_SLIDE_X` = −118 (the body clears `j2_cap_1`'s window only for −127..−109) | `lib/params.py` | set with the belt length |
| Joint limits, efforts, velocities, axis signs, jaw travel | `lib/params.py` `*_LIMIT_DEG`, `ARM_JOINT_*`, `JAW_*` | viewer sweeps + hardware; `robot/arm.urdf` follows via `derive.py --check` |

## Not modelled yet
| item | note |
|---|---|
| Belt-side hardware of the three belt joints: 3 × GT2 20T pulleys on the motor shafts, the belts, the base-yaw driven pulley / what `j1_coupler` is driven by | candidate `tools/bom.py EXTRAS` rows until modelled |
| The arm's own fasteners and the electronics (CAN adapter, wiring) | `tools/bom.py EXTRAS` lists only the drive's arm-mount bolts, nuts and grease |
| Simplified collision primitives in the URDF (visual meshes are reused for collision) | `robot/arm.urdf` TODO |

## Not confirmed
| item | where |
|---|---|
| Which CAN id (`src/config.py` J1..J3) drives which joint; `src/config.py` gear ratios still 1.0 while `CYCLOIDAL_RATIO` = 20 and `GT2_RATIO` = 4.5 | root `README.md`, `robot/arm.urdf` ledger, `robot/frames.py` joint notes |
| The wrist-roll pancake motor's exact model | `parts/wrist/nema17_pancake.py PURCHASE_NOTE` |
| Link-membership assumptions (90T pulleys + J3 couplers with the driven links, the gripper linkage merged into `wrist_roll_link`) | `robot/frames.py LINKS` comments, the URDF ledger |

## Known-broken / pending
| item | where |
|---|---|
| The repo-root `tests/` (motor-control) imports `arctos.*` and does not run | root `CLAUDE.md` "Known issues" |
| 20 custom parts are still SolidWorks wrappers (`CONVERTED = False`); the fit problems above are fixed at conversion | `parts/<group>/*.py`, `cad/CLAUDE.md` "Part states" |
| The drive's motor envelope cuts the D-flat at `shaft_dcut_flat / 2` (flat-to-round 4.75) — ruled correct 2026-09-21 (it is what `reference/cycloidal/nema17_48mm.step` defines and the eccentric shaft's D-bore matches); noted here only because the parameter's name reads like 4.5 | `lib/cycloidal/motor.py flat_offset()` |
