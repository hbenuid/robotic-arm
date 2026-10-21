# lib/ — shared dimensions and geometry code

Loads when you work in `lib/`. `lib/` is the left end of the layering (`cad/AGENTS.md` "Layering"): it never imports
`parts/`, `assemblies/`, `robot/`, `tools/` or `tests/`. The rules for the modules that describe the assembly rather
than geometry live with their users: `mounts.py` (Recipe B) and `placements.py` (retired / skipped / shifted / turned
occurrences) in `assemblies/AGENTS.md`; the `reference.py` registries (`CUSTOM`, `COTS`, `DESIGNED`, `NATIVE`, …) in
`parts/AGENTS.md` and `reference/AGENTS.md`.

## What is where
```
params.py     # single source of truth for shared dimensions (tagged provenance)
units.py      # IN, NUDGE - a leaf module (lib/cycloidal/ imports it; params.py re-exports it)
motors.py     # the arm's motors (NEMA 17 interface, pancake, the 40 mm kit motor + MKS board, MOTOR_40) - a leaf, re-exported by params.py
belts.py      # GT2: the groove's tooth form (flank_offset()), the pulleys (the 20-60T's bands), pulley_od(), closed_belt_length() / centre_distance(), stock belt lengths (STANDARD_2GT_LENGTHS: what the belts are bought as), the belt's section, a loop's exact path round pulleys + a back-side idler (belt_runs() / belt_wraps() / belt_length(), idler_on_run()), the 90T's hub faces + bolt circle (pulley_90t_bolt_points()), the purchased toothless idler (GT2_IDLER_*) - a leaf
bearings.py   # the belt joints' 6806-2RS pair: bore / OD / width, the inner-ring shoulder, the mass; the base_yaw thrust bearing (AXK / AS 6590, THRUST_*) - a leaf
fasteners.py  # the arm's screws + nuts: M4_SHCS / M4_NUT / M4_PITCH, M3_SHCS / M3_CSK / M3_NUT / M3_PITCH, M5_SHCS / M5_NUT / M5_PITCH, the M3-M5 clearance holes, shcs() / csk() / hex_nut() (the plain geometry of the arm's screw / nut parts - the drive's patterns are its own) - a leaf
sensors.py    # the joints' endstop / home sensor: the KY-003 hall module (the A3144 chip, the board, the header), ky003_hall_point() - a leaf
cots.py       # hybrid(): the body of every purchased part (vendor STEP, else the envelope); pattern() for the multi-body ones
geom.py       # the arm's small build123d helpers: align_min(), cylinder(), through() (NUDGE overshoot), single_solid(), hex_prism(); the belt drives' box(), stop_wedge(), slot_y(), nut_slot() - a leaf
base/         # the base and its bolt-on motor mount (BaseConfig: LEGACY = the SolidWorks base, DEFAULT = what is built - the +X lobe cut off at JointParams, the motor in a bolt-on mount - the base's D carried on to a round end, its foot round it (MountParams) - on round holes at YAW_MOTOR_X, short of the stock belt's centre distance, the belt's tensioner on its plate (YawIdlerParams: its slot and the nut's rails; build_yaw_idler_spacer() its printed spacer), the walls j1_coupler's cone carried on down (ShellParams.draft; BASE_R / BASE_DRAFT live here, the coupler imports them) onto a foot - a D flange, a straight chamfer, screw holes (FootParams) -, a band of slits through both parts' walls (SlitParams))
coupler/      # the J3 coupler (CouplerParams: LEGACY = the SolidWorks j3_coupler, DEFAULT = what is built)
yaw_coupler/  # the base_yaw coupler, the drive's yoke (YawCouplerConfig: LEGACY = the SolidWorks j1_coupler, DEFAULT = what is built - the fork round the turning-shell drive, ForkParams: the disc drafted round and carried up the legs' outer faces, the hub leg's disc the hub's own, the motor-side leg its own part bolted to the disc and to the motor plate's sleeve - centred on the base_yaw axis with the drive's discs)
wrist/        # the wrist body (WristConfig: LEGACY = the SolidWorks wrist_link, DEFAULT = what is built - the plate on the pitch axis carrying the wrist roll drive's frame), the wrist roll drive (WristRollParams, stack_positions, the frame / pulley / shaft builders: roll.py) and the gripper's connector to it (ConnectorParams, connector.py)
pulley/       # the 90T pulley (PulleyParams: LEGACY = the SolidWorks gt2_pulley_90t, DEFAULT = what is built, YAW = DEFAULT with 120 teeth - gt2_pulley_120t), the 20-60T compound pulley (CompoundPulleyParams: COMPOUND = the measured gt2_pulley_20_60t, build_compound()); teeth.py: the GT2 groove (arcs solved from lib/belts.py's tooth form) and gt2_ring() (the toothed ring: the pulleys' rims and bands, the roll pulleys' rings), and belt.py: closed_belt(), a belt's smooth band round its loop (lib/belts.py)
upper_arm/    # the upper arm (UpperArmConfig: LEGACY = the SolidWorks j1_link, DEFAULT = no cap sockets, the arm - one flat slab, the old plate's elbow-half depth end to end - rising off the drive's turning shell, flush with its hub end (the face the part prints on), and printed with its body (ArmParams), the elbow motor out along the link at ELBOW_MOTOR_CENTRES, on the arm's +N side down a square hole through the slab onto a web across it, flush with the slab's underside (no pad), the elbow belt's tensioner in the stepped slot's place (ElbowIdlerParams: its slots; build_idler_spacer() its printed spacer), the elbow block's clearance)
forearm/      # the forearm (ForearmConfig: LEGACY = the SolidWorks j2_link, DEFAULT = the roll end, a straight web, the wrist boss as wide as it, no cap sockets, the forearm elbow_offset across the elbow axis, the wrist-pitch motor on round holes at WRIST_MOTOR_CENTRES (MotorHoleParams: its screws' counterbores), the wrist belt's tensioner under the web (IdlerParams: its rib, slot and nut channel)) and the roll drive (RollDriveParams, stack_positions, the frame / pulley / shaft builders; the pulley's 90T ring is lib/pulley/teeth.py gt2_ring())
datum.py      # capture frame W -> base_link frame B: frame(), base_frame() (arm.py arm_from_w(), robot/frames.py); frames as data: IDENTITY, to_location()
mounts.py     # what the SolidWorks capture never placed right, as frames-as-data: MOUNTS (part occurrences) and MODULE_MOUNTS (the code-driven modules' poses)
reference.py  # naming maps (SolidWorks custom/COTS, designed cycloidal parts, native and measured parts, the parts designed here with no reference - NO_REFERENCE -, modules), loaders, path_of(), matches_reference()
manifest.py   # reference/manifest.json: read() / write() / entry() - shared by the import tools (import_solidworks, import_cadquery, import_native) and the tests
placements.py # reference/placements.json -> build123d Location; the design's changes to capture records (RETIRED, SHIFTS, TURNS)
models.py     # model_of() / raw() / geometry(inline=): the @step model of a module, its body, a child for an assembly
assembly.py   # assembly(name, children): the native labelled Compound node; label_shape / label_text from cadgen
cycloidal/    # the cycloidal drive: DriveConfig (params.py), layout.py, profiles.py, housing.py, disc.py, motor.py (THE NEMA 17 pilot + shaft, every kit motor's)
```

## Shared dimensions (DRY)
`lib/params.py` is the single source of truth: mm and grams, every constant tagged
`[MEASURE] / [DATASHEET] / [DESIGN] / [REFERENCE] / [ESTIMATE]` (one tag may head a block) or derived from tagged ones,
with a derivation comment. It re-exports the leaves below it (`lib/units.py`, `lib/motors.py`, `lib/belts.py`,
`lib/bearings.py`, `lib/fasteners.py`, `lib/sensors.py`: What is where) unchanged; every `lib/` package - the ones it
re-exports from and the others alike - takes its globals from those leaves and its geometry helpers from `lib/geom.py`,
never from `lib.params` — and no leaf imports `lib.params` either (`tests/test_layering.py LEAF_PACKAGES` /
`LEAF_MODULES`). Datum: the SolidWorks capture
frame is **Y up** (J1 axis); the URDF base frame (REP-103) is `lib/datum.py base_frame()` (with `frame()`,
`U`, `BASE_FORWARD`; `robot/frames.py` re-exports them and builds the kinematics on top) — and
`assemblies/arm.py` emits the arm in it (`arm_from_w()`, see `assemblies/AGENTS.md`), so `arm.step` is **Z up**.
The cycloidal drive's own dimensions are `lib/cycloidal/params.py` (`DriveConfig`, frozen
dataclasses, variants via `dataclasses.replace`); `lib/params.py` re-exports the interface values
(`CYCLOIDAL_*`, masses) from it - never retype a drive number.

Changing a shared dimension — touchpoints in order:

| # | Edit | What |
|---|---|---|
| 1 | `lib/params.py`, or the leaf / `lib/<pkg>/params.py` it re-exports | the value where it is defined (keep tag + derivation) |
| 2 | `tests/test_params_invariants.py` (a package's own: `tests/<pkg>/`) | the lock; `./cadtool pytest -m "not slow"` |
| 3 | Recipe C (`cad/AGENTS.md`) | regenerate, hash-gate, the full suite, the robot, the snapshots |

## Kernel gotchas (all verified)
- `Compound.volume` skips nested sub-assemblies (build123d 0.10 and 0.11) — use `lib.reference.solid_volume()`.
- `is_valid` is a **property**; `Location.to_tuple()` is deprecated (`tuple(loc)` → two Vectors).
- `Face.center()` on a cylindrical face is the SURFACE midpoint (a half-cylinder reports axis ± r), not the axis:
  measure axes with `BRepAdaptor_Surface(face.wrapped).Cylinder().Axis()` (`tools/reference/split_mks_motor.py
  cylinders()`) or `Face.axis_of_rotation`, and bolt patterns from those, never from face centres.
- Cycloidal discs: chamfer the lobe edges BEFORE cutting holes (the end face must carry only the
  spline edge); the profile is a periodic *interpolating* spline (`Edge.make_spline(periodic=True)`,
  never `make_spline_approx`); OCCT's analytic volume is ~0.3 % off on that face (both ours and the
  reference; the adaptive integration of the `wrist_link` bullet is worse here) - compare tessellations/face sets, not
  `.volume`. A boolean between the two discs takes minutes: probe points instead.
- `wrist_link`'s fillet (`lib/wrist/link.py` `_tower()`, LEGACY only: DEFAULT has none): OCCT's default `.volume` (the
  integration `derive.py`'s inertials use) reads the trimmed fillet face ~0.06 % low, and the error jumps with unrelated
  construction changes (the fillet core's height); the adaptive integration
  `BRepGProp.VolumeProperties_s(shape.wrapped, props, 1e-7, False)` is stable and agrees with the reference - measure a
  change to that fillet with it (or cell-by-cell commons), not `.volume`.
  The fillet itself is made on a core carried over the tower's top: on the part the slope's edge with the bore ends in a
  point at the top, which `fillet()` cannot run through.
