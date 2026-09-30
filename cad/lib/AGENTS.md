# lib/ — shared dimensions and geometry code

Loads when you work in `lib/`. `lib/` is the left end of the layering (`cad/AGENTS.md` "Layering"): it never imports
`parts/`, `assemblies/`, `robot/`, `tools/` or `tests/`. The rules for the modules that describe the assembly rather
than geometry live with their users: `mounts.py` (Recipe B) and `placements.py` (retired / skipped / shifted occurrences) in
`assemblies/AGENTS.md`; the `reference.py` registries (`CUSTOM`, `COTS`, `DESIGNED`, `NATIVE`, …) in
`parts/AGENTS.md` and `reference/AGENTS.md`.

## What is where
```
params.py     # single source of truth for shared dimensions (tagged provenance)
units.py      # IN, NUDGE - a leaf module (lib/cycloidal/ imports it; params.py re-exports it)
motors.py     # the arm's motors (NEMA 17 interface, pancake, the 40 mm kit motor + MKS board, MOTOR_40) - a leaf, re-exported by params.py
belts.py      # GT2: the groove's tooth form (flank_offset()), the pulleys (the 20-60T's bands), pulley_od(), closed_belt_length() / centre_distance(), stock belt lengths, the 90T's hub faces + bolt circle (pulley_90t_bolt_points()), the purchased toothless idler (GT2_IDLER_*) - a leaf
bearings.py   # the belt joints' 6806-2RS pair: bore / OD / width, the inner-ring shoulder, the mass; the base_yaw thrust bearing (AXK / AS 6590, THRUST_*) - a leaf
fasteners.py  # the arm's screws + nuts: M4_SHCS / M4_NUT / M4_PITCH, M3_CSK / M3_NUT / M3_PITCH, the M3-M5 clearance holes, shcs() / csk() / hex_nut() (the plain geometry of the pulley bolts and of the roll and base motor mounts' screws + nuts) - a leaf
sensors.py    # the joints' endstop / home sensor: the KY-003 hall module (the A3144 chip, the board, the header), ky003_hall_point() - a leaf
cots.py       # hybrid(): the body of every purchased part (vendor STEP, else the envelope); pattern() for the multi-body ones
geom.py       # the arm's small build123d helpers: align_min(), cylinder(), through() (NUDGE overshoot), single_solid(), hex_prism() - a leaf
base/         # the base and its bolt-on motor mount (BaseConfig: LEGACY = the SolidWorks base, DEFAULT = what is built - the +X lobe cut off at JointParams, the motor in a narrower bolt-on box (MountParams) slotted at the stock belt's centre distance)
coupler/      # the J3 coupler (CouplerParams: LEGACY = the SolidWorks j3_coupler, DEFAULT = what is built)
yaw_coupler/  # the base_yaw coupler, the drive's yoke (YawCouplerConfig: LEGACY = the SolidWorks j1_coupler, DEFAULT = what is built)
wrist/        # the wrist body (WristConfig: LEGACY = the SolidWorks wrist_link, DEFAULT = what is built)
pulley/       # the 90T pulley (PulleyParams: LEGACY = the SolidWorks gt2_pulley_90t, DEFAULT = what is built, YAW = DEFAULT with 120 teeth - gt2_pulley_120t), the 20-60T compound pulley (CompoundPulleyParams: COMPOUND = the measured gt2_pulley_20_60t, build_compound()) and teeth.py: the GT2 groove (arcs solved from lib/belts.py's tooth form) and gt2_ring() (its rim, the roll shaft's ring)
upper_arm/    # the upper arm (UpperArmConfig: LEGACY = the SolidWorks j1_link, DEFAULT = no cap sockets, the motor's and the drive's hole patterns, the elbow block's clearance)
forearm/      # the forearm (ForearmConfig: LEGACY = the SolidWorks j2_link, DEFAULT = the roll end, no cap sockets) and the roll drive (RollDriveParams, stack_positions, the block / motor mount / shaft / retainer builders; the shaft's 90T ring is lib/pulley/teeth.py gt2_ring())
datum.py      # capture frame W -> base_link frame B: frame(), base_frame() (arm.py arm_from_w(), robot/frames.py); frames as data: IDENTITY, to_location()
mounts.py     # what the SolidWorks capture never placed right, as frames-as-data: the base's motor mount + its M4s, the belt joints' motors + MKS boards, their 6806 pairs, the base_yaw thrust stack, the driven pulleys (the elbow's and the wrist's 90T re-seated, the base_yaw 120T placed) and their M4 screws + nuts
reference.py  # naming maps (SolidWorks custom/COTS, designed cycloidal parts, native and measured parts, the parts designed here with no reference - NO_REFERENCE -, modules), loaders, path_of(), matches_reference()
manifest.py   # reference/manifest.json: read() / write() / entry() - shared by the two import tools and the tests
placements.py # reference/placements.json -> build123d Location
models.py     # model_of() / raw() / geometry(inline=): the @step model of a module, its body, a child for an assembly
assembly.py   # assembly(name, children): the native labelled Compound node; label_shape / label_text from cadgen
cycloidal/    # the cycloidal drive: DriveConfig (params.py), layout.py, profiles.py, housing.py, disc.py, motor.py (THE NEMA 17 pilot + shaft, every motor's)
```

## Shared dimensions (DRY)
`lib/params.py` is the single source of truth: mm and grams, every constant tagged
`[MEASURE] / [DATASHEET] / [DESIGN] / [REFERENCE] / [ESTIMATE]` with a derivation comment. It re-exports the
leaves below it (`lib/units.py`, `lib/motors.py`, `lib/belts.py`, `lib/bearings.py`, `lib/fasteners.py`, `lib/sensors.py`: What is where) unchanged; a `lib/` package it
re-exports from (`lib/cycloidal/`, `lib/forearm/`, `lib/upper_arm/`, `lib/base/`, `lib/coupler/`) takes its globals from those leaves and its geometry
helpers from `lib/geom.py`, never from `lib.params` — and no leaf imports `lib.params` either
(`tests/test_layering.py LEAF_PACKAGES` / `LEAF_MODULES`). Datum: the SolidWorks capture
frame is **Y up** (J1 axis); the URDF base frame (REP-103) is `lib/datum.py base_frame()` (with `frame()`,
`U`, `BASE_FORWARD`; `robot/frames.py` re-exports them and builds the kinematics on top) — and
`assemblies/arm.py` emits the arm in it (`arm_from_w()`, see `assemblies/AGENTS.md`), so `arm.step` is **Z up**.
The cycloidal drive's own dimensions are `lib/cycloidal/params.py` (`DriveConfig`, frozen
dataclasses, variants via `dataclasses.replace`); `lib/params.py` re-exports the interface values
(`CYCLOIDAL_*`, masses) from it - never retype a drive number.

Changing a shared dimension — touchpoints in order:

| # | Edit | What |
|---|---|---|
| 1 | `lib/params.py` | the value (keep tag + derivation) |
| 2 | `tests/test_params_invariants.py` | the lock; `./cadtool pytest -m "not slow"` |
| 3 | `./cadtool gen parts/<group>/<affected>.py` | regenerate the STEP(s) (or just the arm: it rebuilds every stale part) |
| 4 | `./cadtool pytest` + `./cadtool gen assemblies/arm.py` + snapshot | verify geometry and fit |

## Kernel gotchas (all verified)
- `Compound.volume` skips nested sub-assemblies (build123d 0.10 and 0.11) — use `lib.reference.solid_volume()`.
- `is_valid` is a **property**; `Location.to_tuple()` is deprecated (`tuple(loc)` → two Vectors).
- `Face.center()` on a cylindrical face is the SURFACE midpoint (a half-cylinder reports axis ± r), not the axis:
  measure axes with `BRepAdaptor_Surface(face.wrapped).Cylinder().Axis()` (`tools/reference/split_mks_motor.py
  cylinders()`) or `Face.axis_of_rotation`, and bolt patterns from those, never from face centres.
- Cycloidal discs: chamfer the lobe edges BEFORE cutting holes (the end face must carry only the
  spline edge); the profile is a periodic *interpolating* spline (`Edge.make_spline(periodic=True)`,
  never `make_spline_approx`); OCCT's analytic volume is ~0.3 % off on that face (both ours and the
  reference) - compare tessellations/face sets, not `.volume`. A boolean between the two discs takes
  minutes: probe points instead.
- `wrist_link`'s fillet (`lib/wrist/link.py`): OCCT's default `.volume` (and `derive.py`'s inertials, the same
  integration) reads the trimmed fillet face ~0.06 % low, and the error jumps with unrelated construction changes (the
  fillet core's height); the adaptive integration `BRepGProp.VolumeProperties_s(shape.wrapped, props, 1e-7, False)` is
  stable and agrees with the reference - measure a change to that fillet with it (or cell-by-cell commons), not `.volume`.
  The fillet itself is made on a core carried over the tower's top: on the part the slope's edge with the bore ends in a
  point at the top, which `fillet()` cannot run through.
