# lib/ — shared dimensions and geometry code

Loads when you work in `lib/`. `lib/` is the left end of the layering (`cad/CLAUDE.md` "Layering"): it never imports
`parts/`, `assemblies/`, `robot/`, `tools/` or `tests/`. The rules for the modules that describe the assembly rather
than geometry live with their users: `mounts.py` (Recipe B) and `placements.py` (retired / skipped occurrences) in
`assemblies/CLAUDE.md`; the `reference.py` registries (`CUSTOM`, `COTS`, `DESIGNED`, `NATIVE`, …) in
`parts/CLAUDE.md` and `reference/CLAUDE.md`.

## What is where
```
params.py     # single source of truth for shared dimensions (tagged provenance)
units.py      # IN, NUDGE - a leaf module (lib/cycloidal/ imports it; params.py re-exports it)
motors.py     # the arm's motors (NEMA 17 interface, pancake, the 40 mm kit motor + MKS board, MOTOR_40) - a leaf, re-exported by params.py
belts.py      # GT2: the pulleys, pulley_od(), closed_belt_length() / centre_distance(), stock belt lengths - a leaf
cots.py       # hybrid(): the body of every purchased part (vendor STEP, else the envelope); pattern() for the multi-body ones
geom.py       # the arm's small build123d helpers: align_min(), cylinder(), through() (NUDGE overshoot), single_solid(), hex_prism() - a leaf
upper_arm/    # the upper arm (UpperArmConfig: LEGACY = the SolidWorks j1_link, DEFAULT = no cap sockets, the motor's and the drive's hole patterns)
forearm/      # the forearm (ForearmConfig: LEGACY = the SolidWorks j2_link, DEFAULT = the roll end, no cap sockets) and the roll drive (RollDriveParams, stack_positions, the block / shaft / retainer / 90T ring builders)
datum.py      # capture frame W -> base_link frame B: frame(), base_frame() (arm.py arm_from_w(), robot/frames.py); frames as data: IDENTITY, to_location()
mounts.py     # the motor mounts the SolidWorks capture never had (base_yaw / elbow_pitch / wrist_pitch motors + MKS boards) as frames-as-data
reference.py  # naming maps (SolidWorks custom/COTS, designed cycloidal parts, modules), loaders, path_of(), matches_reference()
manifest.py   # reference/manifest.json: read() / write() / entry() - shared by the two import tools and the tests
placements.py # reference/placements.json -> build123d Location
models.py     # model_of() / raw() / geometry(inline=): the @step model of a module, its body, a child for an assembly
assembly.py   # assembly(name, children): the native labelled Compound node; label_shape / label_text from cadgen
cycloidal/    # the cycloidal drive: DriveConfig (params.py), layout.py, profiles.py, housing.py, disc.py, motor.py (THE NEMA 17 pilot + shaft, every motor's)
```

## Shared dimensions (DRY)
`lib/params.py` is the single source of truth: mm and grams, every constant tagged
`[MEASURE] / [DATASHEET] / [DESIGN] / [REFERENCE] / [ESTIMATE]` with a derivation comment. It re-exports the
leaves below it (`lib/units.py`, `lib/motors.py`, `lib/belts.py`: What is where) unchanged; a `lib/` package it
re-exports from (`lib/cycloidal/`, `lib/forearm/`, `lib/upper_arm/`) takes its globals from those leaves and its geometry
helpers from `lib/geom.py`, never from `lib.params` — and no leaf imports `lib.params` either
(`tests/test_layering.py LEAF_PACKAGES` / `LEAF_MODULES`). Datum: the SolidWorks capture
frame is **Y up** (J1 axis); the URDF base frame (REP-103) is `lib/datum.py base_frame()` (with `frame()`,
`U`, `BASE_FORWARD`; `robot/frames.py` re-exports them and builds the kinematics on top) — and
`assemblies/arm.py` emits the arm in it (`arm_from_w()`, see `assemblies/CLAUDE.md`), so `arm.step` is **Z up**.
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
