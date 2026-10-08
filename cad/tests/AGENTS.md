# tests/ — the pytest suite

Loads when you work in `tests/`. Run it only through `./cadtool pytest` (`cad/AGENTS.md`); `-m "not slow"` is the fast
lane. `-n <workers>` (pytest-xdist) spreads the suite over processes, a whole file per worker (`pyproject.toml` sets
`--dist=loadfile`: a file's fixtures and `built.py` shapes are built once) — `-n 4` on a dev machine (each worker holds
its own shapes, ~1.5 GB at most; not `-n auto`, which starts one per core), `-n 2` in CI (`.github/workflows/ci.yml`).
Every test module's docstring says what it checks.

## Tests (`./cadtool pytest`)
`tests/conftest.py` sets `CADGEN_DAEMON=0` and blocks top-level model builds (tests call bodies:
`parts.build(name)`, `lib.models.raw(model)`), and fails the test that changed a shape `built.py` shares.
`built.py` builds each part, module and placed occurrence once per test process (`part`, `legacy`, `model`, `leaf`,
`placed`) — take geometry there, not from a fresh build (a build per test is most of what a slow test costs).
Those shapes are SHARED, so READ-ONLY: measure them and move copies (`.moved()`, `.rotate()`); never
relabel, re-parent, `.move()` / `.locate()` them, mesh them or hand them to build123d's booleans (its docstring says
what is safe) — a test that must do that builds its own (`tests/cycloidal/test_port.py` tessellates: fresh builds).
`test_parts_convention.py` (contract + geometry for
every part, COTS envelopes + vendor frames), `test_reference_match.py` (manifest checksums; converted and native
parts vs reference), `test_placements.py` (JSON integrity, tables cover every key once, the designed
module record + the mounted records vs `lib/mounts.py`), `test_assembly.py` (every assembly file ends with its build call; the arm's leaves / solids / volume / bbox
vs SolidWorks + the module lock — the numbers are IN that file; the arm's leaf colours -
purchased = `BOUGHT_TINT`, which no group / module may reuse, printed = the link's / module's tint, in the arm and in
the standalone gripper; the standalone drives' in their own tests), `test_bom.py` (the print / buy lists
partition `parts.names()` by the flag, the occurrence counts, the drive's pieces follow `DEFAULT_CONFIG`, `EXTRAS`
well-formed), `test_params_invariants.py` (locks), `test_robot.py` (link partition, frames, FK at
zero = capture, the committed meshes vs a fresh export, inertials, URDF/SRDF/SDF consistency + cadgen's validators via
`./cadtool validate`), `test_tooling.py` (the installed cadgen and OCP kernel are the pinned ones, one complete OCP distribution,
`./cadtool inspect` agrees with the kernel),
`test_mounts.py` (the mounted motors: axis on the joint, face on the host's pad / plate, board on the rear face, interference budget,
the base stack above the base bottom, the 90T planes within the shafts, the elbow motor's 20T level with its 90T; each belt joint's bearing stack between its
coupler's stub and its 90T; the 90T pulley bolts: seats, reach, the opened holes, the nuts' designed press; the base_yaw
thrust stack under `j1_coupler`),
`test_layering.py` (the package layering, no `sys.path`, no direct part-module imports — AST scan),
`test_sweeps.py` (the joint limits keep the arm off itself: exact distances between the robot links' parts at poses
on the limits - the upper arm over the shoulder's range against the fork and the base, the shoulder's upper limit over
the elbow's range, the wrist at its limits, the forearm and the gripper at the elbow's limits at any roll and wrist
pitch, the shoulder's limits at any yaw against the base, the forearm roll's limit short of its hard stop; poses with
the arm down in the table skipped),
`test_lazy_kernel.py` (a fresh interpreter imports every template, part, assembly and link model without
loading `build123d` / `OCP`; names the first offender — a new assembly model goes in its module list),
`source_checks.py` (the shared `runs_its_model()` check that a model file ends with its build call),
`helpers.py` (the geometry helpers every geometry test shares: `interference`, `is_inside`, `section_area`, …, and
`module_tints`, a standalone module's colours),
`totals.py` (what an occurrence contributes to the arm / link totals: its SolidWorks record, or its own build once
converted — shared by `test_assembly.py` and `test_robot.py`), `built.py` (above),
`tests/cycloidal/` (the drive: one module per part + housing / purchased / fitment / assembly / port,
`from tests.cycloidal.helpers import CFG, …` for the drive's config, the
`stack` fixture is `tests/cycloidal/conftest.py`; the shell ring and the shell's body (`test_shell_body.py`: the stack
symmetric about the middle of the discs), with no reference, keep their numbers in their own modules),
`tests/upper_arm/` (`elbow_motor_screws`, a purchased part with no reference: its numbers and feature probes; `j1_link`: the LEGACY build's feature probes, DEFAULT's numbers, the elbow motor's web and its
holes, the arm - a flat slab - rising off the drive's turning shell - the drive's frame in the link's, its first pillar
on the arm's centreline, the shell's body inside, its windows under the arm solid, the others open, the slab's faces
level end to end -, no sockets, the elbow block's floor), `tests/base/` (`base`: the LEGACY build's
feature probes, the interface values lib/params.py and lib/datum.py take from it; `base_motor_mount`: the wiring room, the
joint to the base's posts, the slotted seat at the stock belt's centre distance, the motor clear across the travel;
`yaw_pulley_screws` / `yaw_pulley_nuts`, purchased parts with no reference: the numbers no reference file holds for them - pieces, volume,
bbox - and feature probes), `tests/coupler/` (`j3_coupler`: the
LEGACY build's feature probes, the stub the roll drive's block repeats), `tests/yaw_coupler/` (`j1_coupler`: the LEGACY
build's feature probes, the yoke on the port's housing - its bore, od, pillars, bolt circle; DEFAULT's fork on the
drive's own axis - two legs past the shell's ends, the disc's draft up them, the hub on one, the other its own part round the motor sleeve, the lowered ring - and the motor leg's numbers), `tests/wrist/` (`wrist_link`: the LEGACY build's feature probes, the seat on the
coupler's flange, the end face on the NEMA 17 pattern), `tests/pulley/` (`gt2_pulley_90t`: the GT2 groove's tangency
solve against the export's arcs, the LEGACY build's feature probes and surfaces, DEFAULT's opened holes;
`gt2_pulley_20_60t`, a measured conversion: its export's numbers written into the test - both bands' arc centres, the
surfaces, volume, bbox - and feature probes;
`gt2_pulley_120t`, a part with no reference: its volume and bbox written in, the 90T's hub and holes, 120 grooves; `helpers.py`: probe points about the axis, the surface census),
`tests/forearm/` (the forearm: the LEGACY
build vs the SolidWorks part + feature probes, the roll end, the roll drive - axis through the wrist centre, stack, press fits, clean pairs,
clearances in the arm with the elbow folded; `helpers.in_host()` places any occurrence in `j2_link`'s frame). Geometry tests are
`slow`.

## Where the locks live
The totals the docs never quote (`cad/AGENTS.md` Docs) live in the locks — `tests/test_assembly.py`,
`assemblies/cycloidal_drive.py EXPECTED`, `assemblies/forearm_roll_drive.py EXPECTED`, `reference/placements.json
expected`, `tests/test_bom.py`, `tests/test_placements.py`, `MULTI_BODY` in `tests/test_parts_convention.py`, the
interference budgets (`tests/test_mounts.py`, `tests/forearm/`) and the shared dimensions in
`tests/test_params_invariants.py`. A geometry change bumps the ones its failures name — measure, never guess (Recipe C
step 5 in `cad/AGENTS.md` has the `totals()` one-liner).

## Gotchas (all verified)
- `Shape.intersect` on composite operands changed in build123d 0.11 (a placed module against a part reported
  whole solids as common) — `tests/helpers.interference` runs the kernel's `BRepAlgoAPI_Common` directly: the
  two-shape constructor already runs the boolean (a `Build()` after it runs it all again), so it sets the operands on
  an empty operator, non-destructive (the default mode may modify them), and skips pairs whose bounding boxes are apart.
  A line-to-line fit (coincident cylinders: a bearing in a seat of its own diameter) is the kernel's fragile case: one
  such pass on x86_64 Linux returned the whole bearing as common (arm64 macOS: 0), so the Common's fuzzy value
  `COINCIDENT_MM` makes faces that near one face.
- `distance_to` (BRepExtrema, and `cadgen.geometry.closest_points`, the same query - on the shapes' faces too) now and
  then returns 0 at one exact pose of two solids that neither touch nor overlap: the roll shaft against the roll frame
  read 0.000 at -140 and -145 deg (1.004 either side, no overlap), the frame against `j1_link` at elbow +92 (1.217
  either side). A zero is a false contact until a second look agrees - `tests/test_sweeps.py _distance` measures it
  again with the pose moved 0.01 mm three ways and keeps the median; an overlap (`helpers.interference`) is real.
