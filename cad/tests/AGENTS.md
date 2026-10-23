# tests/ — the pytest suite

Loads when you work in `tests/`. Run it only through `./cadtool pytest` (`cad/AGENTS.md`); `-m "not slow"` is the fast
lane. `-n <workers>` (pytest-xdist) spreads the suite over processes, a whole file per worker (`pyproject.toml` sets
`--dist=loadfile`: a file's fixtures and `built.py` shapes are built once) — `-n 4` on a dev machine and in CI's slow
lane (each worker holds its own shapes, ~1.5 GB at most; not `-n auto`, which starts one per core). CI runs the slow
lane as three jobs (`.github/workflows/ci.yml`): `test_sweeps.py`, a few long sweeps, a test per worker
(`--dist=worksteal`); the other files of `tests/` itself; its subfolders - each part leaving out the others' files, so
a new test file always lands in one.
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
module record + the mounted records vs `lib/mounts.py`, the records' `SHIFTS` and the forearm's `TURNS`
(`lib/placements.py`), the skipped nodes),
`test_assembly.py` (every assembly file ends with its build call; the arm's leaves / solids / volume / bbox
vs SolidWorks + the module lock — the numbers are IN that file; the arm's leaf colours -
purchased = `BOUGHT_TINT`, which no group / module may reuse, printed = the link's / module's tint, in the arm and in
the standalone gripper; the standalone drives' in their own tests), `test_bom.py` (the print / buy lists
partition `parts.names()` by the flag, the occurrence counts, the drive's pieces follow `DEFAULT_CONFIG`, `EXTRAS`
well-formed), `test_belts.py` (the GT2 belts, purchased with no reference: each a stock length, its loop that long
round its pulleys, the band on their tooth tips, their volume / bbox written in), `test_params_invariants.py` (locks),
`test_robot.py` (link partition, frames, FK at
zero = capture, the committed meshes vs a fresh export, inertials, URDF/SRDF/SDF consistency + cadgen's validators via
`./cadtool validate`), `test_tooling.py` (the installed cadgen and OCP kernel are the pinned ones, one complete OCP distribution,
`./cadtool inspect` agrees with the kernel, the retired inspect verbs, `daemon stop`'s pattern on both platforms,
telemetry kept off, the two `.claude/settings.json` the same, no tracked `CLAUDE.md` / `CLAUDE.local.md`, every
tracked name lowercase),
`test_mounts.py` (the mounted motors: axis on the joint, face on the host's pad / plate, board on the rear face, interference budget,
the base stack above the base bottom, the driven pulleys' planes within the shafts, the elbow motor's 20T level with its 108T; each belt joint's bearing stack between its
coupler's stub and its driven pulley; the 90T pulley bolts: seats, reach, the opened holes, the nuts' designed press; the base_yaw
thrust stack under `j1_coupler`),
`test_layering.py` (the package layering, no `sys.path`, no direct part-module imports — AST scan),
`test_sweeps.py` (the joint limits keep the arm off itself: exact distances between the robot links' parts at poses
on the limits - the upper arm over the shoulder's range against the fork and the base, the shoulder's upper limit over
the elbow's range, the wrist at its limits, the forearm and the wrist at the elbow's limits at any roll and wrist
pitch - the gripper up to `ELBOW_GRIPPER_FREE_DEG` -, the shoulder's limits at any yaw against the base, the forearm and the wrist roll's limits short of their hard
stops, the gripper rolling clear of the wrist, every MKS board's plug room over the joints' ranges; poses with the
arm down in the table skipped),
`test_lazy_kernel.py` (a fresh interpreter imports every template, part, assembly and link model without
loading `build123d` / `OCP`; names the first offender; its assembly list is kept by hand: `assemblies/AGENTS.md`),
`source_checks.py` (the shared `runs_its_model()` check that a model file ends with its build call),
`helpers.py` (the geometry helpers every geometry test shares: `interference`, `is_inside`, `section_area`, …, and
`module_tints`, a standalone module's colours),
`totals.py` (what an occurrence contributes to the arm / link totals: its SolidWorks record, or its own build once
converted — shared by `test_assembly.py` and `test_robot.py`), `built.py` (above),
`tests/cycloidal/` (the drive: one module per part + housing / purchased / fitment / assembly / port,
`from tests.cycloidal.helpers import CFG, …` for the drive's config, the
`stack` fixture is `tests/cycloidal/conftest.py`; the shell ring and the shell's body (`test_shell_body.py`: the stack
symmetric about the middle of the discs), with no reference, keep their numbers in their own modules),
`tests/upper_arm/` (`elbow_motor_screws`, a purchased part with no reference: its numbers and feature probes;
`elbow_idler`: the elbow belt's tensioner - the stock belt round it, the band of belts its slot takes, its stations,
its axle's, nut's and spacer's numbers (no reference), its slots on `j1_link`, the kit's fit beside the 108T; `j1_link`: the LEGACY build's feature probes, DEFAULT's numbers, the elbow motor's web and its
holes, the arm - a flat slab - rising off the drive's turning shell - the drive's frame in the link's, its first pillar
on the arm's centreline, the shell's body inside, its windows under the arm solid, the others open, the slab's faces
level end to end -, no sockets, the elbow block's floor), `tests/base/` (`base`: the LEGACY build's
feature probes, the interface values lib/params.py and lib/datum.py take from it; DEFAULT's walls carrying
j1_coupler's cone on down onto the foot's flange, its screw holes, its volume / box (`test_foot`); `base_motor_mount`:
the base's other half - its walls, round end and foot the base's carried on, flush at the joint, its own screw holes -,
the wiring room, the joint's posts and bolts, both parts' slits, the seat on round holes short of the stock belt's
centre distance - its pad, its pilot's round hole, the counterbores -, the motor clear of it; `yaw_motor_screws`: the
base_yaw motor's 4x M3 (no reference) - its numbers, the roll motor's screw in the wrist motor's counterbores through
the plate and its pad, the heads flush in place; `yaw_idler`: the base_yaw belt's tensioner - the stock belt round it, the band of belts its slot
takes, the belt frame's points in the base's, its stations, its axle's, nut's and spacer's numbers (no reference), its
seat on the mount, the kit's fit; `yaw_pulley_screws` / `yaw_pulley_nuts`, purchased parts with no reference: the
numbers no reference file holds for them - pieces, volume, bbox - and feature probes; the `j1_coupler` fork's screws and
nuts - `fork_{hub,foot,sleeve}_{screws,nuts}`, `test_fork_bolts` - likewise), `tests/coupler/` (`j3_coupler`: the
LEGACY build's feature probes, the stub the roll drive's block repeats), `tests/yaw_coupler/` (`j1_coupler`: the LEGACY
build's feature probes, the yoke on the port's housing - its bore, od, pillars, bolt circle; DEFAULT's fork on the
drive's own axis - two legs past the shell's ends, the disc's draft up them, the hub on one, the other its own part round the motor sleeve and bolted to it, the lowered ring - and the motor leg's numbers), `tests/wrist/` (`wrist_link`: the LEGACY build's feature probes, the seat on the
coupler's flange, the end face on the NEMA 17 pattern, DEFAULT the wrist roll drive's frame on its plate; the wrist roll
drive - its axis the wrist offset across the pitch axis, the stack, the stock belt, the rotor's walls, the press fits,
clean pairs against `wrist_link`, the pancake's way in; `gripper_j3_connector`: LEGACY's two bodies, DEFAULT's bolted
hub), `tests/pulley/` (`gt2_pulley_90t`: the GT2 groove's tangency
solve against the export's arcs, the LEGACY build's feature probes and surfaces, DEFAULT's opened holes;
`gt2_pulley_20_60t`, a measured conversion: its export's numbers written into the test - both bands' arc centres, the
surfaces, volume, bbox - and feature probes;
`gt2_pulley_108t` / `gt2_pulley_120t`, parts with no reference: their volume and bbox written in, the 90T's hub and
holes, 108 / 120 grooves; `helpers.py`: probe points about the axis, the surface census),
`tests/forearm/` (the forearm: the LEGACY
build vs the SolidWorks part + feature probes, the roll end, the roll drive - axis through the wrist centre, stack, press fits, clean pairs,
clearances in the arm with the elbow folded; the roll belt's tensioner (`test_roll_idler`: the stock belt round it, its
riser, slot and nut channel on the roll frame, its axle's and nut's numbers - no reference) and the wrist belt's
(`test_wrist_idler`: the stock belt round it, the band of belts its slot takes, its stations, its axle's and nut's
numbers - no reference -, its rib, slot and nut channel on `j2_link`, the kit's fit); `test_wrist_motor_screws`: the
wrist-pitch motor's 4x M3 (no reference) - its numbers, the elbow motor's screw and seat, the counterbores on `j2_link`,
the heads flush in place; `test_roll_motor_screws`: the roll motor's 4x M3 (no reference) - its numbers, the same head
and counterbore through the roll frame's plate, its round holes and counterbores on the frame, the heads flush in the
module; `test_roll_clamp`: the roll rotor clamp's 4x M3 + nuts (no reference) - their numbers, on the rotor, the heads
on the pulley's counterbores' floors, the tips at the shaft's rear face, the nuts on their seats and held from turning
by the pockets, clear of the drive; `helpers.in_host()` places any
occurrence in `j2_link`'s frame,
`helpers.in_drive_frame()` in the roll drive's mount frame - `j2_link`'s before the forearm's turn,
`lib/placements.py TURNS` - and `j2_link_in_drive_frame()` the turned `j2_link` in it). Geometry tests are
`slow`.

## Where the locks live
The totals the docs never quote (`cad/AGENTS.md` Docs) live in the locks — `tests/test_assembly.py`,
`assemblies/cycloidal_drive.py EXPECTED`, `assemblies/forearm_roll_drive.py EXPECTED`, `assemblies/wrist_roll_drive.py
EXPECTED`, `reference/placements.json
expected`, `tests/test_bom.py`, `tests/test_placements.py`, `MULTI_BODY` in `tests/test_parts_convention.py`, the
interference budgets and press / clean pairs (`tests/test_mounts.py`, `tests/cycloidal/test_assembly.py`,
`tests/forearm/`, `tests/wrist/`), `tests/test_belts.py` (the belts), the numbers a *no reference*, *measured* or
diverged part keeps in its own test (`parts/AGENTS.md` Part states) and the shared dimensions in
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
