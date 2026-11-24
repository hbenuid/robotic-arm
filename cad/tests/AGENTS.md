# tests/ — the pytest suite

Loads when you work in `tests/`. Run it only through `./cadtool pytest` (`cad/AGENTS.md`); `-m "not slow"` is the fast
lane. Every test module's docstring says what it checks.

## Running the suite (`tests/run.py`)
- `./cadtool pytest` runs the test FILES a change reaches: a file passed once is run again only when something it
  depends on changed since - its source, what it and its conftests import (read from the source, all the way down),
  what `picking.py` recorded while it last ran (the parts and modules it took by name and their imports, the source
  files it read as text, what a subprocess it started may run), and for every file all data and inputs (what is not a
  `.py` of `cad/`) and the list of every path. Nothing reached: nothing runs. The record is `cad/.picked.json`, per
  checkout; a file that fails, or does not run all its tests, leaves it. `--full` runs every file - **Recipe C step 5
  and before a pull request** (`cad/AGENTS.md`); tests named on the command line (a path, a node, `-k`, `-m`) run as
  named.
- A cache shared across test files caches through `picking.cache`, never `functools.cache` (`test_picking.py`
  checks): what its first computation took by name is recorded for every later caller - else a file taking a part
  another file built would not depend on it. A test that starts a subprocess running project code other than through
  `cadtool validate`, git or a `python -c` snippet whose imports `picking.subprocess_deps` can read makes its file
  depend on every `.py` - add the command there if it runs none.
- `-n <workers>` (pytest-xdist) spreads the files over processes, a whole file per worker (`pyproject.toml` sets
  `--dist=loadfile`: a file's fixtures and `built.py` shapes are built once). Unless `-n` is given, the runner takes
  the machine's performance cores and its free memory at `WORKER_MEMORY` a worker (each holds its own shapes, ~1.7 GB
  at most) - a second session's suite, running already, leaves less free and gets fewer -, never more than the files
  to run. A worker the system's out-of-memory killer takes does not fail the run, it hangs it (pytest runs outside the
  cad server's memory cap): when a run overruns, look for the kill in the system log and re-run with `-n 1`.
- On CI (`CI` set) the runner is plain pytest: what `ci.yml` names, nothing picked or recorded. CI runs the slow lane
  as parallel jobs (`.github/workflows/ci.yml`'s matrix): `test_sweeps.py` and `test_robot.py`, a test per worker
  (`--dist=worksteal`); parts that name the longest files or folders outright; and two catch-alls - the rest of
  `tests/` itself, the rest of its subfolders - that leave out every file a part names, so a new test file always
  lands in one. A part running well past the others is rebalanced from its `--durations`.

## Tests (`./cadtool pytest`)
`tests/conftest.py` blocks top-level model builds (tests call bodies:
`parts.build(name)`, `lib.models.raw(model)`), fails the test that changed a shape `built.py` shares, and keeps one
STEP reader process for the whole test process (`cad.engine.reader.session`, as a build does - outside one, cad starts
a reader for every read).
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
vs SolidWorks + the module lock — the numbers are IN that file; the arm's leaf colours - every part
`_occurrences.colour()`, in the arm and in the standalone gripper (the standalone drives' in their own tests), the
`lib/style.py` palette distinct, `ACCENT` only on printed parts, every `<joint>_belt` bought), `test_bom.py` (the print / buy lists
partition `parts.names()` by the flag, the occurrence counts, the drive's pieces follow `DEFAULT_CONFIG`, `EXTRAS`
well-formed, the parts at `LAYER_H_FINE` - its `FINE_LAYERS` set, printed parts only), `test_belts.py` (the GT2 belts, purchased with no reference: each a stock length, its loop that long
round its pulleys, the band on their tooth tips, their volume / bbox written in), `test_params_invariants.py` (locks),
`test_robot.py` (link partition, frames, FK at
zero = capture, the committed meshes vs a fresh export, inertials, `software/control`'s joint table vs the CAD's
reductions / limits / home angles (its `config.py` loaded by path), URDF/SRDF/SDF consistency + cad's URDF / SRDF validators via
`./cadtool validate`, which need the pinned server: `./cadtool server`), `test_tooling.py` (the installed cad and OCP
kernel are the pinned ones, one complete OCP distribution, no cad-server in the venv,
`./cadtool inspect` agrees with the kernel, the retired inspect verbs,
telemetry, cad's own updates and its plugin updates kept off, `./cadtool doctor`'s readers of what moves the machine,
no `cadgen` import, no `CADGEN_` name in the code and no second pin, `./cadtool cad` refusing what
moves the machine and its retired verbs saying what replaced them,
`./cadtool stop` ending only this venv's workers, the two `.claude/settings.json` the same and the plugin from upstream's
`latest` branch, no tracked `CLAUDE.md` / `CLAUDE.local.md`, every tracked name lowercase),
`test_mounts.py` (the mounted motors: axis on the joint, face on the host's pad / plate, board on the rear face, interference budget,
the base stack above the base bottom, the driven pulleys' planes within the shafts, the elbow motor's 20T level with its 90T; each belt joint's bearing stack between its
coupler's stub and its driven pulley; the 90T pulley bolts: seats, reach, the opened holes, the nuts' designed press; the base_yaw
thrust stack under `j1_coupler_plate`),
`test_layering.py` (the package layering, no `sys.path`, no direct part-module imports — AST scan),
`test_gauges.py` (the fit gauges, `lib/gauges`: the list well-formed, every arm value in its sweep, the fasteners'
clearance holes covered, the packing, imported without the kernel; slow: every coupon one solid on the bed, its round
features at its labels' sizes, the plates, a plate's STEP read back),
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
`part_colours`, an assembly's leaf colours),
`totals.py` (what an occurrence contributes to the arm / link totals: its SolidWorks record, or its own build once
converted — shared by `test_assembly.py` and `test_robot.py`), `built.py` (above), `run.py` / `picking.py` (Running
the suite, above; `test_picking.py` checks them: the shared caches, the subprocesses, the imports, the fingerprints),
`tests/cycloidal/` (the drive: one module per part + housing / purchased / fitment / assembly / port,
`from tests.cycloidal.helpers import CFG, …` for the drive's config, the
`stack` fixture is `tests/cycloidal/conftest.py`; the shell ring and the shell's body (`test_shell_body.py`: the stack
symmetric about the middle of the discs), with no reference, keep their numbers in their own modules),
`tests/upper_arm/` (`elbow_motor_screws`, `elbow_gearbox_screws`, `eg17_g5_gearbox`, `gt2_pulley_20t_8mm` - the elbow
motor's gearbox, its screws and its 20T, purchased parts with no reference: their numbers and feature probes, the
gearbox's against its datasheet drawing;
`elbow_home`: the elbow_pitch home sensor - the home angle, the frames through the elbow 90T's (the capture's, from
placements.json), the switch's zone past the lower end for every A3144 operate point and the glued magnet's error, the
layout under the 90T's face, its clip's, magnet's and fasteners' numbers (no reference), the clip's teardrop holes and
slots, its fit over the slab's end, the magnet on the 90T's face, over the chip at home and clear of the sensor and the
clip over the travel, the elbow's turning parts clear of them;
`elbow_idler`: the elbow belt's tensioner - the stock belt round it, the band of belts its slot takes, its stations,
its axle's, nut's and spacer's numbers (no reference), its slots on `j1_link`, the kit's fit beside the 90T; `j1_link`: the LEGACY build's feature probes, DEFAULT's numbers, the web under the elbow motor's gearbox (its seat) and its
holes, the arm - a flat slab - rising off the drive's turning shell - the drive's frame in the link's, its first pillar
on the arm's centreline, the shell's body inside, its windows under the arm solid as deep as the slab, open above it like the others, the slab's faces
level end to end -, no sockets, the elbow block's floor), `tests/base/` (`base`: the LEGACY build's
feature probes, the interface values lib/params.py and lib/datum.py take from it; DEFAULT's walls carrying
j1_coupler's cone on down onto the foot's flange, its screw holes, its volume / box (`test_foot`); `base_motor_mount`:
the base's other half - its walls, round end and foot the base's carried on, flush at the joint, its own screw holes -,
the wiring room, the joint's posts and bolts, both parts' slits, the seat on round holes short of the stock belt's
centre distance - its pad, its pilot's round hole, the counterbores -, the motor clear of it; `yaw_motor_screws`: the
base_yaw motor's 4x M3 (no reference) - its numbers, the roll motor's screw in the wrist motor's counterbores through
the plate and its pad, the heads flush in place; `yaw_idler`: the base_yaw belt's tensioner - the stock belt round it, the band of belts its slot
takes, the belt frame's points in the base's, its stations, its axle's, nut's and spacer's numbers (no reference), its
seat on the mount, the kit's fit; `yaw_home`: the base_yaw home sensor - the home angle, the switch's zone between the travel's ends for every A3144 operate point, the layout under the plate, its bracket's, magnet's and fasteners' numbers (no reference), the bracket's slots and holes and the plate's nut pockets, the fit, the magnet on the 120T's face and over the chip at home; `shoulder_home`: the shoulder_pitch home sensor - the home angle, the switch's zone past the lower end for every A3144 operate point and the glued magnet's error, the layout in the coupler's pocket, its tray's, magnet's and fasteners' numbers (no reference), the tray's holes and slots, the fit in the pocket, the magnet on its pillar, over the chip at home and clear of everything still over the travel; `yaw_pulley_screws` / `yaw_pulley_nuts`, purchased parts with no reference: the
numbers no reference file holds for them - pieces, volume, bbox - and feature probes; the `j1_coupler` fork's screws and
nuts and the screws + nuts that hold it on its plate - `fork_{hub,foot,sleeve}_{screws,nuts}`, `coupler_plate_{screws,nuts}`, `test_fork_bolts` - likewise), `tests/yaw_coupler/` (`j1_coupler`: the LEGACY
build's feature probes, the yoke on the port's housing - its bore, od, pillars, bolt circle; DEFAULT's fork on the
drive's own axis - two legs past the shell's ends, the disc's draft up them, the hub on one, the other its own part round the motor sleeve and bolted to it, the lowered ring - the motor leg's numbers, and the split: the plate under the cut at the pocket's floor, its numbers, the 4 screws' holes, counterbores and nuts' pockets), `tests/wrist/` (`wrist_link`: the LEGACY build's feature probes, the seat on the
coupler's flange, the end face on the NEMA 17 pattern, DEFAULT the wrist roll drive's frame on its plate and the wrist_pitch flange (the post) on it; the wrist roll
drive - its axis the wrist offset across the pitch axis, the stack, the stock belt, the rotor's walls, the press fits,
clean pairs against `wrist_link`, the pancake's way in from the bottom, the plate's slots on its tapped holes and its
pilot; `test_back_panel`: the wrist frame's back panel
and its magnet pairs (no reference) - their numbers, the panel flush and flat on the rear plane, `run_gap` off the
turning shaft, its rim and notches, the frame's step and pockets, the magnets on their pockets' bottoms, clear of the
drive; `test_roll_screws`: the drive's fasteners - the pancake's 4x M3, the rotor clamp's and the connector's 4x M3 +
nuts (no reference) - their numbers, their seats, the counterbores in the plate's front face the pancake's heads sit in, the
wrist 90T's nuts' way in from the pocket kept open; `test_roll_home`: the wrist_roll home sensor - the home angle and the magnet clear of the connector's nut slots,
the switch's zone past the lower end for every A3144 operate point and a magnet glued 1 mm off, the layout (the KY-003
upright, its chip `air` off the magnet, the fin's back flush with the far face, the screws' and nuts' lengths), its
fin's, magnet's and fasteners' numbers (no reference), the fin's holes, nut pockets and slots, its fit in the module
(`gap` off the frame, nothing past the far face), the magnet on the rim, over the chip at home and the rotor clear of
the sensor over the travel; `gripper_j3_connector`: LEGACY's two bodies, DEFAULT's bolted hub), `tests/gripper/` (the gripper's measured parts against their exports' numbers; the module: its layout, its fits, the jaws in mesh over their travel, the cage on the connector), `tests/pulley/` (`gt2_pulley_90t`: the GT2 groove's tangency
solve against the export's arcs, the LEGACY build's feature probes and surfaces, DEFAULT's opened holes and its hub
filled from the ring to the step;
`gt2_pulley_120t`, a part with no reference: its volume and bbox written in, the 90T's journal, ring and holes - its plain hub gone -, 120
grooves; `helpers.py`: probe points about the axis, the surface census),
`tests/forearm/` (the forearm: the LEGACY
build vs the SolidWorks part + feature probes, the roll end, the roll drive - its axis across the wrist pitch axis, stack, press fits, clean pairs,
clearances in the arm with the elbow folded; the roll belt's tensioner (`test_roll_idler`: the stock belt round it, its
riser, slot and nut channel on the roll frame, its axle's and nut's numbers - no reference) and the wrist belt's
(`test_wrist_idler`: the stock belt round it, the band of belts its slot takes, its stations, its axle's and nut's
numbers and its spacer's - no reference -, its slot and nut channel on `j2_link`, the kit's fit); `test_wall_split`: the
forearm printed in two parts - `j2_wall_bracket` and the lap's `wall_bracket_{screws,nuts}` (no reference: their
numbers), the lap's stations, the two parts the whole, the fasteners' fit; `test_wrist_home`: the
wrist_pitch home sensor - the home angle, the frame through the wrist 90T's (the capture's), the switch's zone past the
lower end for every A3144 operate point and the glued magnet's error, the layout under the 90T's face, its clip's,
magnet's and fasteners' numbers (no reference), the clip's teardrop holes and slots, its fit round the wrist boss's end,
the magnet on the 90T's face, over the chip at home and clear of the sensor and the clip over the travel, the wrist's
turning parts and the gripper clear of them; `test_roll_home`: the forearm_roll home sensor - the home angle, the
switch's zone past the lower end for every A3144 operate point and the carrier's play, the layout behind the shaft (the
chip `air` back, the header 0.98 off the shaft, the tube's axis into the motor's pocket behind the board's cover), its
carrier's, magnet's and fasteners' numbers (no reference), the carrier's teardrop holes, head pockets and tube, its fit
in the module (on the floor, the panel's rim in its notch, the wire hole open to the pocket), the magnet in the shaft,
over the chip at home and the rotor clear of the sensor over the travel; `test_wrist_motor_screws`: the
wrist-pitch motor's 4x M3 (no reference) - its numbers, the elbow motor's screw and seat, the counterbores on `j2_link`,
the heads flush in place; `test_roll_motor_screws`: the roll motor's 4x M3 (no reference) - its numbers, the same head
and counterbore through the roll frame's plate, its round holes and counterbores on the frame, the heads flush in the
module; `test_roll_clamp`: the roll rotor clamp's 4x M3 + nuts (no reference) - their numbers, on the rotor, the heads
on the pulley's counterbores' floors, the tips at the shaft's rear face, the nuts on their seats and held from turning
by the pockets, clear of the drive; `test_roll_wall_screws`: the forearm wall's 4x M3 + nuts into the roll pulley (no
reference) - their numbers, on the rotor, the wall's counterbores, the heads flush in them on the forearm, the tips flush
with the nuts' far faces, the nuts on their seats and held from turning by the pockets, clear of the drive and the
forearm; `test_roll_back_panel`: the roll frame's back panel and its magnet pairs (no
reference) - their numbers, the panel flush on the rear line and `run_gap` off the turning shaft, its rim, the frame's
step, opening and pockets, the magnets on their pockets' bottoms, clear of the drive; `helpers.in_host()` places any
occurrence in `j2_link`'s frame,
`helpers.in_drive_frame()` in the roll drive's mount frame - `j2_link`'s before the forearm's turn,
`lib/placements.py TURNS` - and `j2_link_in_drive_frame()` the turned forearm in it; `helpers.forearm()` is `j2_link`
fused with its wall bracket, the whole forearm the end's and the clearance tests probe). Geometry tests are
`slow`.

## Where the locks live
The totals the docs never quote (`cad/AGENTS.md` Docs) live in the locks — `tests/test_assembly.py`,
`assemblies/cycloidal_drive.py EXPECTED`, `assemblies/forearm_roll_drive.py EXPECTED`, `assemblies/wrist_roll_drive.py
EXPECTED`, `assemblies/gripper.py EXPECTED`, `reference/placements.json
expected`, `tests/test_bom.py`, `tests/test_placements.py`, `MULTI_BODY` in `tests/test_parts_convention.py`, the
interference budgets and press / clean pairs (`tests/test_mounts.py`, `tests/cycloidal/test_assembly.py`,
`tests/forearm/`, `tests/wrist/`, `tests/gripper/test_module.py BUDGET`), `tests/test_belts.py` (the belts), the numbers a *no reference*, *measured* or
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
- `distance_to` (BRepExtrema, and `cad.build123d.closest_points`, the same query - on the shapes' faces too) now and
  then returns 0 at one exact pose of two solids that neither touch nor overlap: the roll shaft against the roll frame
  read 0.000 at -140 and -145 deg (1.004 either side, no overlap), the frame against `j1_link` at elbow +92 (1.217
  either side), the wrist 90T against the wrist home clip at wrist_pitch +10 (2.4 either side, arm64 macOS). A zero is
  a false contact until a second look agrees - `helpers.distance` measures it again with the pose moved 0.01 mm three
  ways (`helpers.JITTER`) and keeps the median (`helpers.distance_under`, which the sweeps and the home sensors' travel
  measure with, hands it every reading under 0.05); an overlap (`helpers.interference`) is real.
- A whole-solid distance query is slow on a part of hundreds of faces (a 90T, a roll pulley, `j1_link`: up to seconds
  a pose, most of it on faces far from the other part) - a test measuring the same parts at many poses goes through
  `helpers.Reach` + `distance_under` / `nearest_under` (the faces and their boxes gathered once, BRepExtrema only on the
  faces in reach of each other; exact under its limit, the solids' insides counted), never `closest_points` in a loop.
