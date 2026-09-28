# tests/ — the pytest suite

Loads when you work in `tests/`. Run it only through `./cadtool pytest` (`cad/CLAUDE.md`); `-m "not slow"` is the fast
lane. Every test module's docstring says what it checks.

## Tests (`./cadtool pytest`)
`tests/conftest.py` sets `CADGEN_DAEMON=0` and blocks top-level model builds (tests call bodies:
`parts.build(name)`, `lib.models.raw(model)`). `test_parts_convention.py` (contract + geometry for
every part, COTS envelopes + vendor frames), `test_reference_match.py` (manifest checksums; converted
parts vs reference), `test_placements.py` (JSON integrity, tables cover every key once, the designed
module record + the mounted records vs `lib/mounts.py`), `test_assembly.py` (every assembly file ends with its build call; the arm's leaves / solids / volume / bbox
vs SolidWorks + the module lock — the numbers are IN that file; the arm's leaf colours -
purchased = `BOUGHT_TINT`, which no group / module may reuse, printed = the link's / module's tint, in the arm and in
the standalone gripper and drives), `test_bom.py` (the print / buy lists
partition `parts.names()` by the flag, the occurrence counts, the drive's pieces follow `DEFAULT_CONFIG`, `EXTRAS`
well-formed), `test_params_invariants.py` (locks), `test_robot.py` (link partition, frames, FK at
zero = capture, the committed meshes vs a fresh export, inertials, URDF/SRDF/SDF consistency + cadgen's validators via
`./cadtool validate`), `test_tooling.py` (the installed cadgen and OCP kernel are the pinned ones, one complete OCP distribution,
`./cadtool inspect` agrees with the kernel),
`test_mounts.py` (the mounted motors: axis on the joint, face on the pad, board on the rear face, interference budget,
the base stack above the base bottom, the 90T planes within the shafts; the 90T pulley bolts: seats, reach, the opened
holes, the nuts' designed press; the base_yaw thrust stack under `j1_coupler`),
`test_layering.py` (the package layering, no `sys.path`, no direct part-module imports — AST scan),
`test_lazy_kernel.py` (a fresh interpreter imports every template, part, assembly and link model without
loading `build123d` / `OCP`; names the first offender — a new assembly model goes in its module list),
`source_checks.py` (the shared `runs_its_model()` check that a model file ends with its build call),
`helpers.py` (the geometry helpers every geometry test shares: `interference`, `is_inside`, `section_area`, …),
`totals.py` (what an occurrence contributes to the arm / link totals: its SolidWorks record, or its own build once
converted — shared by `test_assembly.py` and `test_robot.py`),
`tests/cycloidal/` (the drive: one module per part + housing / purchased / fitment / assembly / port,
`from tests.cycloidal.helpers import CFG, …` for the drive's config, the
`stack` fixture is `tests/cycloidal/conftest.py`), `tests/upper_arm/` (`j1_link`: the LEGACY build's feature probes, DEFAULT's
holes on the elbow motor's pattern and the drive's arm-mount bolts, no sockets, the elbow block's relief), `tests/base/` (`base`: the LEGACY build's
feature probes, the interface values lib/params.py and lib/datum.py take from it; `base_motor_mount`: the wiring room, the
joint to the base's posts, the slotted seat at the stock belt's centre distance, the motor clear across the travel), `tests/coupler/` (`j3_coupler`: the
LEGACY build's feature probes, the stub the roll drive's block repeats), `tests/yaw_coupler/` (`j1_coupler`: the LEGACY
build's feature probes, the yoke on the drive's housing - its bore, od, pillars, bolt circle; DEFAULT's sockets round the
6-pillar housing's two pillars), `tests/wrist/` (`wrist_link`: the LEGACY build's feature probes, the seat on the
coupler's flange, the end face on the NEMA 17 pattern), `tests/pulley/` (`gt2_pulley_90t`: the GT2 groove's tangency
solve against the export's arcs, the LEGACY build's feature probes and surfaces, DEFAULT's opened holes),
`tests/forearm/` (the forearm: the LEGACY
build vs the SolidWorks part + feature probes, the roll end, the roll drive - axis through the wrist centre, stack, press fits, clean pairs,
clearances in the arm with the elbow folded; `helpers.in_host()` places any occurrence in `j2_link`'s frame). Geometry tests are
`slow`.

## Where the locks live
The totals the docs never quote (`cad/CLAUDE.md` Docs) live in the locks — `tests/test_assembly.py`,
`assemblies/cycloidal_drive.py EXPECTED`, `assemblies/forearm_roll_drive.py EXPECTED`, `reference/placements.json
expected`, `tests/test_bom.py`, `tests/test_placements.py`, `MULTI_BODY` in `tests/test_parts_convention.py`, the
interference budgets (`tests/test_mounts.py`, `tests/forearm/`) and the shared dimensions in
`tests/test_params_invariants.py`. A geometry change bumps the ones its failures name — measure, never guess (Recipe C
step 5 in `cad/CLAUDE.md` has the `totals()` one-liner).

## Gotchas (all verified)
- `Shape.intersect` on composite operands changed in build123d 0.11 (a placed module against a part reported
  whole solids as common) — `tests/helpers.interference` runs the kernel's `BRepAlgoAPI_Common` directly: the
  two-shape constructor already runs the boolean (a `Build()` after it runs it all again), so it sets the operands on
  an empty operator, non-destructive (the default mode may modify them), and skips pairs whose bounding boxes are apart.
