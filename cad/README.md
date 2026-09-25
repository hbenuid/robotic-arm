# robotic-arm — CAD (build123d)

**Last updated:** 2026-09-24 (the OCP CAD Viewer / `ocp-vscode` and `./cadtool show` removed — the CAD Viewer is cadgen's; ruff lint via `./cadtool lint` + the pre-commit hook `./cadtool setup` installs; GitHub Actions CI, run by hand) — see the root `CHANGELOG.md` for dated changes.

Parametric CAD-as-code for the desktop arm (base yaw, 20:1 cycloidal shoulder pitch, belt-driven
elbow and wrist pitch, wrist roll, MG996R parallel gripper), converted part-by-part from the original
SolidWorks design. This folder is a **separate uv project** (Python 3.12) — the motor-control
software in the repo root never depends on it.

**Status:** every SolidWorks custom part exists as an *import wrapper* around its reference
geometry (`reference/solidworks/<name>.step`), purchased parts use their vendor STEPs, and
`assemblies/arm.py` places all of them from placements extracted from the SolidWorks
assembly — so the whole arm already assembles, renders and is tested. Converting a part means
replacing its wrapper body with real build123d code (see "Converting a part"). The **20:1
cycloidal shoulder drive is fully parametric build123d** (`lib/cycloidal/`, 16 parts,
`assemblies/cycloidal_drive.py`), ported from the `cycloidal_drive` repo and verified against its
CadQuery exports — see [`docs/cycloidal_drive.md`](docs/cycloidal_drive.md).

## Setup (once per machine)

Requirements: [`uv`](https://docs.astral.sh/uv/), Node 20+ on `PATH` (only for STL/3MF/GLB export),
`git-lfs` on `PATH` — the plugin marketplace uses LFS **and so does this folder**: every committed
STEP/STL (the inputs in `reference/` and `vendor/`, plus `robot/meshes/`) is a Git LFS object, so `git lfs
install` once per machine before cloning (a clone that shows ~130-byte pointer files needs `git lfs pull`).
Generated STEPs — each part's `parts/<group>/<name>.step` included — are **git-ignored**: their bytes differ
per machine, so every machine builds its own (`./cadtool gen assemblies/arm.py`, ~35 s the first time) — and
the [`cad@text-to-cad`](https://github.com/earthtojake/text-to-cad) Claude Code plugin **v0.6.x**
(the repo's `.claude/settings.json` enables its marketplace; install/update with
`claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git`,
`claude plugin install cad@text-to-cad`, later `claude plugin marketplace update text-to-cad &&
claude plugin update cad@text-to-cad` — once per scope, `--scope project` for the repo's own install). The plugin only carries the `/cad:*` skill docs; the toolchain
itself is the [`cadgen`](https://pypi.org/project/cadgen/) package installed into this venv.

```bash
cd cad
./cadtool setup        # uv sync (build123d/OCP/cadgen into ./.venv; reinstalls the OCP kernel if it does not import)
                       # + the git pre-commit hook (ruff on staged .py files) + Playwright Chromium (~150 MB, snapshots only)
./cadtool pytest       # everything green?
```

CI (GitHub Actions, `../.github/workflows/ci.yml`, run by hand: `gh workflow run ci.yml --ref <branch>`) runs the lint,
the whole suite and `./cadtool gen` of both arms in a clean clone on a fresh Ubuntu runner (~13 min); when it is worth
running: the root `CLAUDE.md` "CI".

Python is pinned to **3.12** in `.python-version` (the system Python is 3.14; the OCP
wheels are verified on 3.12 — bump deliberately, with the full suite). Never run bare `python` here —
always `./cadtool …` or `uv run …` from `cad/`.

`cadgen` (the text-to-cad runtime, on PyPI) is a **locked dependency** pinned to the installed
plugin version (`cadgen[snapshot]==…` in `pyproject.toml`; the plugin's `skills/cad/requirements.txt`
pins the same) — bump both together; `./cadtool doctor` checks the pair, the CAD kernel, Node and
Chromium. cadgen makes hard cutovers: 0.5 had **no compatibility with 0.4** (2026-09-11 CHANGELOG entry),
0.6 cut the cache / sidecar schemas and 0.6.5 removed `cadgen step inspect` (2026-09-18 entry); 0.6.6 was
additive (2026-09-22 entry).

## `./cadtool` — the one entry point

| Command | What it does |
|---|---|
| `./cadtool gen parts/<group>/<name>.py` (alias `step`) | **run the model script**: writes `parts/<group>/<name>.step` beside it (git-ignored); a second run prints `current …` (freshness gate), `--force` rebuilds |
| `./cadtool gen assemblies/arm.py` | build `assemblies/arm.step` (git-ignored) — calls every part model, so each stale part is rebuilt (in parallel) and its STEP rewritten |
| `./cadtool gen assemblies/arm_no_caps.py` | build `assemblies/arm_no_caps.step` — the same arm without `j1_cap`, `j2_cap_1`, `j2_cap_2` (a working view for the links under them; `arm.step` stays the robot). One-off image instead: `./cadtool snapshot assemblies/arm.step out.png --hide '#j1_cap' --hide '#j2_cap_1' --hide '#j2_cap_2'` |
| `./cadtool python tools/bom.py [--module cycloidal_drive\|gripper] [--md\|--json]` | the **print list** and the **buy list** (what to order, pieces, mass), generated from the assembly tables + each part's `COTS` flag; ends with the purchased items that are not modelled (`EXTRAS`). No CAD kernel, instant |
| `./cadtool python tools/export_printables.py [--parts …]` | one STL per **printed** part into `print/` (git-ignored; mm, part-local frame) with the quantity to print; bought parts are refused |
| `./cadtool why <model.py>` | why the model is current or stale, clause by clause (`cadgen store why`) |
| `./cadtool export <file.step> stl\|3mf\|glb [out]` | one mesh file per call from a STEP document (Node 20+; `--mesh-tolerance` is *relative*, default 1.5e-3 of the bounding diagonal) |
| `./cadtool inspect <file.step> [--planes] [--json]` | leaf refs, solids, faces, volume, bbox of a saved STEP (`--planes`: its planar faces as normal / offset / area) — a **local** tool, `tools/step_facts.py`: cadgen 0.6.5 removed `cadgen step inspect`; distances and overlaps are `cadgen.geometry.closest_points` / `overlap_volume` in a test |
| `./cadtool inspect diff <a.step> <b.step> [--tol X]` | same geometry leaf by leaf? exit 1 if not (checking a regenerated STEP against a copy kept from before the change) |
| `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly --view-labels` | PNG review still (`--job job.json` for a multi-view packet; the path you name is the file written) — also `.urdf`/`.sdf`/`.stl` inputs |
| `./cadtool viewer [--port N]` | CAD Viewer serving this folder — `http://127.0.0.1:3245/?file=assemblies/arm.step` (ships inside cadgen; stops after 12 h or Ctrl+C; `./cadtool cadgen viewer list\|stop --port N`) |
| `./cadtool validate robot/arm.urdf --strict` (`.srdf`, `.sdf --gz-check never`) | robot-description validators |
| `./cadtool parts "<query>" [--download --id <id> --filename <name>.step]` | step.parts search / download into `vendor/` |
| `./cadtool skill <skill> <tool> [args]` | a plugin skill script (`dfam-check dfam_tool.py`, `gcode gcode_tool.py`, …) |
| `./cadtool cadgen …` / `store …` / `daemon …` | any `cadgen` subcommand (`store info\|gc`, `daemon status`, `doctor`); `./cadtool daemon stop` ends the warm build daemon + workers (cadgen has no stop verb) |
| `./cadtool doctor` | installed cadgen vs the plugin's pin, Node, Playwright Chromium |
| `./cadtool pytest [-m "not slow"]` | test suite (the fast lane skips geometry builds) |
| `./cadtool lint [--fix] [path…]` | `ruff check` over `cad/` (rules in `pyproject.toml [tool.ruff]`; lint only - no `ruff format`, the tables are hand-aligned) |
| `./cadtool python …` | any python in the venv with `PYTHONPATH=cad/` (`-c "from assemblies.cycloidal_drive import totals; print(totals())"`) |
| `./cadtool clean [--all]` | delete `__pycache__/`, `.pytest_cache/`, `.ruff_cache/` (and any stray `__cadgen__/`); `--all` also empties `snapshots/` and removes the git-ignored `assemblies/*.step`, `robot/links/*.step` |

`cadtool` always `cd`s to `cad/` (cadgen resolves paths from the working directory and the viewer
serves it) and exports `PYTHONPATH=cad/` so `lib`, `parts`, `assemblies`, `robot` import the same way
for you, for cadgen's dependency scan and for its warm build daemon. Everything derived — trees,
tessellations, the freshness records — lives in `~/.cache/cadgen` (`./cadtool store gc` sweeps it;
deleting it is always safe). `CADGEN_DAEMON=0` runs a build on transient workers instead of the daemon.

This project runs cadgen 0.6.6 on **build123d 0.11.1 / OCP 7.9.3** — cadgen 0.6 requires
`build123d>=0.11.1,<0.12` and `cadquery-ocp-novtk>=7.9,<8`, and `pyproject.toml` pins the exact kernel
(the STEP bytes are deterministic per kernel and per machine). Never add `cadquery-ocp` (the VTK build cadgen
0.5 pulled): both distributions own the same `OCP/` files, so uv removing one guts the other —
`uv sync --reinstall-package cadquery-ocp-novtk` repairs it (`setup` does, `tests/test_tooling.py`
checks). On the older pair (build123d 0.10.0 / OCP 7.8.1) cadgen could not build 11 of the 41 parts (OCCT 7.8.1
mis-read its BinTools VERSION_4 component objects), could not export a linked child (`wrapped` is a
property only since build123d 0.11) and its STEP writer needs OCP 7.9's `HArray1.Value`.

## Layout

```
cad/
├── cadtool                # bash wrapper around the cadgen CLI + uv (see above)
├── .env                   # PYTHONPATH=. for the VS Code Python extension (cadtool/pytest set it themselves)
├── lib/
│   ├── params.py          # single source of truth for shared dimensions (tagged provenance)
│   ├── units.py           # IN, NUDGE - a leaf module (lib/cycloidal/ imports it; params.py re-exports it)
│   ├── motors.py          # the arm's motors (NEMA 17 interface, pancake, the 40 mm kit motor + MKS board, MOTOR_40) - a leaf, re-exported by params.py
│   ├── belts.py           # GT2: the pulleys, pulley_od(), closed_belt_length() / centre_distance(), stock belt lengths - a leaf
│   ├── forearm/           # the forearm (ForearmConfig: LEGACY = the SolidWorks j2_link + caps, DEFAULT = the roll end) and the roll drive (RollDriveParams, stack_positions, the block / shaft / retainer / 90T ring builders)
│   ├── datum.py           # capture frame W -> base_link frame B: frame(), base_frame() (arm.py arm_from_w(), robot/frames.py); frames as data: IDENTITY, to_location()
│   ├── mounts.py          # the motor mounts the SolidWorks capture never had (base_yaw / elbow_pitch / wrist_pitch motors + MKS boards) as frames-as-data
│   ├── reference.py       # naming maps (SolidWorks custom/COTS, designed cycloidal parts, modules), loaders, path_of(), matches_reference()
│   ├── manifest.py        # reference/manifest.json: read() / write() / entry() - shared by the two import tools and the tests
│   ├── placements.py      # reference/placements.json -> build123d Location
│   ├── models.py          # model_of() / raw() / geometry(inline=): the @step model of a module, its body, a child for an assembly
│   ├── assembly.py        # assembly(name, children): the native labelled Compound node; label_shape / label_text from cadgen
│   └── cycloidal/         # the cycloidal drive: DriveConfig (params.py), layout.py, profiles.py, housing.py, disc.py, geom.py, motor.py (THE NEMA 17 pilot + shaft, every motor's)
├── parts/                 # one part per file, grouped by subsystem; parts.names() / parts.load(name) discover them
│   ├── __init__.py            # the directory scan: MODULES / GROUPS, names(), load(), model(), build(), bought(), source_of()
│   ├── _templates/            # designed.py (parametric), wrapper.py (import wrapper), cots.py (purchased) templates
│   ├── base/                  # base, j1_coupler, j1_link, j1_cap                                  (SolidWorks wrappers)
│   ├── joints/                # j2_link, j2_cap_1, j2_cap_2 (parametric - lib/forearm/), j3_coupler, gt2_pulley_90t (SolidWorks wrappers) + COTS nema17_40mm, mks_servo42d (the belt joints' MKS kits) + the roll drive: forearm_roll_block / _shaft / _retainer (native), bearing_6808 (native COTS)
│   ├── wrist/                 # wrist_link, gripper_clamp_bracket, gripper_j3_connector + COTS nema17_pancake, gt2_pulley_20t
│   ├── gripper/               # gripper_* (7), servo_holder + COTS gripper_rail_6mm, mg996r_servo, mg996r_horn
│   └── cycloidal/             # the drive: 6 designed parts + 10 COTS (bearings, nema17_48mm - vendor file composed from the kit exports -, pins, bolts, nuts), _cots.py helper
│       └── <name>.py + <name>.step   # every group: running the .py writes the .step beside it (git-ignored, per machine)
├── assemblies/
│   ├── arm.py             # the whole arm, grouped arm -> base_link/shoulder_link/upper_arm_link/elbow_link/forearm_link/wrist_pitch_link/wrist (GROUPS; the SolidWorks occurrences + the mounted motors and boards of lib/mounts.py + the three modules, printed parts tinted per group, purchased parts grey)
│   ├── arm_no_caps.py     # working view: arm.py's tables minus HIDDEN (the three link caps) - not the robot
│   ├── gripper.py         # the gripper mechanism module (its occurrences placed from placements.json)
│   ├── cycloidal_drive.py # the drive module (rows placed from lib/cycloidal stack_positions - code-driven, the MKS board included; totals in EXPECTED)
│   ├── forearm_roll_drive.py # the forearm roll module (rows from lib/forearm stack_positions; placed by lib/mounts.py MODULE_MOUNTS; stator / rotor BODIES; totals in EXPECTED)
│   └── _occurrences.py    # place()/occurrence_children()/grouped_children() (placement keys), place_at()/located_children() (Locations), world_rows(); BOUGHT_TINT / _tint_parts (purchased parts grey) - children via lib.models.geometry()
├── docs/cycloidal_drive.md  # the drive's spec, port notes and attachment
├── docs/forearm_roll.md     # the forearm roll drive's spec, fits, assembly sequence and attachment
├── docs/open_issues.md      # THE list of unsettled fits, estimates, unmodelled hardware and unconfirmed mappings
├── reference/             # immutable per-part reference STEPs (Git LFS) + manifest.json + placements.json + README
│   ├── solidworks/            # the 25 SolidWorks exports (custom parts + the SolidWorks purchased parts)
│   └── cycloidal/             # the 16 CadQuery exports the drive was ported from
├── vendor/                # purchased-part STEPs (committed via Git LFS; replaceable by better catalog models)
├── robot/                 # URDF / SRDF / SDF + per-link meshes and generators (see below)
├── print/                 # git-ignored: one STL per printed part (tools/export_printables.py)
├── tools/                 # step_facts.py (./cadtool inspect), bom.py (print list + buy list), export_printables.py, reference/{import_solidworks,extract_placements,mount_placements,split_mks_motor}.py (SolidWorks exports -> reference/vendor/placements),
│                          # cycloidal/{export_cadquery,import_cadquery}.py (the drive's references), robot/{derive,export_link_meshes}.py
├── tests/                 # pytest: conventions, reference match, placements, assembly totals, params locks, robot description, package layering, the motor mounts (test_mounts.py);
│   ├── conftest.py            # CADGEN_DAEMON=0 + a guard that fails any test calling a model at top level (tests call bodies)
│   ├── cycloidal/             # the drive's own tests (one module per part + housing / purchased / fitment / assembly / port) + helpers.py (CFG, geometry helpers), conftest.py (stack fixture)
│   └── forearm/               # the forearm's tests: the LEGACY builds vs the SolidWorks parts, the roll end, the roll drive (fits, clearances at the capture pose and with the elbow folded)
└── snapshots/             # snapshot PNGs (git-ignored)
```
The `parts/` groups follow the **physical stage along the arm** (`base` → `joints` → `wrist` →
`gripper`, plus the `cycloidal` drive) — not the name prefix and not the URDF links: the `j1_*` parts
sit in `base/` with the stage they build, and `gripper_clamp_bracket` / `gripper_j3_connector` sit in
`wrist/` because they are the wrist-side mount (`gripper_j3_connector` is nevertheless placed by
`assemblies/gripper.py`). A group is only a directory — the part's name is the key everywhere.

Nothing derived lands in the tree: cadgen keeps trees, tessellations and freshness records in its
content-addressed store (`~/.cache/cadgen`). A `<name>.step.json` sidecar appears beside a STEP only
when its model declares `kinematics=` (none does yet); it would be committed beside the model.

## Part conventions

Each `parts/<group>/<name>.py` is a **cadgen model**: one module-level function named after the
file and decorated with `@step` (`from cadgen import step`), built by running the file (the group
directory is only a folder — the part's *name* is the module stem, unique across groups, and it
keys `reference/manifest.json`, `reference/<origin>/<name>.step`, `placements.json` and the URDF
links; code never imports a part statically but goes through `parts.load(name)` / `parts.model(name)`
/ `parts.build(name)` / `parts.names()`, which scan the group packages):

- `@step def <name>()` **returns** the final `Part`/`Compound` at the part's **local origin** — the
  assembly owns placement; no parameters, no `out=` (the STEP is the sibling file); the file ends with
  `if __name__ == "__main__": <name>()` — that call is the build, so importing the module has **no side
  effects** (build with `./cadtool gen`, look with `./cadtool viewer` / `snapshot`). Imports are plain (`from lib import …`): `cadtool`,
  pytest and `.env` put `cad/` on the import path, and cadgen imports the file as `parts.<group>.<name>`;
- **calling the model outside a build runs the pipeline** (writes the STEP, uses the daemon) — tests and
  tools call the body instead (`parts.build(name)` / `lib.models.raw(model)`), assemblies call
  `lib.models.geometry(model)` (linked child inside a build, body otherwise);
- the result is labelled with the part name; shared dimensions come from `lib/params.py`;
- the CAD kernel is imported **lazily** — `from cadgen import build123d as bd`, `bd.<name>` inside function
  bodies only, no kernel object at module level — so cadgen can tell an unchanged model is current without
  paying for OCP (~0.1 s instead of ~1.5 s per run; `tests/test_lazy_kernel.py` locks it for every model);
- **custom parts** declare `REFERENCE` (their `reference/solidworks/<name>.step`), `CONVERTED`
  (`False` while the file is a wrapper) and `LOCAL_FROM_REF` (rigid transform from the
  SolidWorks part-file frame to the part's local frame, as data `((x, y, z), (rx, ry, rz))` in mm / degrees;
  `lib.datum.IDENTITY` for wrappers — `lib.datum.to_location()` makes the Location inside the model);
- **designed parts** (the cycloidal drive's printed parts, `lib/reference.py DESIGNED`) are the
  same contract with `CONVERTED = True` and a reference that is the CadQuery export they were
  ported from — the strict `tests/cycloidal/test_port.py` checks them beyond the reference match;
- **COTS parts** declare `COTS = True`, `MASS_G`, `VENDOR_STEP` and `VENDOR_TO_REF`, and the model
  returns the vendor STEP (`cadgen.read_step` — a tracked input, so swapping the file makes the part
  stale — re-oriented by `VENDOR_TO_REF`) when present, else a parametric `_envelope()`. They also say
  what to order: `PURCHASE_SPEC`, `PURCHASE_QTY` (pieces per occurrence — a whole pattern for the drive's
  pin and fastener parts) and an optional `PURCHASE_NOTE`.

`tests/test_parts_convention.py` enforces all of this automatically for every discovered part.
(cadgen's default output is the sibling `<name>.step` and the viewer pairs the two — which is why a
part and its STEP always live in the same directory; upstream's `src/` + `STEP/` layout is not used.)

## Converting a part (wrapper → parametric)

1. Inspect the reference: `./cadtool inspect reference/solidworks/<name>.step --planes`
   (dimensions: the CAD Viewer's measure tool, or build123d on `cadgen.read_scene(…).resolve("#o1.f7").shape()`);
   `parts/<group>/<name>.py`'s docstring lists units,
   solids, bounding box and where it is used.
2. Rewrite the model body with `BuildPart`/… code. Put every shared dimension in `lib/params.py`
   with a provenance tag (`[MEASURE]`, `[DATASHEET]`, `[DESIGN]`, `[REFERENCE]`, `[ESTIMATE]`)
   and a lock in `tests/test_params_invariants.py`.
3. Set `CONVERTED = True`. If you pick a nicer local origin than the SolidWorks one, set
   `LOCAL_FROM_REF` to the transform *reference frame → new local frame* (frame data, see above); the
   assemblies compose `placement * to_location(LOCAL_FROM_REF)⁻¹`, so `reference/placements.json` never changes.
   (`j1_cap` and `j2_cap_2` have their geometry ~1 m from the SolidWorks origin — they are the
   ones that want this.)
4. `./cadtool pytest tests/test_reference_match.py -k <name>` — volume within 0.5 % and bounding
   box within 0.2 mm of the reference (per-part overrides: `REF_VOL_TOL`, `REF_BBOX_TOL`).
5. `./cadtool gen parts/<group>/<name>.py` to regenerate the STEP, then
   `./cadtool gen assemblies/arm.py` + `./cadtool snapshot …` to eyeball it in place.

## Purchased parts and step.parts

Each COTS part keeps its SolidWorks re-export in `reference/solidworks/<name>.step` (the frame
and size reference; the drive's purchased parts keep their CadQuery export in
`reference/cycloidal/`) and its current best model in `vendor/<name>.step`. To try a catalog model:
`./cadtool parts "<query>"` → pick an id → `./cadtool parts --id <id> --download --filename
<name>.step --overwrite` → `./cadtool inspect vendor/<name>.step --planes`
→ set `VENDOR_TO_REF` in `parts/<group>/<name>.py` so the model lands in the SolidWorks frame →
`./cadtool pytest -k <name>` (`test_cots_vendor_matches_reference_frame`: bbox within 1.5 mm of
the reference) → `./cadtool gen parts/<group>/<name>.py` → `./cadtool python tools/reference/import_solidworks.py`
(updates `manifest.json`; `tools/cycloidal/import_cadquery.py` for the drive's parts). If the
catalog model is worse, restore the reference copy. See `vendor/README.md` for what has been tried.

## Printed vs. bought

Every part carries the make/buy label once: `COTS = True` in its module means **bought**, anything else is
**printed** (`parts.bought(name)`). Nothing else is kept by hand — the folders follow the arm's physical
stages, not make/buy:

- **Lists** — `./cadtool python tools/bom.py` prints what to print (part, quantity) and what to buy
  (`PURCHASE_SPEC`, pieces = occurrences × `PURCHASE_QTY`, mass, vendor file or envelope), counted from
  the assembly tables; `--module cycloidal_drive` for the drive alone. Purchased items that are **not
  modelled** (the drive's arm-mount bolts and nuts, grease) are the one hand-kept table, `EXTRAS` in that
  tool: they are on the buy list, not in the model, the totals or the inertials.
- **Colours** — every assembly (`arm.step`, `arm_no_caps.step`, `gripper.step`, `cycloidal_drive.step`) shows a
  purchased part in the one `_occurrences.BOUGHT_TINT` grey; a printed part carries its link's or its module's colour
  (`gripper.TINT`, `cycloidal_drive.TINT`). Grey always means bought — no link or module tint reuses it. There is
  one STEP per assembly: no separate make/buy copies.
- **STLs** — `./cadtool python tools/export_printables.py` writes `print/<name>.stl` for every printed part.

## Assembly

`assemblies/arm.py` and `assemblies/gripper.py` hold `OCCURRENCES` tables —
`(part, role, placement_key)` in SolidWorks document order — and place each part model with
`lib.placements` (`"j3_coupler#2"` = second occurrence of that part; inside a cadgen build the child
model is called — built in parallel, its STEP rewritten when stale — and the gripper links its tree
while the tinted arm keeps an inline copy; outside a build it is a fresh in-process copy). The
placements are in the SolidWorks capture frame (+Y up), but the arm is **emitted Z up**, in the
`base_link` frame of `robot/frames.py` (`arm.py arm_from_w()`, composed into every occurrence's
placement): cadgen's viewer and snapshots treat +Z as up and have no up-axis setting, so a
capture-frame `arm.step` would lie on its side. `arm.step` and `robot/arm.urdf` therefore open in the
same pose, the base standing on z = 0. Roles make
duplicate parts' labels unique (`j3_coupler:j2`, `gripper_end:1`); they are positional for
now. `assemblies/cycloidal_drive.py` and `assemblies/forearm_roll_drive.py` are **code-driven modules**: their rows are
`(part, role, placement)` computed from `lib/cycloidal` / `lib/forearm` (`stack_positions`), and `arm.py` locates
the whole module at its pose (`placements.json` `cycloidal_drive#1`, a `designed` module record from the
SolidWorks node; `forearm_roll_drive#1`, the same kind of record written from `lib/mounts.py MODULE_MOUNTS`).
`arm.py GROUPS` buckets the occurrences into the component tree
`arm -> base_link/shoulder_link/upper_arm_link/elbow_link/forearm_link/wrist_pitch_link/wrist` — the rigid-link
partition of `robot/frames.py LINKS` with the three modules kept whole (the cycloidal drive under
`shoulder_link` although `LINKS` puts its rotor body in `upper_arm_link`, the roll drive under `elbow_link`
although its rotor - the shaft - belongs to `forearm_link`) — so each component
toggles as one node in the viewers, and
every subtree is tinted with its group's color (the gripper and the two drive modules keep
their own). `tests/test_assembly.py` checks the rebuilt arm against the SolidWorks totals plus
the module's own lock (leaves, solids, volumes, bbox - the numbers live in the test and in
`assemblies/cycloidal_drive.py EXPECTED`, never in the docs), the group labels and the `LINKS` mirror. The
leaves the SolidWorks capture never had - the belt joints' motors (48 mm at the base, 40 mm at the elbow and
wrist) and their MKS SERVO42D boards - are declared in `lib/mounts.py` and materialised into `placements.json`
by `tools/reference/mount_placements.py` (see `CLAUDE.md` "Mounted occurrences").

## Reference geometry and placements

`reference/solidworks/*.step` are renamed copies of the SolidWorks exports in
`~/Documents/arm_assembly_organized/` — see `reference/README.md` for the naming map;
`reference/cycloidal/*.step` are the CadQuery exports the drive was ported from
(`lib.reference.path_of(name)` resolves the origin, `manifest.json` records it in `file`).
`reference/placements.json` holds every occurrence's placement extracted from the full-assembly
STEP. Both are **immutable inputs** (a checksum test guards them); regenerate with
`./cadtool python tools/reference/import_solidworks.py` and `./cadtool python tools/reference/extract_placements.py --no-pancake`
if the SolidWorks design changes (the exact sequence, the motor-kit exports and `split_mks_motor.py` /
`mount_placements.py`: `reference/README.md` "Provenance"; the mounted occurrences: `lib/mounts.py`). The cycloidal drive's 16 references are the CadQuery exports of
the `cycloidal_drive` repo at `2f1f67d` (`tools/cycloidal/export_cadquery.py` in that repo's venv,
then `./cadtool python tools/cycloidal/import_cadquery.py`), and its SolidWorks node is recorded
in `placements.json` as a designed module (pose only; the contents come from code).

## Robot description (URDF / SRDF / SDF)

`robot/` holds the arm's robot description, derived from the CAD:

```
robot/
├── frames.py          # THE kinematic decomposition: LINKS (which occurrences move together) + JOINTS
│                      # (axis point/direction, parent/child, limits from lib/params.py)
├── _links.py          # link_rows() / build_link(): one link's occurrences placed in the link frame (models, meshes, tests)
├── links/<link>.py    # a @step model per rigid link, in the link's own frame (./cadtool gen robot/links/shoulder_link.py)
├── meshes/<link>.stl  # per-link meshes in mm (committed) - tools/robot/export_link_meshes.py
├── arm.urdf           # SOURCE OF TRUTH (hand-edited ledger + numbers from tools/robot/derive.py)
├── arm.srdf           # MoveIt2 semantics: chain base_link->tool0, gripper group, home/open/closed states
└── arm.sdf            # model-level SDF 1.12 derived from the URDF
```

Links: `base_link → base_yaw → shoulder_link → shoulder_pitch → upper_arm_link → elbow_pitch → elbow_link →
forearm_roll → forearm_link → wrist_pitch → wrist_pitch_link → wrist_roll → wrist_roll_link → jaw_a / jaw_b
(prismatic, jaw_b mimics jaw_a) + tool0` (frame-only) — six revolute joints, the last three concurrent at the
wrist centre (`docs/forearm_roll.md`). The cycloidal drive **is** `shoulder_pitch`:
`LINKS` places its stator (`cycloidal_drive#1:stator` — housing, motor, gear train) in `shoulder_link`
with the yawing `j1_coupler` and its rotor (`cycloidal_drive#1:rotor` — output hub + pins) in
`upper_arm_link` with `j1_link` (`assemblies/cycloidal_drive.py BODIES`, expanded by
`assemblies/_occurrences.world_rows`; see the URDF ledger and `docs/cycloidal_drive.md` §12). Frames
are REP-103 (`base_link` on the base's bottom face at the base_yaw axis, Z up, X forward); every joint
frame has Z on its axis; **all joints are 0 at the SolidWorks capture pose**, so every mesh has an
identity origin and the URDF at zero reproduces `assemblies/arm.py` (which is emitted in the same
`base_link` frame — see Assembly). Limits, effort/velocity and axis
signs are placeholders (`lib/params.py` `*_LIMIT_DEG …`, tagged `[ESTIMATE]`) — confirm with viewer
sweeps and hardware.

| joint | type | parent → child | actuator | notes |
|---|---|---|---|---|
| `base_yaw` | revolute, world up | `base_link → shoulder_link` | NEMA 17 x 48 + MKS SERVO42D (`nema17_48mm#1` + `mks_servo42d#1` under the base plate, hanging `BASE_MOTOR_STACK_PROUD` = 6.1 mm below the base's bottom face; CAN id unconfirmed) | the holder `j1_coupler` turns on the base |
| `shoulder_pitch` | revolute, `N` | `shoulder_link → upper_arm_link` | the 20:1 cycloidal drive, its own NEMA 17 (`CYCLOIDAL_RATIO`) | stator with the holder, rotor with `j1_link` |
| `elbow_pitch` | revolute, `N` | `upper_arm_link → elbow_link` | GT2 90T belt, NEMA 17 x 40 + MKS SERVO42D (`nema17_40mm#2` + `mks_servo42d#2` on `j1_link`'s pad; CAN id unconfirmed) | the pulley carries the roll drive's block, which is the elbow coupler (`j3_coupler#1` retired) |
| `forearm_roll` | revolute, along the forearm through the wrist centre | `elbow_link → forearm_link` | GT2 90T ring on the hollow roll shaft, NEMA 17 x 40 + MKS SERVO42D on the elbow block's plate (`assemblies/forearm_roll_drive.py`; a 4th CAN id) | the shaft's end spigot bolts to `j2_link`'s wall 48 mm from the elbow axis; hard stop ±`FOREARM_ROLL_LIMIT_DEG`; specs `docs/forearm_roll.md` §0 |
| `wrist_pitch` | revolute, `N` | `forearm_link → wrist_pitch_link` | GT2 90T belt, NEMA 17 x 40 + MKS SERVO42D (`nema17_40mm#3` + `mks_servo42d#3` on `j2_link`'s web slots; CAN id unconfirmed) | pulley + J3 coupler on the wrist body |
| `wrist_roll` | revolute, `F` | `wrist_pitch_link → wrist_roll_link` | NEMA17 pancake + 20T pulley (not CAN-driven) | |
| `jaw_a`, `jaw_b` (mimic, −1) | prismatic | `wrist_roll_link → jaw_*_link` | MG996R crank linkage | `open` / `closed` SRDF states |
| `tool0_joint` | fixed | `wrist_roll_link → tool0` | — | fingertip midpoint, Z = approach |

| link | occurrences (`robot/frames.py LINKS`) |
|---|---|
| `base_link` | `base` + `nema17_48mm#1`, `mks_servo42d#1` (the base_yaw 48 mm motor + board under the plate) |
| `shoulder_link` | `j1_coupler` + the drive's **stator** (`cycloidal_drive#1:stator`: motor plate, ring gear body, ring pins, housing bolts/nuts, NEMA 17 + its MKS board, gear train) |
| `upper_arm_link` | the drive's **rotor** (`cycloidal_drive#1:rotor`: output hub, output pins, 625) + `j1_link` + `j1_cap` + `nema17_40mm#2`, `mks_servo42d#2` (the elbow_pitch motor + board on the pad) |
| `elbow_link` | `gt2_pulley_90t#1` + the roll drive's **stator** (`forearm_roll_drive#1:stator`: the elbow block — the elbow coupler and the housing in one, `j3_coupler#1` retired —, 2× 6808, the end cap, its NEMA 17 x 40 + MKS board, the 20T) |
| `forearm_link` | the roll drive's **rotor** (`forearm_roll_drive#1:rotor`: the hollow roll shaft with its 90T ring and end spigot) + `j2_link`, `j2_cap_1`, `j2_cap_2` + `nema17_40mm#3`, `mks_servo42d#3` (the wrist_pitch motor + board on the web) |
| `wrist_pitch_link` | `wrist_link`, `gripper_clamp_bracket`, `nema17_pancake`, `gt2_pulley_90t#2`, `j3_coupler#2` |
| `wrist_roll_link` | `gt2_pulley_20t` + the gripper base (connector, servo holder, servo + horn, cover, rails, crank links) |
| `jaw_a_link` / `jaw_b_link` | slider + two fingers + end, each side |

All limits, efforts, velocities, axis signs and the jaw travel are `[ESTIMATE]` placeholders in
`lib/params.py`; the URDF ledger lists the link-membership assumptions.

```bash
./cadtool python tools/robot/derive.py                 # joint origins + link inertials (m, kg, rad)
./cadtool python tools/robot/derive.py --check robot/arm.urdf robot/arm.sdf   # files vs CAD (tests run this)
./cadtool python tools/robot/export_link_meshes.py           # regenerate meshes after converting a part
./cadtool validate robot/arm.urdf --strict             # also .srdf / .sdf --gz-check never
./cadtool snapshot robot/arm.urdf snapshots/arm_urdf.png --joint-values '{"shoulder_pitch": 45}'   # posed stills
./cadtool viewer                                       # then ?file=robot/arm.urdf: meshes + joint sliders (base_yaw, shoulder_pitch, elbow_pitch, forearm_roll, wrist_pitch, wrist_roll, jaw_a)
```

(cadgen 0.5.0/0.5.1 drew every robot description as a pile of shards and needed a local runtime patch;
fixed upstream in 0.6.0, the patch is gone.) All joints read 0 at
the SolidWorks capture pose and the `shoulder_pitch` slider turns the
drive's rotor with `j1_link` while its housing stays with `j1_coupler`; the `forearm_roll` slider turns the
forearm, wrist and gripper about the forearm while the elbow block stays with the elbow pulley. The drives on
their own: `?file=assemblies/cycloidal_drive.step` (see `docs/cycloidal_drive.md`, "Viewing the drive") and
`?file=assemblies/forearm_roll_drive.step` (`docs/forearm_roll.md`).

After any CAD change that moves geometry: re-export the meshes, re-run the check, and if a
frame moved re-derive the affected `<origin>`/`<inertial>` values with `--urdf-draft` /
`--sdf-draft` (the drafts are scaffolding; the checked-in XML stays canonical).

## Tests

```bash
./cadtool pytest                 # everything (~460 tests; the geometry builds take ~2 min; never writes a STEP)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON, tooling (the pinned cadgen + one complete OCP kernel)
./cadtool pytest tests/cycloidal # the cycloidal drive's tests only (tests/cycloidal/test_<part>.py + helpers.py)
uv run pytest                    # equivalent (cadgen is a normal dependency)
```

## AI CAD assistance

The `cad@text-to-cad` plugin's `/cad:*` skills drive the run-the-model → inspect → snapshot loop
this repo is aligned with (plus `dfam-check` for printability, `step-parts`, and the URDF/SRDF/SDF
skills); they assume the `cadgen` CLI on `PATH` — inside this project that is `./cadtool cadgen …`.
Agent-facing conventions live in `CLAUDE.md`.
