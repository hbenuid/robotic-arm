# robotic-arm — CAD (build123d)

**Last updated:** 2026-09-11 — see the root `CHANGELOG.md` for dated changes.

Parametric CAD-as-code for the 3-joint arm, converted part-by-part from the original
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
STEP/STL (`parts/`, `reference/`, `vendor/`, `robot/meshes/`) is a Git LFS object, so `git lfs install`
once per machine before cloning (a clone that shows ~130-byte pointer files needs `git lfs pull`) — and
the [`cad@text-to-cad`](https://github.com/earthtojake/text-to-cad) Claude Code plugin **v0.5.x**
(the repo's `.claude/settings.json` enables its marketplace; install/update with
`claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git`,
`claude plugin install cad@text-to-cad`, later `claude plugin marketplace update text-to-cad &&
claude plugin update cad@text-to-cad`). The plugin only carries the `/cad:*` skill docs; the toolchain
itself is the [`cadgen`](https://pypi.org/project/cadgen/) package installed into this venv.

```bash
cd cad
./cadtool setup        # uv sync (build123d/OCP/cadgen into ./.venv) + Playwright Chromium (~150 MB, snapshots only)
./cadtool pytest       # everything green?
```

Python is pinned to **3.12** in `.python-version` (the system Python is 3.14; the OCP / ocp-vscode
wheels are verified on 3.12 — bump deliberately, with the full suite). Never run bare `python` here —
always `./cadtool …` or `uv run …` from `cad/`.

`cadgen` (the text-to-cad runtime, on PyPI) is a **locked dependency** pinned to the installed
plugin version (`cadgen[snapshot]==…` in `pyproject.toml`; the plugin's `skills/cad/requirements.txt`
pins the same) — bump both together; `./cadtool doctor` checks the pair, Node and Chromium.
cadgen 0.5 has **no compatibility with 0.4** (see the 2026-09-11 CHANGELOG entry for the migration).

## `./cadtool` — the one entry point

| Command | What it does |
|---|---|
| `./cadtool gen parts/<group>/<name>.py` (alias `step`) | **run the model script**: writes the committed `parts/<group>/<name>.step`; a second run prints `current …` (freshness gate), `--force` rebuilds |
| `./cadtool gen assemblies/arm.py` | build `assemblies/arm.step` (git-ignored) — calls every part model, so each stale part is rebuilt (in parallel) and its committed STEP rewritten |
| `./cadtool why <model.py>` | why the model is current or stale, clause by clause (`cadgen store why`) |
| `./cadtool show <model.py>` | preview the model body in the OCP CAD Viewer VS Code extension (`ocp_vscode`) — no build, nothing written |
| `./cadtool export <file.step> stl\|3mf\|glb [out]` | one mesh file per call from a STEP document (Node 20+; `--mesh-tolerance` is *relative*, default 1.5e-3 of the bounding diagonal) |
| `./cadtool inspect refs <file.step> --facts --planes --positioning` | geometry facts, selector refs, planes; also `measure`, `align`, `frame`, `diff`, `interfere`, `validate` |
| `./cadtool snapshot assemblies/arm.step snapshots/arm.png --size-profile assembly --view-labels` | PNG review still (`--job job.json` for a multi-view packet; the path you name is the file written) — also `.urdf`/`.sdf`/`.stl` inputs |
| `./cadtool viewer [--port N]` | CAD Viewer serving this folder — `http://127.0.0.1:3245/?file=assemblies/arm.step` (ships inside cadgen; stops after 12 h or Ctrl+C; `./cadtool cadgen viewer list\|stop --port N`) |
| `./cadtool validate robot/arm.urdf --strict` (`.srdf`, `.sdf --gz-check never`) | robot-description validators |
| `./cadtool parts "<query>" [--download --id <id> --filename <name>.step]` | step.parts search / download into `vendor/` |
| `./cadtool skill <skill> <tool> [args]` | a plugin skill script (`dfam-check dfam_tool.py`, `gcode gcode_tool.py`, …) |
| `./cadtool cadgen …` / `store …` / `daemon …` | any `cadgen` subcommand (`store info\|gc`, `daemon status`, `doctor`); `./cadtool daemon stop` ends the warm build daemon + workers (cadgen has no stop verb) |
| `./cadtool doctor` | installed cadgen vs the plugin's pin, Node, Playwright Chromium |
| `./cadtool pytest [-m "not slow"]` | test suite (the fast lane skips geometry builds) |
| `./cadtool python …` | any python in the venv with `PYTHONPATH=cad/` (`-c "from assemblies.cycloidal_drive import totals; print(totals())"`) |
| `./cadtool clean [--all]` | delete `__pycache__/`, `.pytest_cache/`; `--all` also empties `snapshots/`, `exports/` and removes the git-ignored `assemblies/*.step`, `robot/links/*.step` |

`cadtool` always `cd`s to `cad/` (cadgen resolves paths from the working directory and the viewer
serves it) and exports `PYTHONPATH=cad/` so `lib`, `parts`, `assemblies`, `robot` import the same way
for you, for cadgen's dependency scan and for its warm build daemon. Everything derived — trees,
tessellations, the freshness records — lives in `~/.cache/cadgen` (`./cadtool store gc` sweeps it;
deleting it is always safe). `CADGEN_DAEMON=0` runs a build on transient workers instead of the daemon.

This project runs cadgen 0.5.1 on **build123d 0.11.1 / OCP 7.9.3** — the pair cadgen is developed
against. build123d pulls `cadquery-ocp-novtk`, cadgen pulls `cadquery-ocp` unconstrained, so
`pyproject.toml` pins both to the same release (two OCP builds in one venv otherwise). On the previous
pair (build123d 0.10.0 / OCP 7.8.1) cadgen 0.5.1 could not build 11 of the 41 parts (OCCT 7.8.1
mis-read its BinTools VERSION_4 component objects), could not export a linked child (`wrapped` is a
property only since build123d 0.11) and its STEP writer needs OCP 7.9's `HArray1.Value`.

## Layout

```
cad/
├── cadtool                # bash wrapper around the cadgen CLI + uv (see above)
├── .env                   # PYTHONPATH=. for the VS Code Python extension (cadtool/pytest set it themselves)
├── lib/
│   ├── params.py          # single source of truth for shared dimensions (tagged provenance)
│   ├── reference.py       # naming maps (SolidWorks custom/COTS, designed cycloidal parts, modules), loaders, path_of(), matches_reference()
│   ├── placements.py      # reference/placements.json -> build123d Location
│   ├── models.py          # model_of() / raw() / geometry(inline=): the @step model of a module, its body, a child for an assembly
│   ├── assembly.py        # AssemblyHelper / label_shape / label_text re-exported from cadgen
│   ├── export.py          # build123d STL/STEP export into exports/ (ad-hoc sidecars)
│   └── cycloidal/         # the cycloidal drive: DriveConfig (params.py), layout.py, profiles.py, housing.py, disc.py, geom.py
├── parts/                 # one part per file, grouped by subsystem; parts.names() / parts.load(name) discover them
│   ├── __init__.py            # the directory scan: MODULES / GROUPS, names(), load(), model(), build(), source_of()
│   ├── _templates/            # designed.py (parametric), wrapper.py (import wrapper), cots.py (purchased) templates
│   ├── base/                  # base, j1_coupler, j1_link, j1_cap                                  (SolidWorks wrappers)
│   ├── joints/                # j2_link, j2_cap_1, j2_cap_2, j3_coupler, gt2_pulley_90t            (SolidWorks wrappers)
│   ├── wrist/                 # wrist_link, gripper_clamp_bracket, gripper_j3_connector + COTS nema17_pancake, gt2_pulley_20t
│   ├── gripper/               # gripper_* (8), servo_holder + COTS gripper_rail_6mm, mg996r_servo, mg996r_horn
│   └── cycloidal/             # the drive: 6 designed parts + 10 COTS (bearings, nema17_48mm, pins, bolts, nuts), _cots.py helper
│       └── <name>.py + <name>.step   # every group: running the .py writes the .step beside it; committed (Git LFS)
├── assemblies/
│   ├── arm.py             # the whole arm, grouped arm -> base_link/link1/link2/link3/wrist (GROUPS; 52 leaves, tinted per group)
│   ├── gripper.py         # the gripper mechanism module (19 occurrences, placed from placements.json)
│   ├── cycloidal_drive.py # the drive module (18 rows placed from lib/cycloidal stack_positions - code-driven)
│   └── _occurrences.py    # place()/add_occurrences()/add_grouped_occurrences() (placement keys), place_at()/add_located() (Locations), world_rows() - children via lib.models.geometry()
├── docs/cycloidal_drive.md  # the drive's spec, port notes and attachment
├── reference/             # immutable per-part reference STEPs (Git LFS) + manifest.json + placements.json + README
│   ├── solidworks/            # the 25 SolidWorks exports (custom parts + the SolidWorks purchased parts)
│   └── cycloidal/             # the 16 CadQuery exports the drive was ported from
├── vendor/                # purchased-part STEPs (committed via Git LFS; replaceable by better catalog models)
├── robot/                 # URDF / SRDF / SDF + per-link meshes and generators (see below)
├── tools/                 # preview.py (./cadtool show), reference/{import_reference,extract_placements}.py (SolidWorks),
│                          # cycloidal/{export_cadquery,import_reference}.py (the drive's references), robot/{frames,export_link_meshes}.py
├── tests/                 # pytest: conventions, reference match, placements, assembly totals, params locks, robot description;
│   ├── conftest.py            # CADGEN_DAEMON=0 + a guard that fails any test calling a model at top level (tests call bodies)
│   └── cycloidal/             # the drive's own tests (one module per part + housing / purchased / fitment / assembly / port) + helpers.py
├── exports/               # STL/3MF sidecars (git-ignored)
└── snapshots/             # snapshot PNGs (git-ignored)
```
Nothing derived lands in the tree: cadgen keeps trees, tessellations and freshness records in its
content-addressed store (`~/.cache/cadgen`). A `<name>.step.json` sidecar appears beside a STEP only
when its model declares `kinematics=` (none does yet); it would be committed with the STEP.

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
  effects** and previews go through `./cadtool show`. Imports are plain (`from lib import …`): `cadtool`,
  pytest and `.env` put `cad/` on the import path, and cadgen imports the file as `parts.<group>.<name>`;
- **calling the model outside a build runs the pipeline** (writes the STEP, uses the daemon) — tests and
  tools call the body instead (`parts.build(name)` / `lib.models.raw(model)`), assemblies call
  `lib.models.geometry(model)` (linked child inside a build, body otherwise);
- the result is labelled with the part name; shared dimensions come from `lib/params.py`;
- **custom parts** declare `REFERENCE` (their `reference/solidworks/<name>.step`), `CONVERTED`
  (`False` while the file is a wrapper) and `LOCAL_FROM_REF` (rigid transform from the
  SolidWorks part-file frame to the part's local frame; identity for wrappers);
- **designed parts** (the cycloidal drive's printed parts, `lib/reference.py DESIGNED`) are the
  same contract with `CONVERTED = True` and a reference that is the CadQuery export they were
  ported from — the strict `tests/cycloidal/test_port.py` checks them beyond the reference match;
- **COTS parts** declare `COTS = True`, `MASS_G`, `VENDOR_STEP` and `VENDOR_TO_REF`, and the model
  returns the vendor STEP (`cadgen.read_step` — a tracked input, so swapping the file makes the part
  stale — re-oriented by `VENDOR_TO_REF`) when present, else a parametric `_envelope()`.

`tests/test_parts_convention.py` enforces all of this automatically for every discovered part.
(cadgen's default output is the sibling `<name>.step` and the viewer pairs the two — which is why a
part and its STEP always live in the same directory; upstream's `src/` + `STEP/` layout is not used.)

## Converting a part (wrapper → parametric)

1. Inspect the reference: `./cadtool inspect refs reference/solidworks/<name>.step --facts --planes --positioning`
   (and `measure` for the dimensions you need); `parts/<group>/<name>.py`'s docstring lists units,
   solids, bounding box and where it is used.
2. Rewrite the model body with `BuildPart`/… code. Put every shared dimension in `lib/params.py`
   with a provenance tag (`[MEASURE]`, `[DATASHEET]`, `[DESIGN]`, `[REFERENCE]`, `[ESTIMATE]`)
   and a lock in `tests/test_params_invariants.py`.
3. Set `CONVERTED = True`. If you pick a nicer local origin than the SolidWorks one, set
   `LOCAL_FROM_REF` to the transform *reference frame → new local frame*; the assemblies compose
   `placement * LOCAL_FROM_REF⁻¹`, so `reference/placements.json` never changes.
   (`j1_cap` and `j2_cap_2` have their geometry ~1 m from the SolidWorks origin — they are the
   ones that want this.)
4. `./cadtool pytest tests/test_reference_match.py -k <name>` — volume within 0.5 % and bounding
   box within 0.2 mm of the reference (per-part overrides: `REF_VOL_TOL`, `REF_BBOX_TOL`).
5. `./cadtool gen parts/<group>/<name>.py` to regenerate the committed STEP, then
   `./cadtool gen assemblies/arm.py` + `./cadtool snapshot …` to eyeball it in place.

## Purchased parts and step.parts

Each COTS part keeps its SolidWorks re-export in `reference/solidworks/<name>.step` (the frame
and size reference; the drive's purchased parts keep their CadQuery export in
`reference/cycloidal/`) and its current best model in `vendor/<name>.step`. To try a catalog model:
`./cadtool parts "<query>"` → pick an id → `./cadtool parts --id <id> --download --filename
<name>.step --overwrite` → `./cadtool inspect refs vendor/<name>.step --facts --planes --positioning`
→ set `VENDOR_TO_REF` in `parts/<group>/<name>.py` so the model lands in the SolidWorks frame →
`./cadtool pytest -k <name>` (`test_cots_vendor_matches_reference_frame`: bbox within 1.5 mm of
the reference) → `./cadtool gen parts/<group>/<name>.py` → `./cadtool python tools/reference/import_reference.py`
(updates `manifest.json`; `tools/cycloidal/import_reference.py` for the drive's parts). If the
catalog model is worse, restore the reference copy. See `vendor/README.md` for what has been tried.

## Assembly

`assemblies/arm.py` and `assemblies/gripper.py` hold `OCCURRENCES` tables —
`(part, role, placement_key)` in SolidWorks document order — and place each part model with
`lib.placements` (`"j3_coupler#2"` = second occurrence of that part; inside a cadgen build the child
model is called — built in parallel, its STEP rewritten when stale — and the gripper links its tree
while the tinted arm keeps an inline copy; outside a build it is a fresh in-process copy). Roles make
duplicate parts' labels unique (`j3_coupler:j2`, `gripper_end:1`); they are positional for
now. `assemblies/cycloidal_drive.py` is a **code-driven module**: its rows are
`(part, role, Location)` computed from `lib/cycloidal` (`stack_positions`), and `arm.py` locates
the whole module at the SolidWorks node's pose (`placements.json` `cycloidal_drive#1`, a
`designed` module record). `arm.py GROUPS` buckets the occurrences into the component tree
`arm -> base_link/link1/link2/link3/wrist` — the rigid-link partition of `robot/frames.py LINKS`
with the gripper module kept whole — so each component toggles as one node in the viewers, and
every subtree is tinted with its group's color (the gripper and cycloidal_drive modules keep
their own). `tests/test_assembly.py` checks the rebuilt arm against the SolidWorks totals plus
the module's own lock (34 + 18 leaves, 50 + 58 solids, volumes, bbox), the group labels and the
`LINKS` mirror.

## Reference geometry and placements

`reference/solidworks/*.step` are renamed copies of the SolidWorks exports in
`~/Documents/arm_assembly_organized/` — see `reference/README.md` for the naming map;
`reference/cycloidal/*.step` are the CadQuery exports the drive was ported from
(`lib.reference.path_of(name)` resolves the origin, `manifest.json` records it in `file`).
`reference/placements.json` holds every occurrence's placement extracted from the full-assembly
STEP. Both are **immutable inputs** (a checksum test guards them); regenerate with
`./cadtool python tools/reference/import_reference.py` and `./cadtool python tools/reference/extract_placements.py`
if the SolidWorks design changes. The cycloidal drive's 16 references are the CadQuery exports of
the `cycloidal_drive` repo at `2f1f67d` (`tools/cycloidal/export_cadquery.py` in that repo's venv,
then `./cadtool python tools/cycloidal/import_reference.py`), and its SolidWorks node is recorded
in `placements.json` as a designed module (pose only; the contents come from code).

## Robot description (URDF / SRDF / SDF)

`robot/` holds the arm's robot description, derived from the CAD:

```
robot/
├── frames.py          # THE kinematic decomposition: LINKS (which occurrences move together) + JOINTS
│                      # (axis point/direction, parent/child, limits from lib/params.py)
├── links/<link>.py    # a @step model per rigid link, in the link's own frame (./cadtool gen robot/links/link1.py)
├── meshes/<link>.stl  # per-link meshes in mm (committed) - tools/robot/export_link_meshes.py
├── arm.urdf           # SOURCE OF TRUTH (hand-edited ledger + numbers from tools/robot/frames.py)
├── arm.srdf           # MoveIt2 semantics: chain base_link->tool0, gripper group, home/open/closed states
└── arm.sdf            # model-level SDF 1.12 derived from the URDF
```

Links: `base_link → j1 → link1 → j2 → link2 → j3 → link3 → wrist_roll → wrist_roll_link → jaw_a / jaw_b
(prismatic, jaw_b mimics jaw_a) + tool0` (frame-only). `link1` carries the whole cycloidal drive
(module key `cycloidal_drive#1` in `LINKS`, expanded by `assemblies/_occurrences.world_rows`) —
physically the drive is a shoulder-pitch joint between `j1_coupler` and `j1_link` that is **not modelled
as a joint yet** (see the URDF ledger and `docs/cycloidal_drive.md` §12). Frames are REP-103 (`base_link` on the base's
bottom face at the J1 axis, Z up, X forward); every joint frame has Z on its axis; **all joints are 0
at the SolidWorks capture pose**, so every mesh has an identity origin and the URDF at zero
reproduces `assemblies/arm.py`. Limits, effort/velocity and axis signs are placeholders
(`lib/params.py` `J*_LIMIT_DEG …`, tagged `[ESTIMATE]`) — confirm with viewer sweeps and hardware.

```bash
./cadtool python tools/robot/frames.py                 # joint origins + link inertials (m, kg, rad)
./cadtool python tools/robot/frames.py --check robot/arm.urdf robot/arm.sdf   # files vs CAD (tests run this)
./cadtool python tools/robot/export_link_meshes.py           # regenerate meshes after converting a part
./cadtool validate robot/arm.urdf --strict             # also .srdf / .sdf --gz-check never
./cadtool snapshot robot/arm.urdf snapshots/arm_urdf.png --joint-values '{"j2": 45}'   # posed stills
./cadtool viewer                                       # then ?file=robot/arm.urdf: meshes + joint sliders (j1, j2, j3, wrist_roll, jaw_a)
```

In the viewer all joints read 0 at the SolidWorks capture pose; the cycloidal drive moves with
`link1` (there is no shoulder-pitch slider until that joint is modelled). The drive on its own:
`?file=assemblies/cycloidal_drive.step` (see `docs/cycloidal_drive.md`, "Viewing the drive").

After any CAD change that moves geometry: re-export the meshes, re-run the check, and if a
frame moved re-derive the affected `<origin>`/`<inertial>` values with `--urdf-draft` /
`--sdf-draft` (the drafts are scaffolding; the checked-in XML stays canonical).

## Tests

```bash
./cadtool pytest                 # everything (~420 tests; the geometry builds take ~2 min; never writes a STEP)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON
./cadtool pytest tests/cycloidal # the cycloidal drive's tests only (tests/cycloidal/test_<part>.py + helpers.py)
uv run pytest                    # equivalent (cadgen is a normal dependency)
```

## AI CAD assistance

The `cad@text-to-cad` plugin's `/cad:*` skills drive the run-the-model → inspect → snapshot loop
this repo is aligned with (plus `dfam-check` for printability, `step-parts`, and the URDF/SRDF/SDF
skills); they assume the `cadgen` CLI on `PATH` — inside this project that is `./cadtool cadgen …`.
Agent-facing conventions live in `CLAUDE.md`.
