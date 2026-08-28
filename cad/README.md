# robotic-arm — CAD (build123d)

**Last updated:** 2026-08-28 — see the root `CHANGELOG.md` for dated changes.

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

Requirements: [`uv`](https://docs.astral.sh/uv/), Node 18+ (the CAD Viewer launcher), `git-lfs`
on `PATH` — the plugin marketplace uses LFS **and so does this folder**: every committed STEP/STL
(`parts/`, `reference/`, `vendor/`, `robot/meshes/`) is a Git LFS object, so `git lfs install` once
per machine before cloning (a clone that shows ~130-byte pointer files needs `git lfs pull`) — and the
[`cad@text-to-cad`](https://github.com/earthtojake/text-to-cad) Claude Code plugin **v0.4.x**
(the repo's `.claude/settings.json` enables its marketplace; install/update with
`claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git`,
`claude plugin install cad@text-to-cad`, later `claude plugin marketplace update text-to-cad &&
claude plugin update cad@text-to-cad`).

```bash
cd cad
./cadtool setup        # uv sync (build123d/OCP/cadgen into ./.venv) + Playwright Chromium (~150 MB, snapshots only)
./cadtool pytest       # everything green?
```

Python is pinned to **3.12** in `.python-version`: build123d caps at `<3.14` and the `vtk`
wheel it pulls stops at cp312, while the system Python is 3.14. Don't bump it. Never run
bare `python` here — always `./cadtool …` or `uv run …` from `cad/`.

`cadgen` (the plugin's Python runtime, on PyPI) is a **locked dependency** pinned to the
installed plugin version — bump `cadgen==…` in `pyproject.toml` together with the plugin.

## `./cadtool` — the one entry point

| Command | What it does |
|---|---|
| `./cadtool gen parts/<group>/<name>.py` (alias `step`) | build the part and write the committed `parts/<group>/<name>.step` (+ the `parts/<group>/__cadgen__/` viewer package); `--force` rebuilds |
| `./cadtool gen assemblies/arm.py` | build `assemblies/arm.step` (git-ignored, regenerable) |
| `./cadtool export parts/<group>/<name>.py --stl [path]` | STL/3MF/GLB sidecars; a bare `--stl` writes `parts/<group>/<name>.stl`, a path resolves **beside the source** (`--stl ../../exports/<name>.stl` for `cad/exports/`) |
| `./cadtool inspect refs <file.step> --facts --planes --positioning` | geometry facts, selector refs, planes; also `measure`, `align`, `frame`, `diff`, `interfere` |
| `./cadtool snapshot --input assemblies/arm.step --output snapshots/arm.png --size-profile assembly --view-labels` | PNG review packet (`--job -` takes a JSON job on stdin; timestamp appended) |
| `./cadtool viewer [--port N]` | CAD Viewer on this folder — `http://127.0.0.1:3245/<abs cad>?file=assemblies/arm.step` (Python backend in our venv; stops after 12 h or Ctrl+C) |
| `./cadtool validate robot/arm.urdf` (`.srdf`, `.sdf`) | robot-description validators |
| `./cadtool parts "<query>" [--download --id <id> --filename <name>.step]` | step.parts search / download into `vendor/` |
| `./cadtool skill <skill> <tool> [args]` | any plugin skill CLI (`urdf snapshot`, `dfam-check dfam_tool.py`, …) |
| `./cadtool pytest [-m "not slow"]` | test suite (the fast lane skips geometry builds) |
| `./cadtool python -m assemblies.arm` | preview in the OCP CAD Viewer VS Code extension (`ocp_vscode`) |
| `./cadtool clean [--all]` | delete the regenerable caches (`__cadgen__/` viewer packages — hundreds of MB —, `__pycache__/`, `.pytest_cache/`); `--all` also empties `snapshots/`, `exports/` and removes the git-ignored `assemblies/*.step`, `robot/links/*.step` |

`cadtool` always `cd`s to `cad/` (the plugin resolves paths from the working directory and the
viewer serves it). If the venv ever lacks `cadgen`, it falls back to the plugin's vendored copy
via `PYTHONPATH`. `CADGEN_WARM=1` uses the plugin's warm daemon for faster repeated builds.

## Layout

```
cad/
├── cadtool                # bash wrapper around the plugin CLIs + uv (see above; `clean` drops the caches)
├── lib/
│   ├── params.py          # single source of truth for shared dimensions (tagged provenance)
│   ├── reference.py       # naming maps (SolidWorks custom/COTS, designed cycloidal parts, modules), loaders, path_of(), matches_reference()
│   ├── placements.py      # reference/placements.json -> build123d Location
│   ├── assembly.py        # AssemblyHelper (cadgen, tiny fallback otherwise)
│   ├── export.py          # plugin-free STL/STEP export into exports/
│   └── cycloidal/         # the cycloidal drive: DriveConfig (params.py), layout.py, profiles.py, housing.py, disc.py, geom.py
├── parts/                 # one part per file, grouped by subsystem; parts.names() / parts.load(name) discover them
│   ├── __init__.py            # the directory scan: MODULES / GROUPS, names(), load(), source_of()
│   ├── _templates/            # designed.py (parametric), wrapper.py (import wrapper), cots.py (purchased) templates
│   ├── base/                  # base, j1_coupler, j1_link, j1_cap                                  (SolidWorks wrappers)
│   ├── joints/                # j2_link, j2_cap_1, j2_cap_2, j3_coupler, gt2_pulley_90t            (SolidWorks wrappers)
│   ├── wrist/                 # wrist_link, gripper_clamp_bracket, gripper_j3_connector + COTS nema17_pancake, gt2_pulley_20t
│   ├── gripper/               # gripper_* (8), servo_holder + COTS gripper_rail_6mm, mg996r_servo, mg996r_horn
│   └── cycloidal/             # the drive: 6 designed parts + 10 COTS (bearings, nema17_48mm, pins, bolts, nuts), _cots.py helper
│       └── <name>.py + <name>.step   # every group: the .step is generated beside its source and committed (Git LFS)
├── assemblies/
│   ├── arm.py             # the whole arm (16 top-level occurrences + the gripper and cycloidal_drive modules = 52 leaves)
│   ├── gripper.py         # the gripper mechanism module (19 occurrences, placed from placements.json)
│   ├── cycloidal_drive.py # the drive module (18 rows placed from lib/cycloidal stack_positions - code-driven)
│   └── _occurrences.py    # place()/add_occurrences() (placement keys), place_at()/add_located() (Locations), world_rows()
├── docs/cycloidal_drive.md  # the drive's spec, port notes and attachment
├── reference/             # immutable per-part reference STEPs (Git LFS) + manifest.json + placements.json + README
│   ├── solidworks/            # the 25 SolidWorks exports (custom parts + the SolidWorks purchased parts)
│   └── cycloidal/             # the 16 CadQuery exports the drive was ported from
├── vendor/                # purchased-part STEPs (committed via Git LFS; replaceable by better catalog models)
├── robot/                 # URDF / SRDF / SDF + per-link meshes and generators (see below)
├── tools/                 # reference/{import_reference,extract_placements}.py (SolidWorks), cycloidal/{export_cadquery,
│                          # import_reference}.py (the drive's references), robot/{frames,export_link_meshes}.py
├── tests/                 # pytest: conventions, reference match, placements, assembly totals, params locks, robot description;
│   └── cycloidal/             # the drive's own tests (one module per part + housing / purchased / fitment / assembly / port) + helpers.py
├── exports/               # STL/3MF sidecars (git-ignored)
└── snapshots/             # snapshot PNGs (git-ignored)
```
`__cadgen__/` directories (viewer packages, STEP import caches) appear next to entries and
imported STEPs; they are git-ignored and regenerable (`./cadtool clean`).

## Part conventions

Each `parts/<group>/<name>.py` follows the plugin-native **`gen_step()`** convention (the group
directory is only a folder — the part's *name* is the module stem, unique across groups, and it
keys `reference/manifest.json`, `reference/<origin>/<name>.step`, `placements.json` and the URDF
links; code never imports a part statically but goes through `parts.load(name)` / `parts.names()`,
which scan the group packages):

- a module-level `gen_step()` **returns** the final `Part`/`Compound` at the part's **local
  origin** — the assembly owns placement; importing the module has **no side effects**
  (`show()` only under `if __name__ == "__main__":`); the 2-line path shim at the top
  (`parents[2]` — two levels below `cad/`) makes `from lib …` resolve under Ctrl+F5, `python -m`,
  and the plugin CLI;
- the result is labelled with the part name; shared dimensions come from `lib/params.py`;
- **custom parts** declare `REFERENCE` (their `reference/solidworks/<name>.step`), `CONVERTED`
  (`False` while the file is a wrapper) and `LOCAL_FROM_REF` (rigid transform from the
  SolidWorks part-file frame to the part's local frame; identity for wrappers);
- **designed parts** (the cycloidal drive's printed parts, `lib/reference.py DESIGNED`) are the
  same contract with `CONVERTED = True` and a reference that is the CadQuery export they were
  ported from — the strict `tests/cycloidal/test_port.py` checks them beyond the reference match;
- **COTS parts** declare `COTS = True`, `MASS_G`, `VENDOR_STEP` and `VENDOR_TO_REF`, and
  `gen_step()` returns the vendor STEP (re-oriented by `VENDOR_TO_REF`) when present, else a
  parametric `_envelope()`.

`tests/test_parts_convention.py` enforces all of this automatically for every discovered part.
(The viewer catalog scans the folder recursively and shows a `.py` generator together with its
sibling `.step` as one entry — which is why a part and its STEP always live in the same directory.)

## Converting a part (wrapper → parametric)

1. Inspect the reference: `./cadtool inspect refs reference/solidworks/<name>.step --facts --planes --positioning`
   (and `measure` for the dimensions you need); `parts/<group>/<name>.py`'s docstring lists units,
   solids, bounding box and where it is used.
2. Rewrite `gen_step()` with `BuildPart`/… code. Put every shared dimension in `lib/params.py`
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
`(part, role, placement_key)` in SolidWorks document order — and place a fresh copy of each
part with `lib.placements` (`"j3_coupler#2"` = second occurrence of that part). Roles make
duplicate parts' labels unique (`j3_coupler:j2`, `gripper_end:1`); they are positional for
now. `assemblies/cycloidal_drive.py` is a **code-driven module**: its rows are
`(part, role, Location)` computed from `lib/cycloidal` (`stack_positions`), and `arm.py` locates
the whole module at the SolidWorks node's pose (`placements.json` `cycloidal_drive#1`, a
`designed` module record). `tests/test_assembly.py` checks the rebuilt arm against the
SolidWorks totals plus the module's own lock (34 + 18 leaves, 50 + 58 solids, volumes, bbox).

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
├── links/<link>.py    # gen_step() per rigid link, in the link's own frame (./cadtool gen robot/links/link1.py)
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
./cadtool validate robot/arm.urdf --strict             # also .srdf / .sdf
./cadtool skill urdf snapshot --input robot/arm.urdf --output snapshots/arm_urdf.png   # posed stills (--help lists the joint options)
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
./cadtool pytest                 # everything (~420 tests; the geometry builds take ~2 min)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON
./cadtool pytest tests/cycloidal # the cycloidal drive's tests only (tests/cycloidal/test_<part>.py + helpers.py)
uv run pytest                    # equivalent (cadgen is a normal dependency)
```

## AI CAD assistance

The `cad@text-to-cad` plugin's `/cad:*` skills drive the generate → inspect → snapshot loop
this repo is aligned with (plus `dfam-check` for printability, `step-parts`, and the URDF/SRDF/SDF
skills). Agent-facing conventions live in `CLAUDE.md`.
