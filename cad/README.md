# robotic-arm — CAD (build123d)

**Last updated:** 2026-08-28 — see the root `CHANGELOG.md` for dated changes.

Parametric CAD-as-code for the 3-joint arm, converted part-by-part from the original
SolidWorks design. This folder is a **separate uv project** (Python 3.12) — the motor-control
software in the repo root never depends on it.

**Status:** every SolidWorks custom part exists as an *import wrapper* around its reference
geometry (`reference/<name>.step`), purchased parts use their vendor STEPs, and
`assemblies/arm.py` places all of them from placements extracted from the SolidWorks
assembly — so the whole arm already assembles, renders and is tested. Converting a part means
replacing its wrapper body with real build123d code (see "Converting a part"). The **20:1
cycloidal shoulder drive is fully parametric build123d** (`lib/cycloidal/`, 16 parts,
`assemblies/cycloidal_drive.py`), ported from the `cycloidal_drive` repo and verified against its
CadQuery exports — see [`docs/cycloidal_drive.md`](docs/cycloidal_drive.md).

## Setup (once per machine)

Requirements: [`uv`](https://docs.astral.sh/uv/), Node 18+ (the CAD Viewer launcher), `git-lfs`
on `PATH` (the plugin marketplace uses LFS), and the
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
| `./cadtool gen parts/<name>.py` (alias `step`) | build the part and write the committed `parts/<name>.step` (+ the `parts/__cadgen__/` viewer package); `--force` rebuilds |
| `./cadtool gen assemblies/arm.py` | build `assemblies/arm.step` (git-ignored, regenerable) |
| `./cadtool export parts/<name>.py --stl [path]` | STL/3MF/GLB sidecars; a bare `--stl` writes `parts/<name>.stl`, a path resolves **beside the source** (`--stl ../exports/<name>.stl` for `cad/exports/`) |
| `./cadtool inspect refs <file.step> --facts --planes --positioning` | geometry facts, selector refs, planes; also `measure`, `align`, `frame`, `diff`, `interfere` |
| `./cadtool snapshot --input assemblies/arm.step --output snapshots/arm.png --size-profile assembly --view-labels` | PNG review packet (`--job -` takes a JSON job on stdin; timestamp appended) |
| `./cadtool viewer [--port N]` | CAD Viewer on this folder — `http://127.0.0.1:3245/<abs cad>?file=assemblies/arm.step` (Python backend in our venv; stops after 12 h or Ctrl+C) |
| `./cadtool validate robot/arm.urdf` (`.srdf`, `.sdf`) | robot-description validators |
| `./cadtool parts "<query>" [--download --id <id> --filename <name>.step]` | step.parts search / download into `vendor/` |
| `./cadtool skill <skill> <tool> [args]` | any plugin skill CLI (`urdf snapshot`, `dfam-check dfam_tool.py`, …) |
| `./cadtool pytest [-m "not slow"]` | test suite (the fast lane skips geometry builds) |
| `./cadtool python -m assemblies.arm` | preview in the OCP CAD Viewer VS Code extension (`ocp_vscode`) |

`cadtool` always `cd`s to `cad/` (the plugin resolves paths from the working directory and the
viewer serves it). If the venv ever lacks `cadgen`, it falls back to the plugin's vendored copy
via `PYTHONPATH`. `CADGEN_WARM=1` uses the plugin's warm daemon for faster repeated builds.

## Layout

```
cad/
├── cadtool                # bash wrapper around the plugin CLIs + uv (see above)
├── lib/
│   ├── params.py          # single source of truth for shared dimensions (tagged provenance)
│   ├── reference.py       # naming maps (SolidWorks custom/COTS, designed cycloidal parts, modules), loaders, matches_reference()
│   ├── placements.py      # reference/placements.json -> build123d Location
│   ├── assembly.py        # AssemblyHelper (cadgen, tiny fallback otherwise)
│   ├── export.py          # plugin-free STL/STEP export into exports/
│   └── cycloidal/         # the cycloidal drive: DriveConfig (params.py), layout.py, profiles.py, housing.py, disc.py, geom.py
├── parts/                 # one part per file, gen_step() returns it at its LOCAL origin
│   ├── _template.py           # designed (parametric) part template
│   ├── _wrapper_template.py   # import-wrapper template (day-one state of every SolidWorks custom part)
│   ├── _cots_template.py      # purchased part template (vendor STEP else envelope)
│   ├── _cycloidal_cots.py     # shared body of the drive's purchased-part modules
│   └── <name>.py + <name>.step   # 20 custom (wrappers) + 6 designed (cycloidal_*) + 15 COTS parts; the .step is generated and committed
├── assemblies/
│   ├── arm.py             # the whole arm (16 top-level occurrences + the gripper and cycloidal_drive modules = 52 leaves)
│   ├── gripper.py         # the gripper mechanism module (19 occurrences, placed from placements.json)
│   ├── cycloidal_drive.py # the drive module (18 rows placed from lib/cycloidal stack_positions - code-driven)
│   └── _occurrences.py    # place()/add_occurrences() (placement keys), place_at()/add_located() (Locations), world_rows()
├── docs/cycloidal_drive.md  # the drive's spec, port notes and attachment
├── reference/             # per-part reference STEPs (SolidWorks exports; CadQuery exports for the drive), manifest, placements
├── vendor/                # purchased-part STEPs (committed; replaceable by better catalog models)
├── robot/                 # URDF / SRDF / SDF + per-link meshes and generators (see below)
├── tools/                 # import_reference.py, extract_placements.py, export_link_meshes.py, robot_frames.py,
│                          # export_cycloidal_cadquery.py (runs in the old CadQuery venv), import_cycloidal_reference.py
├── tests/                 # pytest: conventions, reference match, placements, assembly totals, params locks, robot description,
│                          # test_cycloidal_*.py (the drive's own 200+ tests)
├── exports/               # STL/3MF sidecars (git-ignored)
└── snapshots/             # snapshot PNGs (git-ignored)
```
`__cadgen__/` directories (viewer packages, STEP import caches) appear next to entries and
imported STEPs; they are git-ignored and regenerable.

## Part conventions

Each `parts/<name>.py` follows the plugin-native **`gen_step()`** convention:

- a module-level `gen_step()` **returns** the final `Part`/`Compound` at the part's **local
  origin** — the assembly owns placement; importing the module has **no side effects**
  (`show()` only under `if __name__ == "__main__":`); the 2-line path shim at the top makes
  `from lib …` resolve under Ctrl+F5, `python -m`, and the plugin CLI;
- the result is labelled with the part name; shared dimensions come from `lib/params.py`;
- **custom parts** declare `REFERENCE` (their `reference/<name>.step`), `CONVERTED`
  (`False` while the file is a wrapper) and `LOCAL_FROM_REF` (rigid transform from the
  SolidWorks part-file frame to the part's local frame; identity for wrappers);
- **designed parts** (the cycloidal drive's printed parts, `lib/reference.py DESIGNED`) are the
  same contract with `CONVERTED = True` and a reference that is the CadQuery export they were
  ported from — the strict `tests/test_cycloidal_port.py` checks them beyond the reference match;
- **COTS parts** declare `COTS = True`, `MASS_G`, `VENDOR_STEP` and `VENDOR_TO_REF`, and
  `gen_step()` returns the vendor STEP (re-oriented by `VENDOR_TO_REF`) when present, else a
  parametric `_envelope()`.

`tests/test_parts_convention.py` enforces all of this automatically for every file in `parts/`.
(The plugin's viewer catalog lists generator *entries* named `<name>.step.py`; our importable
`parts/<name>.py` modules still build with `gen`, and the committed `parts/<name>.step` files
show up in the viewer as imported STEPs.)

## Converting a part (wrapper → parametric)

1. Inspect the reference: `./cadtool inspect refs reference/<name>.step --facts --planes --positioning`
   (and `measure` for the dimensions you need); `parts/<name>.py`'s docstring lists units,
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
5. `./cadtool gen parts/<name>.py` to regenerate the committed STEP, then
   `./cadtool gen assemblies/arm.py` + `./cadtool snapshot …` to eyeball it in place.

## Purchased parts and step.parts

Each COTS part keeps its SolidWorks re-export in `reference/<name>.step` (the frame and size
reference) and its current best model in `vendor/<name>.step`. To try a catalog model:
`./cadtool parts "<query>"` → pick an id → `./cadtool parts --id <id> --download --filename
<name>.step --overwrite` → `./cadtool inspect refs vendor/<name>.step --facts --planes --positioning`
→ set `VENDOR_TO_REF` in `parts/<name>.py` so the model lands in the SolidWorks frame →
`./cadtool pytest -k <name>` (`test_cots_vendor_matches_reference_frame`: bbox within 1.5 mm of
the reference) → `./cadtool gen parts/<name>.py` → `./cadtool python tools/import_reference.py`
(updates `manifest.json`). If the catalog model is worse, restore the reference copy. See
`vendor/README.md` for what has been tried.

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

`reference/*.step` are renamed copies of the SolidWorks exports in
`~/Documents/arm_assembly_organized/` — see `reference/README.md` for the naming map.
`reference/placements.json` holds every occurrence's placement extracted from the full-assembly
STEP. Both are **immutable inputs** (a checksum test guards them); regenerate with
`./cadtool python tools/import_reference.py` and `./cadtool python tools/extract_placements.py`
if the SolidWorks design changes. The cycloidal drive's 16 references are the CadQuery exports of
the `cycloidal_drive` repo at `2f1f67d` (`tools/export_cycloidal_cadquery.py` in that repo's venv,
then `./cadtool python tools/import_cycloidal_reference.py`), and its SolidWorks node is recorded
in `placements.json` as a designed module (pose only; the contents come from code).

## Robot description (URDF / SRDF / SDF)

`robot/` holds the arm's robot description, derived from the CAD:

```
robot/
├── frames.py          # THE kinematic decomposition: LINKS (which occurrences move together) + JOINTS
│                      # (axis point/direction, parent/child, limits from lib/params.py)
├── links/<link>.py    # gen_step() per rigid link, in the link's own frame (./cadtool gen robot/links/link1.py)
├── meshes/<link>.stl  # per-link meshes in mm (committed) - tools/export_link_meshes.py
├── arm.urdf           # SOURCE OF TRUTH (hand-edited ledger + numbers from tools/robot_frames.py)
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
./cadtool python tools/robot_frames.py                 # joint origins + link inertials (m, kg, rad)
./cadtool python tools/robot_frames.py --check robot/arm.urdf robot/arm.sdf   # files vs CAD (tests run this)
./cadtool python tools/export_link_meshes.py           # regenerate meshes after converting a part
./cadtool validate robot/arm.urdf --strict             # also .srdf / .sdf
./cadtool skill urdf snapshot --input robot/arm.urdf --output snapshots/arm_urdf.png
./cadtool viewer                                       # then ?file=robot/arm.urdf
```

After any CAD change that moves geometry: re-export the meshes, re-run the check, and if a
frame moved re-derive the affected `<origin>`/`<inertial>` values with `--urdf-draft` /
`--sdf-draft` (the drafts are scaffolding; the checked-in XML stays canonical).

## Tests

```bash
./cadtool pytest                 # everything (~420 tests; the geometry builds take ~2 min)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON
uv run pytest                    # equivalent (cadgen is a normal dependency)
```

## AI CAD assistance

The `cad@text-to-cad` plugin's `/cad:*` skills drive the generate → inspect → snapshot loop
this repo is aligned with (plus `dfam-check` for printability, `step-parts`, and the URDF/SRDF/SDF
skills). Agent-facing conventions live in `CLAUDE.md`.
