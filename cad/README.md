# robotic-arm — CAD (build123d)

Parametric CAD-as-code for the 3-joint arm, converted part-by-part from the original
SolidWorks design. This folder is a **separate uv project** (Python 3.12) — the motor-control
software in the repo root never depends on it.

**Status:** every custom part exists as an *import wrapper* around its SolidWorks reference
geometry (`reference/<name>.step`), purchased parts use their vendor STEPs, and
`assemblies/arm.py` places all of them from placements extracted from the SolidWorks
assembly — so the whole arm already assembles, renders and is tested. Converting a part means
replacing its wrapper body with real build123d code (see "Converting a part"). The cycloidal
drive is not modelled here (it lives in the `cycloidal_drive` repo).

## Setup (once per machine)

Requirements: [`uv`](https://docs.astral.sh/uv/), Node 18+ (for the CAD viewer), and the
[`cad@text-to-cad`](https://github.com/earthtojake/text-to-cad) Claude Code plugin (the repo's
`.claude/settings.json` enables its marketplace; install with
`claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git` then
`claude plugin install cad@text-to-cad`).

```bash
cd cad
./cadtool setup        # uv sync (build123d/OCP into ./.venv) + Playwright Chromium (~150 MB, snapshots only)
./cadtool pytest       # everything green?
```

Python is pinned to **3.12** in `.python-version`: build123d caps at `<3.14` and the `vtk`
wheel it pulls stops at cp312, while the system Python is 3.14. Don't bump it. Never run
bare `python` here — always `./cadtool …` or `uv run …` from `cad/`.

## `./cadtool` — the one entry point

| Command | What it does |
|---|---|
| `./cadtool step parts/<name>.py` | generate the committed `parts/<name>.step` (+ hidden `.<name>.step.glb` viewer artifact); add `--stl exports/<name>.stl` for a printable sidecar |
| `./cadtool step assemblies/arm.py` | build `assemblies/arm.step` (git-ignored, regenerable) |
| `./cadtool inspect refs <file.step> --facts --planes --positioning` | geometry facts, selector refs, planes; also `measure`, `align`, `frame`, `diff` |
| `./cadtool snapshot --input assemblies/arm.step --output snapshots/arm.png --size-profile assembly --view-labels` | PNG review packet (timestamp appended to the file name) |
| `./cadtool viewer` | start/reuse the CAD Viewer on this folder; open the printed URL and append `&file=assemblies/arm.step` |
| `./cadtool pytest [-m "not slow"]` | test suite (the fast lane skips geometry builds) |
| `./cadtool python -m assemblies.arm` | preview in the OCP CAD Viewer VS Code extension (`ocp_vscode`) |

`cadtool` always `cd`s to `cad/` (the plugin resolves paths from the working directory) and
exposes the plugin's `cadpy` helper via `PYTHONPATH` instead of installing it, so `uv sync`
can never prune it. `playwright` is a locked dev dependency for the same reason.

## Layout

```
cad/
├── cadtool                # bash wrapper around the plugin CLIs + uv (see above)
├── lib/
│   ├── params.py          # single source of truth for shared dimensions (tagged provenance)
│   ├── reference.py       # naming map SolidWorks <-> clean names, reference loaders, matches_reference()
│   ├── placements.py      # reference/placements.json -> build123d Location
│   ├── assembly.py        # AssemblyHelper (cadpy when available, tiny fallback otherwise)
│   └── export.py          # plugin-free STL/STEP export into exports/
├── parts/                 # one part per file, gen_step() returns it at its LOCAL origin
│   ├── _template.py           # designed (parametric) part template
│   ├── _wrapper_template.py   # import-wrapper template (day-one state of every custom part)
│   ├── _cots_template.py      # purchased part template (vendor STEP else envelope)
│   └── <name>.py + <name>.step   # 20 custom + 5 COTS parts; the .step is generated and committed
├── assemblies/
│   ├── arm.py             # the whole arm (15 top-level occurrences + the gripper module)
│   ├── gripper.py         # the gripper mechanism module (19 occurrences)
│   └── _occurrences.py    # place()/add_occurrences() helpers
├── reference/             # SolidWorks per-part exports (immutable inputs, committed) + manifest + placements
├── vendor/                # purchased-part STEPs (committed; source of truth for COTS parts)
├── tools/                 # import_reference.py, extract_placements.py (regenerate reference/ + vendor/nema17_pancake.step)
├── tests/                 # pytest: conventions, reference match, placements, assembly totals, params locks
├── exports/               # STL/3MF sidecars (git-ignored)
└── snapshots/             # snapshot PNGs (git-ignored)
```

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
- **COTS parts** declare `COTS = True`, `MASS_G` and `VENDOR_STEP`, and `gen_step()` returns
  the vendor STEP when present, else a parametric `_envelope()`.

`tests/test_parts_convention.py` enforces all of this automatically for every file in `parts/`.

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
5. `./cadtool step parts/<name>.py` to regenerate the committed STEP, then
   `./cadtool step assemblies/arm.py` + `./cadtool snapshot …` to eyeball it in place.

## Assembly

`assemblies/arm.py` and `assemblies/gripper.py` hold `OCCURRENCES` tables —
`(part, role, placement_key)` in SolidWorks document order — and place a fresh copy of each
part with `lib.placements` (`"j3_coupler#2"` = second occurrence of that part). Roles make
duplicate parts' labels unique (`j3_coupler:j2`, `gripper_end:1`); they are positional for
now. `tests/test_assembly.py` checks the rebuilt arm against the SolidWorks totals
(34 occurrences, 50 solids, volume and bounding box).

## Reference geometry and placements

`reference/*.step` (custom parts) and `vendor/*.step` (purchased parts) are renamed copies of
the SolidWorks exports in `~/Documents/arm_assembly_organized/` — see `reference/README.md`
for the naming map. `reference/placements.json` holds every occurrence's placement extracted
from the full-assembly STEP. Both are **immutable inputs** (a checksum test guards them);
regenerate with `./cadtool python tools/import_reference.py` and
`./cadtool python tools/extract_placements.py` if the SolidWorks design changes.

## Tests

```bash
./cadtool pytest                 # everything (geometry builds take a minute)
./cadtool pytest -m "not slow"   # fast lane: metadata, params, placements JSON
uv run pytest                    # also works without the plugin (lib/assembly.py falls back)
```

## AI CAD assistance

The `cad@text-to-cad` plugin's `/cad:*` skills drive the generate → inspect → snapshot loop
this repo is aligned with. Agent-facing conventions live in `CLAUDE.md`.
