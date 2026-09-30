# Toolchain — pins, upgrades, per-machine setup

Read this before bumping cadgen / build123d / OCP / Python, after a pull that changes `pyproject.toml` / `uv.lock`,
or when the kernel or the plugin misbehaves. Everyday running is `cad/AGENTS.md` "Running things"; the history
of every bump is `git log -- pyproject.toml uv.lock` from `cad/` (the older bumps carry their CHANGELOG entry as a git note).

## Pins
- cadgen and the plugin (what each one is: `cad/AGENTS.md` "Running things"): `pyproject.toml` pins
  `cadgen[snapshot]==<ver>`; the `cad@text-to-cad` plugin **v0.7.x** (`~/.claude/plugins/cache/text-to-cad/cad/<ver>/skills/`,
  the skill docs + the step.parts script) pins the same cadgen version in its `skills/cad/requirements.txt` — bump both
  together, `./cadtool doctor` checks (plugin updates need `git-lfs` on `PATH`).
- **build123d 0.11.1 / OCP 7.9.3**: cadgen 0.7.x requires `build123d>=0.11.1,<0.12` and
  `cadquery-ocp-novtk>=7.9,<8`; `pyproject.toml` pins the exact kernel (`cadquery-ocp-novtk==…`, the
  STEP bytes are per-kernel) and must never gain `cadquery-ocp`, the VTK build (see Gotchas below). On 0.10 / 7.8.1 cadgen could not
  build 11 parts (OCCT 7.8.1 mis-read BinTools VERSION_4 component objects), could not export a linked
  child (`LazyCompound` needs 0.11's `wrapped` property) and its STEP writer needs `HArray1.Value`.
  `test_part_survives_cadgen_component_round_trip` locks the first.
- Python is pinned to **3.12** (`.python-version`; the OCP wheels are verified on 3.12 — bump only with the full suite
  green).

## After a pull that changes `pyproject.toml` / `uv.lock` (per-machine state git does not carry)
`./cadtool daemon stop && ./cadtool setup` (it also (re)installs the ruff git pre-commit hook: a stub
`.git/hooks/pre-commit` → the committed `.githooks/pre-commit`; never `core.hooksPath`, which would switch off
git-lfs's hooks in `.git/hooks` — and makes `git fetch` bring the git notes), `claude plugin marketplace update text-to-cad && claude plugin
update cad@text-to-cad` (each scope — `--scope project` too; `~/.claude/plugins/installed_plugins.json` must show
the new version for both, `doctor` cannot tell — see Gotchas below; restart Claude Code), then `./cadtool doctor` must be clean. No
`CAD_PLUGIN` in a shell profile (it overrides the plugin detection).

## Gotchas (all verified)
- `uv sync` prunes anything `uv pip install`-ed; `uv run` doesn't — that's why `cadgen` is a
  pyproject dependency, not a manual install.
- After `uv sync` changes cadgen / build123d / OCP, `./cadtool daemon stop`: the warm daemon's workers
  keep the old code loaded (its identity token only tracks cadgen's version and file mtimes; cadgen
  has no stop verb of its own and the daemon shrugs off a bare SIGTERM). The next `gen` starts a fresh one.
- `cadquery-ocp` (VTK) and `cadquery-ocp-novtk` own the same 322 `OCP/` files (the 162 MB kernel `.so`
  included). When uv removes one (as when cadgen 0.6 dropped `cadquery-ocp`) it deletes them and
  still counts the other as installed: `uv sync` reports success and `import OCP.gp` fails (a bare `import OCP`
  can still succeed: the leftover `OCP/` directory is an empty namespace package). Repair with
  `uv sync --reinstall-package cadquery-ocp-novtk` (`./cadtool setup` does it when `OCP.gp` does not import;
  `test_tooling.py` checks every RECORD file exists). cadgen's `doctor` (since 0.7) runs a bare `import OCP` in a
  subprocess (exit 4 when it fails to load, only a note when OCP is absent), so it does NOT catch this gutted kernel —
  the `OCP.gp` probe of `./cadtool setup` does.
- cadgen makes hard cutovers (0.6.0: cache / sidecar schemas, so every model read stale once; 0.6.5: the
  inspect CLI; 0.6.6 was additive only — `cadgen.eng_drawing` and matplotlib / pillow as hard deps, nothing
  retired, no stale wave, same STEP bytes; 0.7.0: the rebuilt CAD Viewer and the snapshot display settings — `--render`
  is gone, `--display` takes a preset or the grouped JSON, the old mode names (`transparent`, `shaded_edges` …) are
  refused, and viewer state is per browser tab; no store schema change, same STEP bytes; 0.7.3 / 0.7.4 were fixes
  only — cadgen's own mesher (the viewer, snapshots, `./cadtool export`; not `robot/meshes/`, which build123d's
  `export_stl` writes) meshes a hole through a curved face without slivers, and its version bump re-tessellates the
  store's display meshes once; the viewer's playback and orbit were fixed, and the plugin folded its `cad-viewer` skill
  into `cad` / `dxf` / `urdf` / `srdf` / `sdf`; same STEP bytes): a retired interface fails
  with a teaching error, never an alias (a bad `--display` value is refused with the list of presets). The freshness gate
  does not hash cadgen's own version, so a bump makes no model stale and a plain `gen` afterwards rewrites nothing: to
  hash-gate a bump, build every model on the old pin (a fresh worktree has no STEPs), hash, then `./cadtool gen <every
  model> --force` on the new one and `shasum -c`. On a bump re-check the
  private names this repo leans on — `cadgen.authoring.build_in_progress` / `_build` / `ModelDef.func|fmt|script_path|out`
  (`lib/models.py`, `tests/conftest.py`, `test_parts_convention.py`), `cadgen._internal.component_package`
  (`_shape_brep_bytes`, `_build123d_shape_from_brep_bytes`), the `-m cadgen.daemon` cmdline
  of the venv interpreter (`./cadtool daemon stop`; it is `.venv/bin/python3` on Linux and `.venv/bin/python` on macOS —
  the pattern takes both, `test_tooling.py`) — and that `./cadtool why assemblies/arm.py` still lists every child (one per
  occurrence of `arm.py OCCURRENCES`, the modules' rows included) as
  pinned (a `build_in_progress` that silently read False would inline every child and still build).
- `./cadtool doctor` never reads `~/.claude/plugins/installed_plugins.json`: `plugin_dir()` pairs the venv's cadgen
  version with the plugin cache directory of the same name, so after `uv sync` to a new cadgen it reports `pin OK`
  while the plugin may still be INSTALLED at the old version (`claude plugin marketplace update` alone creates the
  new cache directory while both scopes still record the old version). Update the plugin in BOTH
  scopes (`claude plugin update cad@text-to-cad`, then `--scope project`), check that file shows the new `version` +
  `installPath` for each, and restart Claude Code; the marketplace clone's LFS pointers (`assets/**`, `models/**`)
  are excluded by its own `.lfsconfig` and need no `git lfs pull`. A session first started in `cad/` records a third
  entry (project scope, `projectPath` = `cad/`) that no `update` reaches: run from `cad/`, `update --scope project`
  answers for the repo root's entry ("already at the latest version") while `install` there still reports the old one.
  Copy the root entry's `version`, `installPath`, `gitCommitSha` and `lastUpdated` into it by hand (back the file up
  first); `claude plugin list` run from `cad/` then shows the new version for all three entries.
