# Toolchain — pins, upgrades, per-machine setup

Read this before bumping cadgen / build123d / OCP / Python, after a pull that changes `pyproject.toml` / `uv.lock`,
or when the kernel or the plugin misbehaves. Everyday running is `cad/AGENTS.md` "Running things"; the history
of every bump is `git log -- pyproject.toml uv.lock` from `cad/` (the older bumps carry their CHANGELOG entry as a git note).

## Pins
- cadgen and the plugin (what each one is: `cad/AGENTS.md` "Running things"): `pyproject.toml` pins
  `cadgen==<ver>` (Playwright, the snapshot browser's driver, is a plain cadgen dependency since 0.7.12: no `[snapshot]`
  extra); the `text-to-cad@earthtojake` plugin **v0.7.x** (`~/.claude/plugins/cache/earthtojake/text-to-cad/<ver>/`)
  pins the same cadgen version in one launch command, `uvx --no-config --managed-python --python 3.13 --from
  cadgen==<ver>`, written into every skill's `SKILL.md` (the skill docs + their scripts) and into its MCP server `cad`
  (`claude.mcp.json`: that command + `cadgen mcp`, started by Claude Code with every session, in uv's cache on a
  uv-managed Python 3.13 — never this venv; in a terminal its `cad_show` answers with a CAD Viewer link, `/mcp` turns
  it off). Bump both together, `./cadtool doctor` checks the skills' pin (`cadgen doctor` reads it from
  `skills/cad/SKILL.md`). `claude plugin install` / `update` take the marketplace's newest release, so update the plugin
  WITH a bump here, not before: the server's Viewer and this venv write the same store, and a cadgen never cleans a
  store a newer one wrote to in the last 30 days. Since 0.7.12 the server asks `api.texttocad.dev` once a day for the
  latest release (one anonymous request; `CADGEN_UPDATE_CHECK=0` turns it off) and, while it is behind, shows an
  Update button in the viewer and a line on the session's first `cad_show`: here that update IS a bump (the pin with
  the plugin: "After a pull …" below), never the plugin alone. Up to 0.7.6 it was `cad@text-to-cad` (plugin `cad`,
  marketplace `text-to-cad`): a machine that still has that one switches once (Gotchas below).
- **cadgen's telemetry stays off.** Since 0.7.16 cadgen sends usage stats and crash reports by default (a random
  install id; never file names, paths, contents or prompts) from every process — builds, the daemon, the viewer, the
  plugin's server — once a cadgen command has shown its one-line notice. This repo keeps them off, two ways:
  `cadtool` and `.env` export `CADGEN_TELEMETRY=0`, so no command of theirs shows the notice or sends; and
  `./cadtool setup` stores `cadgen telemetry off` in cadgen's state directory (`settings.json`:
  `~/.local/state/cadgen/` on Linux, `~/Library/Application Support/cadgen/` on the Mac), which the processes cadtool
  does not start go by — the environment does not reach them. The stored answer is per machine, and 0.7.7–0.7.15's
  opt-in `analytics` answer does not carry over: `telemetry` → `choice: off` in that file is the check
  (`./cadtool doctor`'s `sharing` line reports cadtool's own environment). `test_tooling.py` holds both.
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
git-lfs's hooks in `.git/hooks` — makes `git fetch` bring the git notes, and stores cadgen's telemetry off: Pins,
above — so run it before the plugin update, whose skills run cadgen outside cadtool),
`claude plugin marketplace update earthtojake && claude plugin update text-to-cad@earthtojake` (each scope — `--scope project` too; `~/.claude/plugins/installed_plugins.json` must show
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
  into `cad` / `dxf` / `urdf` / `srdf` / `sdf`; same STEP bytes; 0.7.5: the rebuild gate takes every file a build
  OPENS as an input (a native file tracer on `open` / `fopen` — not on `stat`, so an `exists()` that answered False
  records nothing, `cad/AGENTS.md` Gotchas) and every folder the model's code lists (by its entry names — hence no
  `__pycache__/` in the tree, same Gotchas), `declare_input`, `@memo` and the kernel-op cache are gone, model records
  moved to schema 8 (every model read stale once: `no record`), and a model's wrapper lost `__wrapped__` —
  `lib/models.raw()` reaches the body as `__cadgen_model__.func`, which cadgen counts as taking the child's file as the
  caller's source; a few parts came back with float noise (~1e-12) in their bytes, the same geometry by `./cadtool
  inspect diff`; 0.7.6: the store drops 0.7.4's op cache (`index/op`) and keeps itself under `CADGEN_STORE_MAX` (default
  20G) by evicting derived meshes / surfaces, never records or outputs; 0.7.7: the plugin's rename (Pins, above) and
  opt-in anonymous analytics in `cadgen viewer` / `cadgen mcp` — off until allowed on its card or in Settings,
  `cadgen analytics status|off`, `DO_NOT_TRACK=1`; 0.7.8–0.7.11: the plugin starts CAD's MCP server (Pins, above), its
  `gcode` / `bambu-labs` skills became an OrcaSlicer route and a Bambu Connect handoff (their old scripts are gone), the
  upstream repo dropped Git LFS (plugin installs need no `git-lfs`), and cadgen's own changes are the MCP server's and
  the bundled viewer's (STEPs with empty components load); same STEP bytes, no stale wave; 0.7.12–0.7.15: faster gates
  and rebuilds (a rebuild whose STEP bytes would not change keeps the file: `kept STEP: …`), every process cadgen starts
  runs as `python -P` (the daemon's: `./cadtool daemon stop`, below), Playwright became a plain dependency (the
  `[snapshot]` extra is gone) and a snapshot fetches its headless shell itself when it is missing, cadgen's mesher closes
  the holes and non-manifold edges it left on some filleted / swept faces (the display meshes re-tessellate once), the
  plugin's skills pin cadgen in their `SKILL.md` launch command (their pinning `requirements.txt` and the
  `cad-mcp-setup` skill are gone) and its server tells of new releases (Pins, above); same STEP bytes, no stale wave;
  0.7.16–0.7.19: telemetry on by default (Pins, above: `cadgen analytics` became `cadgen telemetry`,
  `CADGEN_ANALYTICS` is no longer read), a listed folder's digest leaves out the model's own outputs and cadgen's STEP
  staging folders (#564: `cad/AGENTS.md` Gotchas), a build whose worker died names why (`worker N was killed by
  SIGTERM (signal 15)` / `SIGKILL`: on the Linux PC, earlyoom), stricter URDF / SDF / SRDF validators, every URDF
  visual pickable in the viewer; no record-schema change (`closure.own` is optional), same STEP bytes, no stale wave):
  a retired interface fails with a teaching error, never an alias (a bad `--display` value is refused with the list of
  presets). The freshness gate
  does not hash cadgen's own version, so a bump makes no model stale (a record-schema change aside, as in 0.7.5) and a
  plain `gen` afterwards rewrites nothing: to
  hash-gate a bump, build every model on the old pin (a fresh worktree has no STEPs), hash, then `./cadtool gen <every
  model> --force` on the new one and `shasum -c`. On a bump re-check the
  private names this repo leans on — `cadgen.authoring.build_in_progress` / `_build` / `ModelDef.func|fmt|script_path|out`
  (a model's `__cadgen_model__`; `lib/models.py`, `tests/conftest.py`, `test_parts_convention.py`),
  `cadgen._internal.component_package` (`_shape_brep_bytes`, `_build123d_shape_from_brep_bytes`), the `-m cadgen.daemon` cmdline
  of the venv interpreter (`./cadtool daemon stop`; it is `.venv/bin/python3` on Linux and `.venv/bin/python` on macOS,
  and `python -P -m cadgen.daemon` since 0.7.13 — the pattern takes all of them, `test_tooling.py`) — and that
  `./cadtool why assemblies/arm.py` still lists every child (one per occurrence of `arm.py OCCURRENCES`, the modules'
  rows included) as
  pinned (a `build_in_progress` that silently read False would inline every child and still build).
- `./cadtool doctor` never reads `~/.claude/plugins/installed_plugins.json`: `plugin_dir()` pairs the venv's cadgen
  version with the plugin cache directory of the same name, so after `uv sync` to a new cadgen it reports `pin OK`
  while the plugin may still be INSTALLED at the old version (`claude plugin marketplace update` alone creates the
  new cache directory while both scopes still record the old version). Update the plugin in BOTH
  scopes (`claude plugin update text-to-cad@earthtojake`, then `--scope project`), check that file shows the new `version` +
  `installPath` for each, and restart Claude Code (its first start after an update fetches the server's new cadgen;
  on a slow connection start once with `MCP_TIMEOUT=300000 claude`). A session first started in `cad/` records a third
  entry (project scope, `projectPath` = `cad/`) that no `update` reaches: run from `cad/`, `update --scope project`
  answers for the repo root's entry ("already at the latest version") while `install` there still reports the old one.
  Copy the root entry's `version`, `installPath`, `gitCommitSha` and `lastUpdated` into it by hand (back the file up
  first); `claude plugin list` run from `cad/` then shows the new version for all three entries.
- The plugin's rename (0.7.7: `cad@text-to-cad` → `text-to-cad@earthtojake`) needs a one-time switch per machine, not
  an `update`: Claude Code keys a marketplace by its git URL, so `claude plugin marketplace add` of the same URL on a
  machine that registered it as `text-to-cad` answers "already on disk — declared in user settings" and adds nothing.
  Drop `text-to-cad` from `extraKnownMarketplaces` and `cad@text-to-cad` from `enabledPlugins` in
  `~/.claude/settings.json`, `claude plugin marketplace remove text-to-cad` (its installs go with it), then
  `claude plugin marketplace add https://github.com/earthtojake/text-to-cad.git` and
  `claude plugin install text-to-cad@earthtojake` in each scope (`--scope project` from the repo root and from `cad/`),
  and restart Claude Code. Do it after the pull that brings the rename: a checkout older than it still enables
  `cad@text-to-cad`. Run on the Mac: `marketplace add` declares `earthtojake` in `~/.claude/settings.json` by itself,
  and each `install --scope project` rewrites the repo's two `.claude/settings.json` with the same content in another
  key order — `git checkout` them.
