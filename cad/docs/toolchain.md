# Toolchain — pins, upgrades, per-machine setup

Read this before bumping cadgen / build123d / OCP / Python, after a pull that changes `pyproject.toml` / `uv.lock`,
or when the kernel, the engine or the plugin misbehaves. Everyday running is `cad/AGENTS.md` "Running things"; the
history of every bump is `git log -- pyproject.toml uv.lock` from `cad/` (the older bumps carry their CHANGELOG entry
as a git note).

## Pins
- cadgen and the plugin (what each one is: `cad/AGENTS.md` "Running things"): `pyproject.toml` pins
  `cadgen==<ver>`; the `text-to-cad@earthtojake` plugin (`~/.claude/plugins/cache/earthtojake/text-to-cad/<ver>/`)
  pins the same cadgen version in one launch command, `uvx --no-config --managed-python --python 3.13 --from
  cadgen==<ver>`, written into the `SKILL.md` of every skill that runs cadgen and into its MCP server `cad`
  (`claude.mcp.json`: that command + `cadgen mcp`, started by Claude Code with every session, in uv's cache on a
  uv-managed Python 3.13 — never this venv; `cad_show` takes a file's absolute path or a document id and answers with
  the page's link; `/mcp` turns it off). Bump both together; `./cadtool doctor` checks the pin (this venv's cadgen
  against the `cadgen==<ver>` in the plugin's `skills/cad/SKILL.md`). `claude plugin install` / `update` take the
  marketplace's newest release, so update the plugin WITH a bump here, not before: its skills describe the cadgen
  they pin. The plugin's server asks `api.texttocad.dev` once a day for the latest release (one anonymous request;
  `CADGEN_UPDATE_CHECK=0` turns it off) and, while it is behind, shows an Update button in the viewer: here that
  update IS a bump (the pin with the plugin: "After a pull …" below), never the plugin alone. Up to 0.7.6 it was
  `cad@text-to-cad` (plugin `cad`, marketplace `text-to-cad`): a machine that still has that one switches once
  (Gotchas below).
- **One cadgen engine per installation.** cadgen builds in an engine — warm workers, the documents they made, the
  CAD Viewer's page — one per Python environment it is installed in (keyed by the environment's `sys.prefix`): this
  venv's (started by the first `./cadtool gen` / `viewer` / `snapshot`; no timer stops it, `./cadtool stop` does) and
  the plugin's (its uvx environment, started by its server). A model run reaches its own venv's engine only, so every
  build here runs on this venv's pinned kernel. Every worktree's venv is an installation of its own, with an engine
  of its own and a memory budget of its own (`./cadtool status` shows it): `./cadtool stop` when leaving a worktree.
  The first engine serves on port 3245, another takes the next free port and keeps it. Each document is cached as
  last built in `~/.cache/cadgen/documents` (kept until deleted: `./cadtool cadgen delete <id>`); cadgen 0.7's
  store beside it (`objects/`, `index/`) is read by nothing — the plugin's `cadgen doctor --prune` removes it once no
  0.7 has written it for 7 days (Gotchas: never through `./cadtool cadgen doctor`).
- **cadgen's telemetry stays off.** cadgen sends usage stats and crash reports by default (a random install id;
  never file names, paths, contents or prompts; its viewer also counts which of its tools are used) from every
  process — builds, the engine, the plugin's server — once a cadgen command has shown its one-line notice. This repo
  keeps them off, two ways: `cadtool` and `.env` export `CADGEN_TELEMETRY=0`, so no command of theirs shows the notice
  or sends; and `./cadtool setup` stores `cadgen telemetry off` in cadgen's state directory (`settings.json`:
  `~/.local/state/cadgen/` on Linux, `~/Library/Application Support/cadgen/` on the Mac), which the processes cadtool
  does not start go by — the environment does not reach them. The stored answer is per machine: `telemetry` →
  `choice: off` in that file is the check. `test_tooling.py` holds both.
- **build123d 0.11.1 / OCP 7.9.3**: cadgen requires `build123d>=0.11.1,<0.12` and `cadquery-ocp-novtk>=7.9,<8`;
  `pyproject.toml` pins the exact kernel (`cadquery-ocp-novtk==…`, the STEP bytes are per-kernel) and must never gain
  `cadquery-ocp`, the VTK build (see Gotchas below). On OCCT 7.8.1 some parts could not be read back from the BinTools
  VERSION_4 bytes cadgen keeps every part as (its document cache, its STEP reader's wire):
  `test_part_survives_a_bintools_round_trip` locks that.
- Python is pinned to **3.12** (`.python-version`; the OCP wheels are verified on 3.12 — bump only with the full suite
  green).

## After a pull that changes `pyproject.toml` / `uv.lock` (per-machine state git does not carry)
`./cadtool stop && ./cadtool setup` (it also (re)installs the ruff git pre-commit hook: a stub `.git/hooks/pre-commit`
→ the committed `.githooks/pre-commit`; makes `git fetch` bring the git notes, and stores cadgen's telemetry off: Pins,
above — so run it before the plugin update, whose skills run cadgen outside cadtool),
`claude plugin marketplace update earthtojake && claude plugin update text-to-cad@earthtojake` (each scope —
`--scope project` too; `~/.claude/plugins/installed_plugins.json` must show the new version for every entry (a
session first started in `cad/` adds a third, which no `update` reaches: copy the new version into it by hand),
`doctor` cannot tell — see Gotchas below; restart Claude Code), then `./cadtool doctor` must be clean. No `CAD_PLUGIN`
in a shell profile (it overrides the plugin detection).

## Gotchas (all verified)
- `uv sync` prunes anything `uv pip install`-ed; `uv run` doesn't — that's why `cadgen` is a
  pyproject dependency, not a manual install.
- The venv's packages are compiled to bytecode when they are installed — `./cadtool setup` and CI sync with
  `--compile-bytecode` — because no process writes its own: `cadtool` sets `PYTHONDONTWRITEBYTECODE=1` (no
  `__pycache__/` in the tree), so without them every engine worker and pytest worker compiles build123d, numpy … anew
  on import. Not as a `[tool.uv] compile-bytecode` setting: uv then re-checks every file of the venv on every `uv run`
  (0.3 s a command against 0.03 s, uv 0.13.4). A package a later `uv run` installs (after a pull) stays uncompiled
  until the next `./cadtool setup`.
- After `uv sync` changes cadgen / build123d / OCP, `./cadtool stop`: the engine's workers keep the old packages
  loaded. A source edit needs no restart — the next run picks it up and names it (`rebuilt: ../lib/assembly.py:
  label_text changed`).
- `cadquery-ocp` (VTK) and `cadquery-ocp-novtk` own the same `OCP/` files (the kernel `.so`
  included). When uv removes one (as when cadgen 0.6 dropped `cadquery-ocp`) it deletes them and
  still counts the other as installed: `uv sync` reports success and `import OCP.gp` fails (a bare `import OCP`
  can still succeed: the leftover `OCP/` directory is an empty namespace package). Repair with
  `uv sync --reinstall-package cadquery-ocp-novtk` (`./cadtool setup` does it when `OCP.gp` does not import, and
  `./cadtool doctor` reports it; `test_tooling.py` checks every RECORD file exists).
- cadgen makes hard cutovers, and 0.8 has no teaching errors: a retired flag is an unrecognised argument, a retired
  name an `ImportError` / `AttributeError`, a retired decorator argument a `TypeError` — read the error as the spec
  (the 0.7 → 0.8 move: upstream `docs/migrations/migrating-0.7-to-0.8.md`). What each bump retired or changed is its
  commit (`git log -- pyproject.toml uv.lock`). The engine keys a body by its code and the files it reads, its
  document by the toolchain (build123d, OCP, the system and the architecture) — not by cadgen's release, which it
  records only — so a cadgen bump makes no model rebuild and a plain `gen` afterwards rewrites nothing: to hash-gate a
  bump, build every model on the old pin (a fresh worktree has no STEPs), hash, then `./cadtool stop` and
  `./cadtool gen --force <every model>` on the new one and `shasum -c`. A bump whose STEP writer changed (0.7 → 0.8)
  changes the bytes: compare the geometry instead, every leaf in world coordinates (`./cadtool inspect diff`, after
  checking that `facts()` reads the old files the same way).
- The private names this repo leans on — re-check them on a bump: `cadgen.engine.model._CONTEXT.build` (a build in
  progress on this thread: `lib/models.py building()`), a model's `.function` / `.outputs` (`Output.kind` / `.out`) /
  `__xbf_model__` and `Model.build` (`lib/models.py`, `tests/conftest.py`, `test_parts_convention.py`) — and that
  `./cadtool gen assemblies/arm.py` still lists every part as `…, a child` (a `building()` that silently read False
  would build every child as a raw body: the arm would still build, its children's STEPs unwritten).
- Never run `cadgen doctor` from this venv (`./cadtool doctor` does not): it registers the environment it runs in as
  the machine's installation of its version (`installations/<ver>.json` in cadgen's state directory — the record the
  `cadgen` command, `~/.local/bin/cadgen`, and the plugin's skills run by) and starts that environment's engine; a
  worktree's venv registered there goes with the worktree. The plugin's server registers its own as it starts.
- `./cadtool doctor` never reads `~/.claude/plugins/installed_plugins.json`: `plugin_dir()` pairs the venv's cadgen
  version with the plugin cache directory of the same name, so after `uv sync` to a new cadgen it reports the pin as
  matching while the plugin may still be INSTALLED at the old version (the new cache directory exists once one scope
  has updated, the other entries still recording the old version). `claude plugin marketplace update` alone makes no
  cache directory (Claude Code 2.1.304: it refreshes only the clone `~/.claude/plugins/marketplaces/earthtojake/`), so
  on a branch that bumps the pin before the plugin update `doctor` warns `no plugin cache for cadgen <ver>` and reports
  a pin MISMATCH against the old one: `CAD_PLUGIN=~/.claude/plugins/marketplaces/earthtojake ./cadtool doctor` checks
  the new pin against that clone. Update the plugin in BOTH
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
  `cad@text-to-cad`. Seen on the Mac: `marketplace add` declares `earthtojake` in `~/.claude/settings.json` by itself,
  and each `install --scope project` rewrites the repo's two `.claude/settings.json` with the same content in another
  key order — `git checkout` them.
