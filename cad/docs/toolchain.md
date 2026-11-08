# Toolchain — pins, upgrades, per-machine setup

Read this before bumping cad / build123d / OCP / Python, after a pull that changes `pyproject.toml` / `uv.lock`,
or when the kernel, the server or the plugin misbehaves. Everyday running is `cad/AGENTS.md` "Running things"; the
history of every bump is `git log -- pyproject.toml uv.lock` from `cad/` (the older bumps carry their CHANGELOG entry
as a git note).

## Pins
- **One pin, `"cad[build123d]==<ver>"` in `pyproject.toml`**, for the three things upstream releases together: the
  library in the venv (`cad`, imported here as `cadgen`, its earlier name: an alias of the same modules, so
  `cadgen.read_step` is `cad.read_step` - from 0.11.1 importing `cadgen` warns `CadDeprecationWarning`, an error under
  pytest (Gotchas), so a bump past 0.11.0 moves every import, and the docs that name `cadgen`, to `cad`); the
  machine's server, `cad-server`, which is the `cad` command and everything that runs (a uv tool: `./cadtool server` installs it from the pin on a uv-managed Python, `uv tool dir --bin` holds the
  command); and the `text-to-cad@earthtojake` plugin (`~/.claude/plugins/cache/earthtojake/text-to-cad/<ver>/`), whose
  `claude.mcp.json` pins `cad-mcp==<ver>`: CAD's MCP server `cad`, a shim Claude Code starts through uvx with every
  session, which talks to the running server (`cad_show` takes a file's absolute path or a document id and answers with
  the page's link; `cad_inspect` / `cad_export` / `cad_snapshot` run asynchronously; `/mcp` turns it off). Bump them
  together; `./cadtool doctor` reads each against the pin. `cadtool` restates the pin nowhere, and `cad-server` is never
  a dependency here (`test_tooling.py` holds both). `claude plugin install` / `update` take the marketplace's newest
  release, so update the plugin WITH a bump here, not before: its skills describe the release they came with. Upstream
  can release again between a bump and its plugin update, and the plugin then lands ahead of the pin: after `claude
  plugin marketplace update`, the clone's `VERSION` must be this pin - if it moved, bump again first.
  **The server comes before the plugin, always:** the shim never moves a live installation, but on a machine with none
  it installs `cad-server` at its own release through uvx and makes that the machine's, with no telemetry off stored.
  The server asks `api.texttocad.dev` for the latest release (one anonymous request; `CAD_UPDATE_CHECK=0` in the server's
  environment turns it off) and, while it is behind, shows an Update button in the viewer: here that update IS a bump
  (the pin with the plugin: "After a pull …" below), and the button is never pressed (Gotchas). Nor does the server
  update itself: automatic updates stay off (below).
- **One cad server per computer.** Every venv, worktree and session, and the plugin, use one server — the engine, warm
  workers, the documents they made, the CAD Viewer's page —, started by the first client that finds none (no timer stops
  it) from the machine's current installation, which `cad doctor` registers (`./cadtool server` runs it). A model run
  is built by a worker of the CALLER's Python: a worker is keyed by its script, the environment and the cad release, so
  every build here runs on this venv's pinned kernel and two worktrees share no worker, and the run's environment
  (`PYTHONPATH`, `CAD_ENGINE_MEMO`) goes with each build. What is NOT in the venv: `cad <type> snapshot` / `export` of a file and
  the viewer run in the server's own kernel environment, in its cache (`cad[build123d]` at its release, so ranges and not
  this project's pins), made on its first need (a first snapshot after a new release fetches it), `cad <type> dfm` in
  its `cad[trimesh]` environment, made the same way, the validators in the `cad` process; none writes a STEP this repo
  keeps. A library on another
  release than the server still builds if the two speak one contract (a line says which to update) and is refused if
  not. One memory budget: `./cadtool status` shows the server's processes against a cap of a quarter of the RAM and the
  workers against the core count; a worker retires over a quarter of the RAM, and the least recently used goes at the
  cap or on memory pressure (no idle timer). Those caps (`CAD_ENGINE_MEMORY_MB`, `CAD_ENGINE_WORKERS`,
  `CAD_ENGINE_WORKER_RSS_MB`) are read once, from the environment of whichever client started the server. pytest is
  outside all of it: its workers are plain processes. `./cadtool stop` ends this venv's idle workers, no other session's
  (`tools/stop_workers.py`: `cad status --json` names each worker's Python, the server clears a dead idle one at its next
  grant): do it when leaving a worktree. `./cadtool stop --device` stops the server for every session on the machine.
  The server serves on port 3245, the next free port if that is held, and keeps it. Each document is cached as last
  built in `~/.cache/cad/documents` (kept until deleted: `./cadtool cad delete <id> --yes`), its id made from the script's path
  and function, so worktrees never collide.
- **cad's telemetry stays off.** cad sends usage stats and crash reports by default (a random install id; never file
  names, paths, contents or prompts; its viewer also counts which of its tools are used) from every process — the
  command, the server, the plugin's — once a `cad` command has shown its one-line notice. This repo keeps them off, two
  ways: `cadtool` and `.env` export `CAD_TELEMETRY=0`, so no command of theirs shows the notice or sends; and `./cadtool
  server` stores `cad telemetry off` in cad's state directory (`settings.json`: `~/.local/state/cad/` on Linux,
  `~/Library/Application Support/cad/` on the Mac) BEFORE `cad doctor` starts the server, which goes by the stored choice
  alone — the environment does not reach it. The stored answer is per machine: `telemetry` → `choice: off` in that file
  is the check, and `./cadtool doctor` reads it with the environment's switch taken off (with the switch on, `cad
  telemetry status` says off whatever is stored). `test_tooling.py` holds the order. Two ways this fails silently on a
  bump: a renamed switch does nothing and says nothing, and a new state folder starts with no answer (the old off moves into it only with `cad doctor` or `cad setup`).
- **cad's automatic updates stay off.** From 0.11 the server reads the version feed once an hour and, by default,
  installs a release that breaks nothing (a patch release below 1.0) by itself, moves every agent app's plugin to it
  and switches once nothing has run for a minute: the machine would leave the pin under every session. `./cadtool
  server` stores `cad update auto off` (`updates` → `automatic: false` in the same `settings.json`) beside the
  telemetry off, before `cad doctor` starts the server; the viewer's menu item Automatic updates stays unticked.
  `./cadtool doctor` reads it (`cad update auto status`: `off (your choice)`) and fails without it. The version check
  and its Update button stay: they say a release is out, which here is a bump.
- **build123d 0.13.0 / OCP 8.0.1 (OCCT 8)**: cad 0.11's `build123d` extra requires `build123d>=0.13.0,<0.14` and
  `cadquery-ocp-novtk>=8.0,<8.1`; `pyproject.toml` pins the exact kernel (`cadquery-ocp-novtk==…`, the STEP bytes are
  per-kernel) and must never gain `cadquery-ocp`, the VTK build (see Gotchas below). OCCT 8.0.1's two known defects and
  what this repo checks for them: Gotchas below. Every part must read back from the BinTools VERSION_4 bytes cad
  keeps it as (its document cache, its STEP reader's wire): `test_part_survives_a_bintools_round_trip`.
- Python is pinned to **3.14** (`.python-version`, a uv-managed interpreter on every machine: `[tool.uv]
  python-preference`). cad requires it, and build123d and the OCP wheels allow nothing newer; bump only with the full
  suite green.

## After a pull that changes `pyproject.toml` / `uv.lock` (per-machine state git does not carry)
A new pin moves the server under every session on the machine, so close the others first. `./cadtool stop &&
./cadtool setup` (it also (re)installs the ruff git pre-commit hook: a stub `.git/hooks/pre-commit` → the committed
`.githooks/pre-commit`; makes `git fetch` bring the git notes; and runs `./cadtool server`, which reinstalls `cad-server`
at the pin, stores cad's telemetry and automatic updates off, runs `cad doctor` and stops the old release's server - in that order, and all of
it before the plugin update: Pins, above), `claude plugin marketplace update earthtojake` (its clone's `VERSION` must be
this pin: Pins, above), then from the repo root `claude plugin update text-to-cad@earthtojake --scope user` and
`--scope project` (`~/.claude/plugins/installed_plugins.json` must show the new version for every entry (a session first
started in `cad/` adds a third, which no `update` reaches: copy the new version into it by hand, Gotchas below); restart
Claude Code), then `./cadtool doctor` must be clean. No `CAD_PLUGIN` in a shell profile (it overrides the plugin
detection).

## Testing a bump (a branch never moves the machine)
`./cadtool setup` on a bump's branch leaves the machine's server alone: a linked worktree only checks it, because `main`
and the other sessions are still on the old release. Test the new one beside it, on a server of its own, with every
command through a small wrapper outside the repo that exports, under one folder on disk (`/tmp` is RAM on the PC):
`CAD_STATE_DIR` and `CAD_CACHE_DIR` (with a state folder named, nothing moves out of the old release's),
`CAD_ENGINE_DIR` (its `stop` stays there), `UV_TOOL_DIR`, `UV_TOOL_BIN_DIR` and `CAD_LAUNCHER_DIR` (no second `cad` on the
machine). `./cadtool server` then works from the worktree, and the rest of `cadtool` follows. Check these names against
upstream's source first: a renamed one does nothing, silently. A change of Python alone needs the same for the document
cache, whose key omits Python. The proof the geometry did not move: the hash gate in Gotchas below.

## Gotchas (all verified)
- `uv sync` prunes anything `uv pip install`-ed; `uv run` doesn't — that's why `cad` is a pyproject dependency, not a
  manual install.
- The venv's packages are compiled to bytecode when they are installed — `./cadtool setup` and CI sync with
  `--compile-bytecode` — because no process writes its own: `cadtool` sets `PYTHONDONTWRITEBYTECODE=1` (no
  `__pycache__/` in the tree), so without them every worker and pytest worker compiles build123d, numpy … anew on
  import. Not as a `[tool.uv] compile-bytecode` setting: uv then re-checks every file of the venv on every `uv run`
  (0.3 s a command against 0.03 s, uv 0.13.4). A package a later `uv run` installs (after a pull) stays uncompiled
  until the next `./cadtool setup`.
- `uv lock` on a uv older than the one that wrote `uv.lock` downgrades the file, not only the package it was asked to
  move (its `revision`, and `marker`s on packages nothing touched). Reading it is fine
  (`uv sync`, `./cadtool setup` and `--frozen` leave it alone), so on a machine that is behind, re-lock with a one-off
  newer uv, `uvx uv@<ver> lock`: a bump's lock diff is cad's own entries and nothing else (a change of Python also
  drops the forks and wheels of the Pythons it leaves).
- After `uv sync` changes build123d / OCP, `./cadtool stop`: a worker keeps the packages it loaded, and the same cad
  release reuses it (a new cad release starts a new worker, the old one left idle). A source edit needs no restart — the
  next run picks it up and names it (`rebuilt: ../lib/assembly.py: label_text changed`).
- `cadquery-ocp` (VTK) and `cadquery-ocp-novtk` own the same `OCP/` files (the kernel `.so`
  included). When uv removes one it deletes them and
  still counts the other as installed: `uv sync` reports success and `import OCP.gp` fails (a bare `import OCP`
  can still succeed: the leftover `OCP/` directory is an empty namespace package). Repair with
  `uv sync --reinstall-package cadquery-ocp-novtk` (`./cadtool setup` does it when `OCP.gp` does not import, and
  `./cadtool doctor` reports it; `test_tooling.py` checks every RECORD file exists).
- cad makes hard cutovers on the command line, and has no teaching errors: a retired verb or flag is an unrecognised
  argument (0.11: `cad snapshot` is `cad <type> snapshot`, the type the file's), a retired decorator argument a
  `TypeError` — read the error as the spec (each move: upstream `docs/migrations/`) — and a retired environment
  variable says nothing at all. The library's renames keep the old spelling one release at least, as an alias that
  warns once with `CadDeprecationWarning` (0.11: `read_step` → `step.read`, `cadgen.geometry` → `cadgen.build123d`):
  pytest turns that warning into an error (`pyproject.toml` `filterwarnings`), so a renamed name is moved by its bump,
  not when the alias goes. What each bump retired or
  changed is its commit (`git log -- pyproject.toml uv.lock`). The engine keys a body by its code and the files it reads,
  its document by the toolchain (build123d, OCP, the system and the architecture) — not by cad's release or Python,
  neither of which it weighs — so a cad bump makes no model rebuild and a plain `gen` afterwards rewrites nothing: to
  hash-gate a bump, copy every model's STEP aside from a checkout on the old pin (a fresh worktree has none; list the
  models with `git ls-files`, a checkout can hold a STEP whose model is gone), hash them, `--force` every model on the old
  pin as a control, then `./cadtool stop` and `./cadtool gen --force <every model>` on the new one and compare, always
  against the copy. A `--force` build can name the elements of mirror-image features the other way round (the swapped
  `~<digest>` names of two edges or vertices, every other record equal: cad derives a name from the history the kernel
  reports, whose order a boolean can change), so it rewrites such a part's bytes on the SAME pin too, and a new
  Python moves those digests on many more files: settle each file that fails with `./cadtool inspect diff` against its
  old copy. A bump whose STEP writer or kernel changed changes the bytes: compare the geometry instead, every leaf in world coordinates
  (`./cadtool inspect diff`, after checking that `facts()` reads the old files the same way: the old STEPs read by the
  old venv and by the new one). A new kernel can re-approximate a curve it builds (a chamfer along a spline) with
  another degree and other poles: the surface stays put, but `volume` - an integral over the representation - moves by
  float noise above `inspect diff`'s default `--tol`. Settle such a
  leaf by sampling its faces' distance to the old ones (`Face.position_at` → `distance_to`), not by the volume alone.
- **OCCT 8's names in `OCP`**, for code written against an older example (upstream's
  `docs/migrations/migrating-to-opencascade-8.md` has the full table): the collections moved to `OCP.collections`,
  named by their template arguments (`TopTools_ListOfShape` → `List_TopoDS_Shape`, `TopTools_IndexedMapOfShape` →
  `IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher`), the downcasts lost their `_s` (`TopoDS.Face_s` → `TopoDS.Face`),
  `Bnd_Box.Get()` is unbound (`CornerMin()` / `CornerMax()`). build123d 0.12 removed its deprecated API
  (`Location.to_tuple()` → `tuple(loc)`).
- **OCCT 8.0.1's two known defects.** `BRepBndLib.AddOptimal` - what build123d's `bounding_box()` uses - can return a
  box smaller than a B-spline shape (OCCT #1604): the repo's boxes were checked against OCCT 7.9's when OCCT 8 came in,
  but a vendor STEP or spline part added since is unchecked - compare its `bounding_box()` with cad's
  support-plane box (`cadgen.engine.kernel.bounds.optimal_box(shape.wrapped)`, a private name) before a box of it
  becomes a lock or an envelope. A sweep along a binormal guide curve distorts its profile while still reporting valid
  (OCCT #1681): `cad/` has no sweep; a model that needs a guided one checks its volume against the profile's.
- The private names this repo leans on — re-check them on a bump: `cadgen.engine.model._CONTEXT.build` (a build in
  progress on this thread: `lib/models.py building()`), a model's `.function` / `.outputs` (`Output.kind` / `.out`) /
  `__xbf_model__` and `Model.build` (`lib/models.py`, `tests/conftest.py`, `test_parts_convention.py`) — and that
  `./cadtool gen assemblies/arm.py` still lists every part as `…, a child` (a `building()` that silently read False
  would build every child as a raw body: the arm would still build, its children's STEPs unwritten).
- `cad doctor` runs only from `./cadtool server` (`./cadtool cad doctor` is refused, `./cadtool doctor` changes nothing):
  with no live installation on the machine it registers the environment it runs in as the machine's
  (`installations/` in cad's state directory: what a client starts the server from) and starts the server. That is why
  `cad-server` stays out of the venv: a worktree's venv registered there goes with the worktree. Run from the uv tool it
  is safe. A second reason to keep it there: uvx would leave it in uv's cache, which `uv cache prune` empties.
- Never run `cad setup` / `update` / `uninstall` here, the viewer's Update button (or its Automatic updates item) or
  the `curl … install.sh | sh` line
  upstream's README and migration guide lead with, which is `cad setup`: nothing asks, it takes every agent app on the
  machine to the newest release whatever `pyproject.toml` pins, registers the installation it runs in, and replaces this
  plugin with `text-to-cad@text-to-cad`, a local catalog in cad's state folder the committed `.claude/settings.json`
  cannot name. The plugin's skills print that line when `cad` is missing, and build with `cad run`, which runs in an
  environment of its own (0.11: `uv run` with cad at the machine's release) instead of this venv: inside this project `./cadtool` overrides both (`cad/AGENTS.md`
  Running things). `./cadtool cad` refuses all of these. The plugin moves only with a bump, by the steps of "After a pull
  …". `text-to-cad@earthtojake` from GitHub stays upstream's documented manual install.
- `claude plugin marketplace update` alone makes no plugin cache directory (it refreshes only the clone `~/.claude/plugins/marketplaces/earthtojake/`), so on a branch that bumps the pin before the plugin update
  `./cadtool doctor` warns `no plugin cache for cad <ver>` and reports a MISMATCH against the old one:
  `CAD_PLUGIN=~/.claude/plugins/marketplaces/earthtojake ./cadtool doctor` checks the new pin against that clone, the
  installed entries staying a MISMATCH until the update. Update the plugin in BOTH
  scopes, naming each (`claude plugin update text-to-cad@earthtojake --scope user`, then `--scope project`: with no
  `--scope` it auto-detects one - from the repo root, the project's, leaving the user entry behind), and restart Claude
  Code (its first start after an update fetches the new `cad-mcp`; on a slow connection start once with
  `MCP_TIMEOUT=300000 claude`). A session first started in `cad/` records a third
  entry (project scope, `projectPath` = `cad/`) that no `update` reaches: run from `cad/`, `update --scope project`
  answers for the repo root's entry ("already at the latest version") while `install` there still reports the old one.
  Copy the root entry's `version`, `installPath`, `gitCommitSha` and `lastUpdated` into it by hand (back the file up
  first); `claude plugin list` run from `cad/` then shows the new version for all three entries. `./cadtool doctor`
  reads every entry.
- **A machine on cadgen 0.8 moves to cad (0.10 or later: the pin) once, from a plain terminal with every Claude
  session closed**, not by "After a pull …": `cad doctor` moves 0.8's documents and Recents out of its state folder, which a live 0.8 is still using, and
  the plugin's 0.8 server outlives its session. In order: `claude plugin marketplace update earthtojake` (its clone's
  `VERSION` must be the pin, else stop); close every session; back up 0.8's state folder (`~/.local/state/cadgen`, on the
  Mac `~/Library/Application Support/cadgen`); stop every 0.8 engine BEFORE pulling, since the pulled `cadtool` cannot
  reach one: `./cadtool stop` in each checkout, then `~/.local/bin/cadgen stop` for the plugin's, and `pgrep -af cadgen`
  must print nothing; `git pull --ff-only`; `./cadtool setup`; the plugin update of "After a pull …", third entry
  included; `./cadtool doctor` clean; restart Claude Code; `./cadtool gen assemblies/arm.py`, cold once, as the cache does
  not move. Once no 0.8 worktree is left: `"$(uv tool dir --bin)/cad" doctor --prune` (it lists what it removes and
  asks; it takes `~/.local/bin/cadgen`), delete `~/.cache/cadgen` and the backup. Keep `~/.local/state/cadgen/settings.json`: its stored off covers an old
  checkout run later.
