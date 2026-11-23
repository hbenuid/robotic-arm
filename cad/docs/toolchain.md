# Toolchain — pins, upgrades, per-machine setup

Read this before bumping cad / build123d / OCP / Python, after a pull that changes `pyproject.toml` / `uv.lock`,
or when the kernel, the server or the plugin misbehaves. Everyday running is `cad/AGENTS.md` "Running things"; the
history of every bump is `git log -- pyproject.toml uv.lock` from `cad/` (the older bumps carry their CHANGELOG entry
as a git note).

## Pins
- **One pin, `"cad[build123d]==<ver>"` in `pyproject.toml`**, for the two things upstream releases together. One is
  `cad`, a single package (0.11.12 merged `cad-server` and `cad-mcp` into it; both names are empty aliases now): in
  the venv it is the library (its earlier name `cadgen` is a deprecated alias of the same modules, which warns
  `CadDeprecationWarning` on import from 0.11.1 - an error under pytest (Gotchas) -, so nothing here imports it:
  `test_tooling.py` checks), and on the machine the same package as a uv tool is the `cad` command and everything that
  runs (`./cadtool server` installs it from the pin on a uv-managed Python; `uv tool dir --bin` holds the command). The
  library brings its own `cad` command (and a `uv`, a dependency of it) into the venv: `cadtool` never runs that one
  (Gotchas). The other is the `text-to-cad@earthtojake` plugin (`~/.claude/plugins/cache/earthtojake/text-to-cad/<ver>/`),
  from upstream's `latest` branch (the committed `.claude/settings.json` names it: the plugin alone, moved to each
  release once it is on PyPI; upstream's `main` is where development happens, ahead of a release between two): its
  skills, and `.claude-plugin/plugin.json`, whose `version` is the release. It names no MCP server for Claude Code, so
  a session here has no `cad_*` tools and shows a model with `./cadtool viewer <file>` (`cad/AGENTS.md` Running
  things). Bump them together; `./cadtool doctor` reads each against the pin. `cadtool` restates the pin nowhere, and
  `cad-server` is never a dependency here (`test_tooling.py` holds both). `claude plugin install` / `update` take the
  marketplace's newest release, so update the plugin WITH a bump here, not before: its skills describe the release they
  came with. Upstream can release again between a bump and its plugin update, and the plugin then lands ahead of the
  pin: after `claude plugin marketplace update`, the clone's `.claude-plugin/plugin.json` `version` must be this pin -
  if it moved, bump again first.
  **The machine's cad comes before the plugin, always:** the skills teach their release's `cad` verbs, and a skill that
  finds `cad` missing or older than 0.11.12 tells the agent to run `uv tool install cad` - the newest release, whatever
  the pin -, which nothing here runs (Gotchas): `./cadtool server` installs it, its choices stored off first.
  The server reads `api.texttocad.dev/v1/releases` at most once an hour (one anonymous request; `CAD_UPDATE_CHECK=0` in
  the server's environment turns it off) to decide its own update, which stays off here (below); a release it names is
  a bump ("After a pull …" below).
- **One cad server per computer.** Every venv, worktree and session, and the plugin, use one server — the engine, warm
  workers, the documents they made, the CAD Viewer's page. From 0.11.12 nothing starts it but `cad server start` (no
  timer stops it): a client that finds none says `the cad server is not running: run cad server start`, so `./cadtool
  gen` and every `cadtool` verb that needs it start it first, from the uv tool (`need_server`; a start joins a server of
  its own release, replaces an older release's and joins a newer one's, which `cadtool` refuses). The server runs in
  the environment of the `cad` that started it. A model run
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
  cap or on memory pressure (no idle timer). A BUSY worker is never retired, but from 0.11.14 one past half the RAM (at
  least 4 GiB) at two of the server's rounds running is killed, and its build fails as out of memory - a build in native
  code cannot stop itself. Those caps (`CAD_ENGINE_MEMORY_MB`, `CAD_ENGINE_WORKERS`, `CAD_ENGINE_WORKER_RSS_MB`,
  `CAD_ENGINE_WORKER_CEILING_MB`) are read once, from the environment of whichever client started the server. pytest is
  outside all of it: its workers are plain processes. `./cadtool stop` ends this venv's idle workers, no other session's
  (`tools/stop_workers.py`: `cad server status --json` names each worker's Python, the server clears a dead idle one at
  its next grant): do it when leaving a worktree. `./cadtool stop --device` (`cad server stop`) stops the server for every
  session on the machine, whatever its release.
  The server serves on port 3245, the next free port if that is held, and keeps it. Each document is cached as last
  built in `~/.cache/cad/documents` (kept until deleted: `./cadtool cad delete <id> --yes`), its id made from the script's path
  and function, so worktrees never collide.
- **cad's telemetry stays off.** cad sends usage stats and crash reports by default (a random install id; never file
  names, paths, contents or prompts; its viewer also counts which of its tools are used) from every process — the
  command, the server, the plugin's — once a `cad` command has shown its one-line notice. This repo keeps them off, two
  ways: `cadtool` and `.env` export `CAD_TELEMETRY=0`, so no command of theirs shows the notice or sends; and `./cadtool
  server` stores `cad telemetry off` in cad's state directory (`settings.json`: `~/.local/state/cad/` on Linux,
  `~/Library/Application Support/cad/` on the Mac) BEFORE it runs `cad server start`, and the server goes by the stored
  choice — its own environment is not a client's. The one exception is the model builds it answers, which it counts only where
  the caller's environment allows too (a build carries the caller's whole environment, cadtool's `CAD_TELEMETRY=0`
  included); everything else the server and the plugin's server do goes by the stored choice. The stored answer is per machine: `telemetry` → `choice: off` in that file
  is the check, and `./cadtool doctor` reads it with the environment's switch taken off (with the switch on, `cad
  telemetry status` says off whatever is stored). `test_tooling.py` holds the order. Two ways this fails silently on a
  bump: a renamed switch does nothing and says nothing, and a new state folder starts with no answer.
- **cad's own updates and its plugin updates stay off.** From 0.11.12 cad updates itself by default to every newer
  release of its major version - for 0.x, any 0.x, minor releases with their hard cutovers included (Gotchas) -: its
  server reads the feed about once an hour and, once nothing has run for a minute, stops, runs `uv tool install
  --upgrade "cad>=<this>,<1"` on its own uv tool and starts again, under every session; and where its plugin switch is
  on, the server then runs the plugin's install once (`cad plugin install`, in cad from 0.11.14; `cad plugin update` is
  its hidden 0.11.12 name: the newest plugin in every agent app it finds). Either takes the machine off the pin. `./cadtool server` stores both off beside the telemetry off, before
  `cad server start`: `cad update auto off` (`auto_update` → `on: false` in the same `settings.json`; 0.11.11's
  `updates` → `automatic: false` is read as off too, never written) and `cad plugin update auto off` (`plugin` →
  `auto_update: false`, off by default, turned on by the install command). `./cadtool doctor` reads each (`update auto:
  off`, `plugin update auto: off`) and fails without it. A typed `cad update` updates with the switch off, so
  `./cadtool cad` refuses it (Gotchas).
- **build123d 0.13.0 / OCP 8.0.1 (OCCT 8)**: cad 0.11's `build123d` extra requires `build123d>=0.13.0,<0.14` and
  `cadquery-ocp-novtk>=8.0,<8.1`; `pyproject.toml` pins the exact kernel (`cadquery-ocp-novtk==…`, the STEP bytes are
  per-kernel) and must never gain `cadquery-ocp`, the VTK build (see Gotchas below). OCCT 8.0.1's two known defects and
  what this repo checks for them: Gotchas below. Every part must read back from the BinTools VERSION_4 bytes cad
  keeps it as (its document cache, its STEP reader's wire): `test_part_survives_a_bintools_round_trip`.
- Python is pinned to **3.14** (`.python-version`, a uv-managed interpreter on every machine: `[tool.uv]
  python-preference`). cad requires it, and build123d and the OCP wheels allow nothing newer; bump only with the full
  suite green.

## After a pull that changes `pyproject.toml` / `uv.lock` (per-machine state git does not carry)
A new pin moves the server under every session on the machine, so close the others first. `./cadtool stop; ./cadtool
setup` (the stop ends this venv's workers where only the kernel moved, and refuses where the cad pin moved: setup then
stops the old release's server itself; setup also (re)installs the ruff git pre-commit hook: a stub `.git/hooks/pre-commit` → the committed
`.githooks/pre-commit`; makes `git fetch` bring the git notes; and runs `./cadtool server`, which stops the old
release's server, reinstalls the cad uv tool at the pin - taking the cad-server tool of 0.11.11 and earlier away -,
stores cad's telemetry, its own updates and its plugin updates off and starts the pin's server - in that order, and all
of it before the plugin update: Pins, above), `claude plugin marketplace update earthtojake` (its clone must be on
upstream's `latest` branch, `git -C ~/.claude/plugins/marketplaces/earthtojake branch --show-current`, and its
`.claude-plugin/plugin.json` `version` this pin: Pins, above; a clone still on `main` moves once: Gotchas below), then
from the repo root `claude plugin update text-to-cad@earthtojake --scope user` and `--scope project`
(`~/.claude/plugins/installed_plugins.json` must show the new version for every entry (a session first started in
`cad/` adds a third, which no `update` reaches: copy the new version into it by hand, Gotchas below); restart Claude
Code), then `./cadtool doctor` must be clean. No `CAD_PLUGIN` in a shell profile (it overrides the plugin detection).

## Testing a bump (a branch never moves the machine)
`./cadtool setup` on a bump's branch leaves the machine's server alone: a linked worktree only checks it, because `main`
and the other sessions are still on the old release. Test the new one beside it, on a server of its own, with every
command through a small wrapper outside the repo that exports, under one folder on disk (`/tmp` is RAM on the PC):
`CAD_STATE_DIR` and `CAD_CACHE_DIR` (with a state folder named, nothing moves out of the old release's),
`CAD_ENGINE_DIR` (its door, lock and port file: a server of its own, on the next free port, and its `server stop` stays
there), `UV_TOOL_DIR` and `UV_TOOL_BIN_DIR` (no second `cad` on the machine), with that bin folder first on `PATH`.
`./cadtool server` then works from the worktree, and the rest of `cadtool` follows; `CAD_PLUGIN=<a checkout of the new
release's plugin> ./cadtool doctor` checks the plugin's release, the installed entries staying the old one. Check these
names against upstream's source first: a renamed one does nothing, silently. A change of Python alone needs the same for
the document cache, whose key omits Python. The proof the geometry did not move: the hash gate in Gotchas below (every
model is a file ending in its build call under `parts/`, `assemblies/` and `robot/links/` - `parts/_templates/` holds
templates, not models; `./cadtool gen` stops at the first model that fails).

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
  argument (0.11: `cad snapshot` is `cad <type> snapshot`, the type the file's; 0.11.12: `cad status` / `cad stop` are
  `cad server status` / `cad server stop`, the short forms hidden and kept only for the skills of 0.11.11 and earlier,
  `cad doctor` reports and changes nothing, and nothing starts the server but `cad server start`; 0.11.15: the `sdf`
  document type is gone - `cad sdf validate` / `snapshot` and `cad.sdf`, with no alias -, so `./cadtool` refuses an
  `.sdf` itself and `robot/arm.sdf` is checked by `tools/robot/derive.py --check` alone), a retired decorator
  argument a `TypeError` — read the error as the spec (each move: upstream `docs/migrations/`) — and a retired environment
  variable says nothing at all. The library's renames keep the old spelling one release at least, as an alias that
  warns once with `CadDeprecationWarning` (0.11: `read_step` → `step.read`, `cad.geometry` → `cad.build123d`;
  0.11.1: `cadgen` → `cad`):
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
  support-plane box (`cad.engine.kernel.bounds.optimal_box(shape.wrapped)`, a private name) before a box of it
  becomes a lock or an envelope. A sweep along a binormal guide curve distorts its profile while still reporting valid
  (OCCT #1681): `cad/` has no sweep; a model that needs a guided one checks its volume against the profile's.
- build123d's `Mesher.write()` (lib3mf) leaves the process in the C locale, so every later `open()` / `read_text()`
  without an encoding in that process reads ASCII and fails on the first non-ASCII byte. Code that writes a 3MF
  in-process restores the locale round the write (`locale.setlocale(locale.LC_ALL)` before, set back after); no code
  in `cad/` writes one today - the print files are STEP (`parts/AGENTS.md` "Files to print").
- The private names this repo leans on — re-check them on a bump: `cad.engine.model._CONTEXT.build` (a build in
  progress on this thread: `lib/models.py building()`), a model's `.function` / `.outputs` (`Output.kind` / `.out`) /
  `__xbf_model__` and `Model.build` (`lib/models.py`, `tests/conftest.py`, `test_parts_convention.py`) — and that
  `./cadtool gen assemblies/arm.py` still lists every part as `…, a child` (a `building()` that silently read False
  would build every child as a raw body: the arm would still build, its children's STEPs unwritten).
- **The venv has a `cad` command of its own** (and a `uv`): from 0.11.12 the library is the whole package, so
  `.venv/bin/cad` is there, and on the `PATH` of everything `uv run` starts and of a shell with the venv activated.
  `cadtool` never runs it (`cad_bin` is the uv tool's). A server started from it (`cad server start` runs the server in
  the starting `cad`'s own environment) would serve every session from a worktree's venv and go with the worktree: the
  server's environment is the uv tool's (`./cadtool doctor` reads `cad server status --json`'s `prefix`; the way back is
  `./cadtool stop --device`, then any `./cadtool gen`). Not uvx either: it leaves cad in uv's cache, which `uv cache
  prune` empties.
- Never run `cad setup` / `update` / `plugin install` / `plugin update` here, `uv tool install cad` or `uv tool upgrade
  cad` without the pin, or the `curl … install.sh | sh` line upstream's README and install guide lead with: `cad update`
  (typed, it updates with automatic updates off) and an unpinned uv install take the machine to the newest release
  whatever `pyproject.toml` pins; `cad plugin install` (0.11.14; `cad plugin update` its hidden earlier name) puts the
  newest plugin in every agent app it finds and removes what earlier releases left there; and the hosted install
  command (plain `cad setup` too, kept only for the updater of 0.10.0 to 0.11.11) installs cad at the newest release
  where none is - from 0.11.14 it also brings a working one up to date with `cad update`, where a person ran it -, then
  hands over to `cad plugin install` and turns the plugin updates on. The plugin's skills say `uv
  tool install cad` when `cad` is missing or older than 0.11.12, and build with `cad run`, which runs in an
  environment of its own (`uv run` with cad at the machine's release) instead of this venv: inside this project
  `./cadtool` overrides both (`cad/AGENTS.md` Running things). `./cadtool cad` refuses `setup`, `update`, `plugin` and
  `run`. The plugin moves only with a bump, by the steps of "After a pull …". `text-to-cad@earthtojake` from GitHub
  stays upstream's documented manual install.
- **When one ran anyway**, the machine leaves the pin with no error. An update (typed, or automatic with the switch
  on) moves the uv tool itself - `uv-receipt.toml` then keeps a range, `cad>=<release>,<1`: `./cadtool doctor` says the
  tool's release and `./cadtool gen` refuses; the way back, every other session closed, is `./cadtool stop --device`
  and `./cadtool server` (it installs the pin again). `cad plugin install` leaves cad as it is (the hosted install
  command updates it too, as above) but puts the newest plugin in every agent app it finds (Claude Code's from upstream's `latest`, so an entry past the pin; earlier
  releases' installer used `text-to-cad@text-to-cad`, a local catalog in cad's state folder, and GitHub Copilot's
  skills in `~/.agents/skills`) and turns the plugin updates on: `./cadtool doctor` names the plugin entries, the
  switch and other text-to-cad plugins. The way back: `./cadtool server` (the switch off), the entries brought back to
  the pin as the third entry is (Gotchas below), another plugin uninstalled (`claude plugin uninstall
  text-to-cad@text-to-cad --scope user`, then `claude plugin marketplace remove text-to-cad` - from a folder outside
  the repo: below), the other apps' skills removed by hand. What 0.11.11 and earlier left in cad's state and cache
  folders (`installations/`, `releases/<ver>/`) is read by nothing now: `./cadtool cad cache prune` removes the
  release folders.
- **A machine's marketplace still on upstream's `main`** (its clone from before the committed `.claude/settings.json`
  named `latest`) stays there: `claude plugin marketplace update` keeps the branch it was added with, and no command
  changes it. Move it once, every other session closed, **from a folder outside the repo**: `claude plugin
  marketplace remove earthtojake` takes the marketplace out of the settings of every scope, the folder it runs in
  counting as a project - from the repo root it would strip `extraKnownMarketplaces` and `enabledPlugins` from the
  committed `.claude/settings.json` -, and uninstalls every entry of the plugin. Then `claude plugin marketplace add
  earthtojake/text-to-cad#latest` (it records `{"source": "github", "repo": "earthtojake/text-to-cad", "ref":
  "latest"}`, the committed settings' source, and declares it in `~/.claude/settings.json`, where an earlier add left
  the old source), `claude plugin install text-to-cad@earthtojake --scope user`, and from
  the repo root `--scope project` (and the third entry: below). Back up `~/.claude/plugins/installed_plugins.json`
  and `known_marketplaces.json` first.
- `claude plugin marketplace update` alone makes no plugin cache directory (it refreshes only the clone `~/.claude/plugins/marketplaces/earthtojake/`), so on a branch that bumps the pin before the plugin update
  `./cadtool doctor` warns `no plugin cache for cad <ver>` and reports a MISMATCH against the old one:
  `CAD_PLUGIN=~/.claude/plugins/marketplaces/earthtojake ./cadtool doctor` checks the new pin against that clone, the
  installed entries staying a MISMATCH until the update. Update the plugin in BOTH
  scopes, naming each (`claude plugin update text-to-cad@earthtojake --scope user`, then `--scope project`: with no
  `--scope` it auto-detects one - from the repo root, the project's, leaving the user entry behind), and restart Claude
  Code (a session loads the skills it starts with). A session first started in `cad/` records a third
  entry (project scope, `projectPath` = `cad/`) that no `update` reaches: run from `cad/`, `update --scope project`
  answers for the repo root's entry ("already at the latest version") while `install` there still reports the old one.
  Copy the root entry's `version`, `installPath`, `gitCommitSha` and `lastUpdated` into it by hand (back the file up
  first); `claude plugin list` run from `cad/` then shows the new version for all three entries. `./cadtool doctor`
  reads every entry.
- **A machine on cadgen 0.8 moves to cad (the pin) once, from a plain terminal with every Claude session closed**,
  not by "After a pull …": the plugin's 0.8 server outlives its session, and the pulled `cadtool` cannot reach a 0.8
  engine. Nothing of cad reads or moves 0.8's state or cache (from 0.11.12; the documents and Recents stay behind). In
  order: close every session; stop every 0.8 engine BEFORE pulling: `./cadtool stop` in each checkout, then
  `~/.local/bin/cadgen stop` for the plugin's, and `pgrep -af cadgen` must print nothing; `git pull --ff-only`;
  `./cadtool setup`; the marketplace moved to `latest` (Gotchas above) and the plugin update of "After a pull …", third
  entry included; `./cadtool doctor` clean; restart Claude Code; `./cadtool gen assemblies/arm.py`, cold once, as the
  cache does not move. Once no 0.8 worktree is left: delete `~/.local/bin/cadgen` and `~/.cache/cadgen` by hand
  (`./cadtool cad doctor` says where they are, with their size). Keep `~/.local/state/cadgen/settings.json`: its stored
  off covers an old checkout run later.
