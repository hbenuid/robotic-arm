# robotic-arm

A six-joint desktop robot arm with a parallel gripper: its CAD, and the software that drives its MKS SERVO42D stepper
motors over CAN (a CANable / slcan-compatible USB adapter; macOS, Linux and Windows).

The repo splits CAD from everything else: `cad/` (the build123d CAD, its own uv project) and `software/` —
`software/control/` (the motor-control CLI, its own uv project) and `software/firmware/` (microcontroller firmware).
The root holds only repo-wide files (docs, `.github/`, `.githooks/`, `.claude/`, git config, the VS Code workspace).

The software will be worked on later: the current work is the CAD, and `software/` stays as it is until then. It stays
in this repo on purpose — when it is picked up it needs the CAD's robot description (`cad/robot/arm.urdf`, the joint
table, the drive ratios) to agree with `software/control/src/config.py`, and one repo keeps both sides in one commit.

## Git workflow
- Work on a branch — `cad/<topic>` for CAD work, `<area>/<topic>` otherwise — and push the **branch**.
- **Every branch reaches `main` through a pull request.** When the branch is done (committed, the local suite green,
  pushed), open one: `gh pr create --base main --head <branch> --title "<what it did, one line>" --body "<why, what
  changed, the checks run>"` (the commits stay the record: the body summarises, it does not replace them).
  `gh pr list` is then the list of branches waiting for `main`, each with its diff against today's `main` and a
  warning when it conflicts. If `main` moves under an open branch, merge `main` into the branch (`git fetch`,
  `git merge origin/main`, fix conflicts, re-run the suite, push): the pull request follows the branch.
- The pull request is merged only when the user says so, and always as a **merge commit** — never squash, rebase or
  fast-forward (the repo's GitHub settings allow only merge commits): `gh pr merge <n> --merge --subject "Merge
  <branch>: <what it did, one line>"`, then `git pull --ff-only` in the main checkout (on `main`, clean). So `main`
  reads one entry per branch -
  `git log --first-parent main`, or a branch graph (VS Code's Source Control Graph, `git log --graph`) that draws
  each branch as a side line into its merge (GitHub Desktop's History is a flat list: it shows no grouping, a merge's
  row there shows the branch's whole change); the older history was fast-forwarded and stays linear. Never commit to
  or push `main` directly, never `git merge` into `main` by hand and never `git push origin <branch>:main` (an
  approved plan that says "commit + push" means the branch; opening its pull request is part of finishing a branch,
  merging it is not).
- **The commit message is the record** — there is no CHANGELOG file. Subject `<area>: what changed` (`cad:`, `docs:`,
  `tooling:` …); body: why, what it replaces or removes, and every measured number that changed (totals, masses,
  fits, lock values); one logical change per commit. History is `git log` (`--grep`, `-- <path>`, `-S <CONSTANT>`,
  `-p` on a lock file). The entries of the retired `CHANGELOG.md` (to 2026-09-25) are **git notes** on the commits
  they describe — `git log` prints them under the message; `./cadtool setup` makes a machine fetch them.
- Case matters: git and the `*.step` / `*.stl` rules are case-sensitive. `.gitignore` also ignores `*.STEP` / `*.STP` /
  `*.STL` and `.gitattributes` LFS-tracks `*.STEP` / `*.STL`, but keep every file in the tree lowercase (README /
  CLAUDE.md excepted) — the tests and tools assume it, and `cad/tests/test_tooling.py` checks. On the Mac git runs with
  `core.ignorecase=true`: a case-only rename needs `git mv -f`.

## Two development machines
Worked on from a **Fedora Linux PC and an arm64 Mac**; git is the only sync channel, venvs / plugin / caches are
per machine. In `cad/` no generated STEP is committed (its bytes differ per machine; each machine builds its
own) — rules in `cad/CLAUDE.md` "Two machines". Scripts must run on macOS's bash 3.2 without GNU coreutils.

## The SolidWorks inputs are in the repo; the raw exports are not
The CAD reads only committed inputs (Git LFS): `cad/reference/` (the renamed SolidWorks / CadQuery exports,
`placements.json`, `manifest.json`) and `cad/vendor/`. The raw exports they were derived from are **never committed**
and need not exist on a machine, and they have no fixed place: the tools that read one take its path. A new export
the user hands over is read where the user put it — never moved, renamed or copied elsewhere on the machine, and no
folder is made for it outside the repo — and goes through Recipe E in `cad/reference/CLAUDE.md`.

## Toolchain
- Always use `uv` — never `pip install` directly. `uv add <pkg>` for new deps;
  commit `pyproject.toml` and `uv.lock`. The motor-control project is `software/control/`: run its uv
  commands there (the repo root is no uv project).
- Lint with ruff (a locked dev dependency in both uv projects, config in each `pyproject.toml` `[tool.ruff]`:
  ruff's default set + `E F I UP B SIM` in full, the ignores commented):
  `uv run ruff check` in `software/control/`, `./cadtool lint` in `cad/`. Both are clean; keep them clean.
  Every `zip()` takes `strict=` (`B905`): `True` where the inputs must pair up — a length mismatch raises instead of
  silently truncating —, `False` only where a mismatch is expected and handled.
  Lint only — `ruff format` is not adopted (it would re-flow `cad/`'s hand-aligned tables).
  It runs by itself at three points: a Claude Code PostToolUse hook (`.claude/hooks/ruff-check.sh`) reports
  findings on every `.py` file Claude edits — fix them in the same turn. It is registered twice, in
  `.claude/settings.json` AND `cad/.claude/settings.json`: a session reads the shared settings file of the
  directory it starts in only (not inherited like CLAUDE.md), so the cad/ file is a copy of the root one (edit the
  root one, copy it; `cad/tests/test_tooling.py` checks);
  the git pre-commit hook (`.githooks/pre-commit`, installed per machine by `./cadtool setup`) blocks a commit
  with findings in the staged files; VS Code fixes / sorts imports on save (workspace settings, Ruff extension).
  `F401` is never auto-fixed (`unfixable`): an import written before its first use must survive a save.

## CI (GitHub Actions)
`.github/workflows/ci.yml` runs **only by hand** (Actions tab → CI → Run workflow, or
`gh workflow run ci.yml --ref <branch>`; `gh run watch` follows it): a fresh `ubuntu-24.04` runner clones the repo,
pulls the LFS objects, lints both projects, imports the CLI, runs `cad/`'s fast lane and slow tests and `./cadtool
gen`s the arm (~13 min, from the private repo's monthly Actions minutes). It is the **clean-clone check** neither
machine can give — no `~/.cache/cadgen`, no raw exports, no generated STEPs — so a test or model that quietly needs
a file outside git fails there first. Everyday pushes do not need it (the pre-commit hook and the local suite cover
them); run it on the branch before its pull request is merged after: a cadgen / build123d / OCP bump, a new
SolidWorks or vendor export or vendor STEP, a change to `lib/reference.py` or to how parts read their inputs, or any
large change - on the branch as it will merge (with `main` merged in if `main` moved), its result noted on the pull
request (`gh pr comment <n>`). Suggest it then; never trigger it unasked. It checks and never writes (no hash gate,
no snapshots, no commits). The runner is a third machine (x86_64 Linux): the "no exact float equality" rule of
`cad/CLAUDE.md` "Two machines" holds for it too. The motor-control `software/control/tests/` stay out until they are
fixed.

## Software (`software/`)
- `software/control/` is the motor-control CLI, its own uv project. Its rules — running it, the load-bearing
  `sys.path` line, its known issues — are `software/control/CLAUDE.md`, which loads when you work there; setup and use
  are its `README.md`.
- `software/firmware/` holds all firmware, one subfolder per board, part of neither uv project
  (`software/firmware/README.md`).

## CAD (`cad/`)
- `cad/` is a **separate uv project** (Python 3.12, build123d) — the motor-control
  project (`software/control/`) never depends on it, and `launch.bat` never installs it. Never run
  CAD code with the motor-control venv.
- Work from `cad/` via `./cadtool …` (`./cadtool help` lists the commands). Start at `cad/CLAUDE.md` — the map and the
  rules for every folder; each folder's own CLAUDE.md (`parts/`, `assemblies/`, `robot/`, `lib/`, `tests/`, `tools/`,
  `reference/`, `vendor/`) loads when you work there.
- What the software has to match — the joints and links, the reductions (`CYCLOIDAL_RATIO`, `GT2_RATIO`,
  `FOREARM_ROLL_RATIO` in `cad/lib/params.py`, and which way round `config.py`'s `gear_ratio` reads), which motor sits
  on which joint — is `cad/robot/CLAUDE.md`; which CAN id drives which joint is
  unconfirmed (`cad/docs/open_issues.md`).

## Docs
- No CHANGELOG and no "Last updated" lines: git dates every change (Git workflow above). Docs describe the current
  state and the reasons for it, never a dated history.
- `cad/docs/open_issues.md` is the ONE list of what is not settled (fit problems, estimates to confirm on
  hardware, unmodelled hardware, unconfirmed mappings): add a row when you flag something, remove it when you
  close it.
