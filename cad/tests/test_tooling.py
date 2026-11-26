"""The local tooling stays healthy: the venv holds the cad library and the OCP kernel pyproject.toml pins
(and only ONE OCP distribution) and no cad-server, and ./cadtool inspect (tools/step_facts.py) agrees with the kernel;
the retired inspect verbs teach the new syntax, cad's telemetry, its own updates and its plugin updates stay off,
./cadtool doctor names what moves the machine off the pin, no `cadgen` import, no
CADGEN_ name and no second pin is left, the retired cadtool verbs say what replaced them and its snapshot / export name cad's document type,
./cadtool refuses what would move the machine off the pin and its stop ends only this venv's workers,
the two .claude/settings.json are the same and take the plugin from upstream's `latest` branch, no tracked CLAUDE.md
hides an AGENTS.md, every tracked name is lowercase."""
import json
import os
import pathlib
import re
import subprocess
import tomllib
from importlib import metadata

import pytest

from lib.reference import CAD_DIR, describe, path_of
from tools import step_facts, stop_workers


def pinned(package: str) -> str:
    """The `==` pin of `package` in cad/pyproject.toml (extras ignored)."""
    dependencies = tomllib.loads((CAD_DIR / "pyproject.toml").read_text())["project"]["dependencies"]
    for spec in dependencies:
        match = re.fullmatch(rf"{re.escape(package)}(?:\[[^\]]*\])?==(?P<pin>\S+)", spec)
        if match:
            return match["pin"]
    raise AssertionError(f"{package} is not pinned with == in pyproject.toml")


def test_installed_cad_is_the_pinned_one():
    # The machine's cad (its uv tool) and the plugin's .claude-plugin/plugin.json carry the same pin; ./cadtool doctor
    # checks those.
    assert metadata.version("cad") == pinned("cad"), "run ./cadtool setup"


def test_the_server_stays_out_of_the_venv():
    """cad-server, an empty alias of cad since 0.11.12, is no dependency here: the toolchain's one pin is cad's, and
    the machine's server runs from the cad uv tool, never from a venv's own `cad` command, whose server would go with
    its worktree (docs/toolchain.md Pins)."""
    with pytest.raises(metadata.PackageNotFoundError):
        metadata.version("cad-server")


def test_one_ocp_distribution_and_it_is_complete():
    """cadquery-ocp (VTK) and cadquery-ocp-novtk own the same OCP/ files: installing both, or letting
    uv remove one, leaves a kernel that is 'installed' but gutted (2026-09-18, the cadgen 0.6.5 bump)."""
    assert metadata.version("cadquery-ocp-novtk") == pinned("cadquery-ocp-novtk")
    with pytest.raises(metadata.PackageNotFoundError):
        metadata.version("cadquery-ocp")
    files = metadata.files("cadquery-ocp-novtk")
    assert files is not None, "cadquery-ocp-novtk lists no files (no RECORD)"
    missing = [str(f) for f in files if not pathlib.Path(f.locate()).exists()]
    assert not missing, f"{len(missing)} OCP files missing - uv sync --reinstall-package cadquery-ocp-novtk"


def test_retired_inspect_verbs_teach_the_new_syntax(capsys):
    assert step_facts.main(["refs", "x.step", "--facts"]) == 2
    assert "cadtool inspect <file.step>" in capsys.readouterr().err


@pytest.mark.slow
def test_step_facts_agree_with_the_kernel():
    path = path_of("gripper_j3_connector")
    report, expected = step_facts.facts(path), describe(path)
    totals = report["totals"]
    assert totals["solids"] == expected["solids"]
    assert totals["volume"] == pytest.approx(expected["solid_volume"], abs=1e-3)
    assert totals["bbox_min"] == pytest.approx(expected["bbox_min"], abs=1e-3)
    assert totals["bbox_size"] == pytest.approx(expected["bbox_size"], abs=1e-3)
    assert [p["ref"] for p in step_facts.planes(path)], "a printed part has planar faces"
    assert step_facts.diff(path, path) == []


def stored_before_the_server_starts(source: str, command: str) -> bool:
    """Whether cadtool's install_server() runs `command` before `cad server start` - the one thing that starts the
    server from cad 0.11.12, which goes by the choices stored then."""
    install = re.search(r"^install_server\(\) \{(?P<body>.*?)^\}$", source, re.MULTILINE | re.DOTALL)
    assert install, "cadtool has no install_server()"
    stored, start = install["body"].find(f'"$cad" {command}'), install["body"].find('"$cad" server start')
    return -1 < stored < start


def test_cad_telemetry_stays_off():
    """cad sends usage stats and crash reports by default once a cad command has shown its notice; this repo keeps
    them off (docs/toolchain.md Pins). In the environment of every cadtool command (and VS Code's, `.env`) nothing shows
    the notice or sends; `./cadtool server` stores the "off" the server goes by, before it starts the server. That the
    choice IS stored is per machine: `./cadtool doctor` reads it and fails without it."""
    source = (CAD_DIR / "cadtool").read_text()
    assert re.search(r"^export CAD_TELEMETRY=0$", source, re.MULTILINE), "cadtool no longer exports it"
    assert stored_before_the_server_starts(source, "telemetry off"), "./cadtool server must store the off first"
    assert re.search(r"^    install_server ;;$", source, re.MULTILINE), "`server` / `setup` no longer run it"
    assert "CAD_TELEMETRY=0" in (CAD_DIR / ".env").read_text().splitlines()


@pytest.mark.parametrize("switch", ["update auto", "plugin update auto"])
def test_cad_updates_stay_off(switch):
    """From cad 0.11.12 cad installs every newer 0.x release by itself, on by default (`update auto`), and its server
    can run `cad plugin install` in every agent app each time cad got newer (`plugin update auto`): either takes
    the machine off the pin (docs/toolchain.md Pins). `./cadtool server` stores both off before it starts the server,
    as it does the telemetry off; `./cadtool doctor` reads each back."""
    source = (CAD_DIR / "cadtool").read_text()
    assert stored_before_the_server_starts(source, f"{switch} off"), f"./cadtool server must store `{switch} off` first"
    assert f'"$(cad_bin)" {switch} status' in source, f"./cadtool doctor no longer reads `{switch}`"


def cadtool_function(name: str, *args: str, env: dict[str, str] | None = None, stdin: str = "") -> str:
    """What cadtool's function `name` prints, loaded alone with `eval` (macOS's bash 3.2 sources nothing from a
    process substitution)."""
    load = f'f=$(sed -n "/^{name}() {{/,/^}}/p" cadtool); eval "$f"; {name} "$@"'
    proc = subprocess.run(["bash", "-c", load, "_", *args], cwd=CAD_DIR, env={**os.environ, **(env or {})},
                          input=stdin, capture_output=True, text=True, check=True)
    return proc.stdout.strip()


def test_doctor_names_what_moves_the_machine(tmp_path):
    """What leaves the pin says no error of its own: a cad updated by itself or by hand (the uv tool's release), the
    cad-server tool of 0.11.11 and earlier left beside the cad one, a server of another release or one run from a
    venv's own `cad` (gone with its worktree), and upstream's installer, which puts text-to-cad@text-to-cad (or its
    older cad@text-to-cad) beside this plugin (docs/toolchain.md Gotchas). `./cadtool doctor` reads each; its readers
    are checked here on files of their shape: the tool's metadata and receipt (`UV_TOOL_DIR`, as uv reads it), the
    plugin's manifest, `cad server status --json` and the installed plugins."""
    source = (CAD_DIR / "cadtool").read_text()
    doctor = re.search(r"^  doctor\)(?P<body>.*?)^  setup\)", source, re.MULTILINE | re.DOTALL)
    assert doctor, "cadtool has no doctor verb"
    for check in ('said "the server running"', "the server's environment", "$(old_tool ||", "$(other_plugins ||",
                  "$(plugin_release "):
        assert check in doctor["body"], f"./cadtool doctor no longer checks {check}"
    tools = tmp_path / "tools"
    (tools / "cad" / "lib" / "python3.14" / "site-packages" / "cad-9.9.9.dist-info").mkdir(parents=True)
    (tools / "cad" / "lib" / "python3.14" / "site-packages" / "cad_mcp").mkdir()
    (tools / "cad-server").mkdir()
    (tools / "cad-server" / "uv-receipt.toml").write_text(
        '[tool]\nrequirements = [{ name = "cad-server", specifier = "==0.11.11" }]\n')
    assert cadtool_function("server_release", env={"UV_TOOL_DIR": str(tools)}) == "9.9.9"
    assert cadtool_function("old_tool", env={"UV_TOOL_DIR": str(tools)}) == "0.11.11"
    assert cadtool_function("server_release", env={"UV_TOOL_DIR": str(tmp_path / "none")}) == ""
    (tmp_path / ".claude-plugin").mkdir()
    (tmp_path / ".claude-plugin" / "plugin.json").write_text(json.dumps({"name": "text-to-cad", "version": "9.9.9"},
                                                                        indent=2))
    assert cadtool_function("plugin_release", str(tmp_path)) == "9.9.9"
    status = json.dumps({"ok": True, "pid": 1, "version": "9.9.9", "prefix": str(tools / "cad"), "workers": []})
    assert cadtool_function("running", stdin=status) == f"9.9.9 {os.path.realpath(tools / 'cad')}"
    refused = json.dumps({"ok": False, "error": "the cad server is not running: run `cad server start`",
                          "reason": "failed"})
    assert cadtool_function("running", stdin=refused) == "- the cad server is not running: run `cad server start`"
    plugins = tmp_path / "installed_plugins.json"
    plugins.write_text(json.dumps({"version": 2, "plugins": {name: [{}] for name in (
        "text-to-cad@earthtojake", "text-to-cad@text-to-cad", "cad@text-to-cad", "clangd-lsp@claude-plugins-official")}}))
    assert cadtool_function("other_plugins", str(plugins)) == "cad@text-to-cad text-to-cad@text-to-cad"


@pytest.mark.parametrize("verb", ["parts", "why", "compile", "store", "daemon", "cadgen"])
def test_retired_cadtool_verbs_say_what_replaced_them(verb):
    """A retired verb exits 2 and says what replaced it, before anything of cad runs (`parts`: cad 0.11 retired the
    plugin's step-parts skill; step.parts is reached by hand, vendor/AGENTS.md Recipe D)."""
    proc = subprocess.run(["./cadtool", verb], cwd=CAD_DIR, capture_output=True, text=True, check=False)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert f"cadtool {verb}: retired - " in proc.stderr


@pytest.mark.parametrize(("source", "kind"), [
    ("assemblies/arm.step", "step"), ("vendor/x.STP", "step"), ("print/base.stl", "stl"), ("a.glb", "glb"),
    ("a.3mf", "threemf"), ("a.xbf", "xbf"), ("a.dxf", "dxf"), ("robot/arm.urdf", "urdf"), ("robot/arm.srdf", "srdf"),
    ("009e0c63-bcc4-5f1c-9e2c-f9146bfc6a87", "model")])
def test_cadtool_names_cads_document_type(source, kind):
    """cad 0.11's verbs are `cad <document type> <operation> FILE`: `./cadtool snapshot` / `export` take the type from
    the file's suffix, and anything else is a document id, `model`'s. The function is loaded with `eval`: macOS's bash
    3.2 sources nothing from a process substitution (`source <(…)`), so the test would fail there alone."""
    load = 'f=$(sed -n "/^doc_type() {/,/^}/p" cadtool); eval "$f"; doc_type "$1"'
    proc = subprocess.run(["bash", "-c", load, "_", source], cwd=CAD_DIR, capture_output=True, text=True, check=True)
    assert proc.stdout.strip() == kind


@pytest.mark.parametrize("verb", [["validate", "robot/arm.sdf"], ["snapshot", "robot/arm.sdf", "x.png"],
                                  ["export", "robot/arm.SDF", "glb"], ["viewer", "robot/arm.sdf"]])
def test_cadtool_refuses_an_sdf(verb):
    """cad 0.11.15 removed its SDF document type (`cad sdf` is an unknown command): `./cadtool` refuses an `.sdf`
    itself, before anything of cad runs, and names the check the repo keeps (`tools/robot/derive.py --check`)."""
    proc = subprocess.run(["./cadtool", *verb], cwd=CAD_DIR, capture_output=True, text=True, check=False)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "no SDF reader" in proc.stderr and "derive.py --check" in proc.stderr


def code_files() -> list[pathlib.Path]:
    """The tracked files that are run: cadtool, the environment files, the hooks, the workflows and all Python."""
    root = CAD_DIR.parent
    names = [n for n in tracked_names() if n.startswith((".github/", ".githooks/", ".claude/hooks/", "cad/.env"))
             or n == "cad/cadtool" or n == "cad/pyproject.toml" or (n.startswith("cad/") and n.endswith(".py"))]
    return [root / n for n in names if not n.startswith("cad/reference/")]


def test_no_cadgen_import_is_left():
    """From cad 0.11.1 `cadgen` is the library's deprecated alias: importing it warns CadDeprecationWarning, an error
    under pytest (pyproject.toml filterwarnings) and a line in every build a model's worker runs - so the code imports
    `cad` (docs/toolchain.md Pins)."""
    imports = re.compile(r"^\s*(?:from|import)\s+cadgen\b", re.MULTILINE)
    left = [str(path) for path in code_files() if path.suffix == ".py" and imports.search(path.read_text())]
    assert not left, f"import cad, not cadgen: {left}"


def test_no_cadgen_environment_name_is_left():
    """cad reads no CADGEN_ name and says nothing of one, so one left in the code is a switch silently off: unrenamed,
    `./cadtool gen --force` forces nothing and the environment's telemetry off is gone."""
    left = [str(path) for path in code_files() if path.name != pathlib.Path(__file__).name
            and "CADGEN_" in path.read_text()]
    assert not left, f"cad reads CAD_* names only: {left}"


def test_the_pin_has_one_home():
    """cad/pyproject.toml's `cad[build123d]==` is the toolchain's one pin: cadtool and CI read it, never restate it."""
    second = re.compile(r"\bcad(?:-server|-mcp)?(?:\[[^\]]*\])?==[0-9]")
    for path in (CAD_DIR / "cadtool", CAD_DIR.parent / ".github" / "workflows" / "ci.yml"):
        assert not second.search(path.read_text()), f"{path.name} restates the pin"


@pytest.mark.parametrize("verb", ["setup", "update", "plugin", "run"])
def test_cadtool_refuses_what_moves_the_machine(verb):
    """`cad update` takes the machine past the pin, `cad setup` (kept for 0.11.11 and earlier's updater) runs upstream's
    install command and `cad plugin install` (`update` its earlier name) puts the newest plugin in every agent app, and
    `run` builds outside this venv (docs/toolchain.md Gotchas). Refused before anything of cad runs."""
    proc = subprocess.run(["./cadtool", "cad", verb], cwd=CAD_DIR, capture_output=True, text=True, check=False)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert f"'cad {verb}' is not run here" in proc.stderr


def test_stop_ends_only_this_venvs_idle_workers():
    """The server is shared: `./cadtool stop` must leave another session's workers, and this venv's busy one, alone."""
    ours, theirs = "/work/cad/.venv", "/other/cad/.venv"
    status = {"workers": [{"pid": 1, "python": f"{ours}/bin/python", "alive": True, "busy": False},
                          {"pid": 2, "python": f"{ours}/bin/python", "alive": True, "busy": True},
                          {"pid": 3, "python": f"{theirs}/bin/python", "alive": True, "busy": False},
                          {"pid": 4, "python": f"{ours}/bin/python", "alive": False, "busy": False},
                          {"pid": 5, "alive": True, "busy": False}]}  # a worker of the server's own environment
    assert stop_workers.split(status, ours) == ([1], [2])
    assert stop_workers.split({"workers": None}, ours) == ([], [])
    # `cad server status --json` with no server running is an error document: no worker to end, and no failure;
    # one of another release (which a cad command does not talk to) is said and fails
    assert stop_workers.refusal({"ok": True, **status}) is None
    for answer, code in (({"ok": False, "error": "the cad server is not running: run `cad server start`"}, 0),
                         ({"ok": False, "error": "cad here is 9.9.9; the running server is 9.9.8: ..."}, 1), ({}, 1)):
        refused = stop_workers.refusal(answer)
        assert refused is not None and refused[1] == code, answer


def test_claude_settings_are_the_same_in_the_root_and_in_cad():
    """A Claude Code session reads the shared .claude/settings.json of the directory it starts in only (it is not
    inherited like AGENTS.md), so cad/.claude/settings.json is a copy of the root one: the ruff and ty hooks and the
    text-to-cad@earthtojake plugin, whether the session starts in the repo root or in cad/."""
    root = json.loads((CAD_DIR.parent / ".claude" / "settings.json").read_text())
    cad = json.loads((CAD_DIR / ".claude" / "settings.json").read_text())
    assert cad == root, "the two .claude/settings.json differ - edit the root one and copy it to cad/.claude/"


def test_the_plugin_comes_from_upstreams_latest_branch():
    """Upstream's `latest` branch carries the plugin alone, moved to each release once it is on PyPI; its `main` is where
    development happens, ahead of the release between two (docs/toolchain.md Pins)."""
    root = json.loads((CAD_DIR.parent / ".claude" / "settings.json").read_text())
    source = root["extraKnownMarketplaces"]["earthtojake"]["source"]
    assert source.get("ref") == "latest", source
    assert root["enabledPlugins"] == {"text-to-cad@earthtojake": True}


def tracked_names() -> list[str]:
    """Every path git tracks in the repo, relative to its root."""
    return subprocess.run(["git", "-C", str(CAD_DIR.parent), "ls-files", "-z"], capture_output=True, text=True,
                          check=True).stdout.split("\0")[:-1]


def test_no_claude_md_hides_the_agents_md():
    """Claude Code reads a folder's AGENTS.md only when it finds no CLAUDE.md there (a CLAUDE.local.md counts as one),
    so a tracked CLAUDE.md would silently stop the AGENTS.md beside it loading (root AGENTS.md "Docs")."""
    hiding = [n for n in tracked_names() if n.rpartition("/")[2].lower() in ("claude.md", "claude.local.md")]
    assert not hiding, f"the agent instructions are AGENTS.md files - move these into the AGENTS.md there: {hiding}"


def test_every_tracked_name_is_lowercase():
    """On Linux git and the *.step / *.stl rules are case-sensitive, the Mac's filesystem is not: git there runs with
    core.ignorecase=true, so a case-only rename goes unnoticed on the Mac and arrives on Linux as a second file.
    Every tracked path is lowercase except the README / AGENTS.md docs (root AGENTS.md "Git workflow")."""
    names = tracked_names()
    checked = [n.rpartition("/")[0] if n.rpartition("/")[2] in ("README", "README.md", "AGENTS.md") else n for n in names]
    upper = [n for n, c in zip(names, checked, strict=True) if c != c.lower()]
    assert not upper, f"uppercase in tracked names (rename with git mv -f): {upper}"
