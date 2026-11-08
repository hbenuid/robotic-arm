"""The local tooling stays healthy: the venv holds the cad library and the OCP kernel pyproject.toml pins
(and only ONE OCP distribution) and no cad-server, and ./cadtool inspect (tools/step_facts.py) agrees with the kernel;
the retired inspect verbs teach the new syntax, cad's telemetry and automatic updates stay off, no CADGEN_ name and no
second pin is left, the retired cadtool verbs say what replaced them and its snapshot / export name cad's document type,
./cadtool refuses what would move the machine off the pin and its stop ends only this venv's workers,
the two .claude/settings.json are the same, no tracked CLAUDE.md hides an AGENTS.md, every tracked name is lowercase."""
import json
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
    # The machine's cad-server and the plugin's claude.mcp.json carry the same pin; ./cadtool doctor checks those.
    assert metadata.version("cad") == pinned("cad"), "run ./cadtool setup"


def test_the_server_stays_out_of_the_venv():
    """cad-server in a venv lets that venv register as the machine's installation, and a worktree's goes with the
    worktree (docs/toolchain.md Pins): the server is a uv tool, one per machine."""
    with pytest.raises(metadata.PackageNotFoundError):
        metadata.version("cad-server")


def test_one_ocp_distribution_and_it_is_complete():
    """cadquery-ocp (VTK) and cadquery-ocp-novtk own the same OCP/ files: installing both, or letting
    uv remove one, leaves a kernel that is 'installed' but gutted (2026-09-18, the cadgen 0.6.5 bump)."""
    assert metadata.version("cadquery-ocp-novtk") == pinned("cadquery-ocp-novtk")
    with pytest.raises(metadata.PackageNotFoundError):
        metadata.version("cadquery-ocp")
    missing = [str(f) for f in metadata.files("cadquery-ocp-novtk") if not pathlib.Path(f.locate()).exists()]
    assert not missing, f"{len(missing)} OCP files missing - uv sync --reinstall-package cadquery-ocp-novtk"


def test_retired_inspect_verbs_teach_the_new_syntax(capsys):
    assert step_facts.main(["refs", "x.step", "--facts"]) == 2
    assert "cadtool inspect <file.step>" in capsys.readouterr().err


@pytest.mark.slow
def test_step_facts_agree_with_the_kernel():
    path = path_of("j3_coupler")
    report, expected = step_facts.facts(path), describe(path)
    totals = report["totals"]
    assert totals["solids"] == expected["solids"]
    assert totals["volume"] == pytest.approx(expected["solid_volume"], abs=1e-3)
    assert totals["bbox_min"] == pytest.approx(expected["bbox_min"], abs=1e-3)
    assert totals["bbox_size"] == pytest.approx(expected["bbox_size"], abs=1e-3)
    assert [p["ref"] for p in step_facts.planes(path)], "a printed coupler has planar faces"
    assert step_facts.diff(path, path) == []


def test_cad_telemetry_stays_off():
    """cad sends usage stats and crash reports by default once a cad command has shown its notice; this repo keeps
    them off (docs/toolchain.md Pins). In the environment of every cadtool command (and VS Code's, `.env`) nothing shows
    the notice or sends; `./cadtool server` stores the "off" the server and the plugin go by, before `cad doctor`
    starts the server. That the choice IS stored is per machine: `./cadtool doctor` reads it and fails without it."""
    source = (CAD_DIR / "cadtool").read_text()
    assert re.search(r"^export CAD_TELEMETRY=0$", source, re.MULTILINE), "cadtool no longer exports it"
    install = re.search(r"^install_server\(\) \{(?P<body>.*?)^\}$", source, re.MULTILINE | re.DOTALL)
    assert install, "cadtool has no install_server()"
    off, doctor = install["body"].find('"$cad" telemetry off'), install["body"].find('"$cad" doctor')
    assert -1 < off < doctor, "./cadtool server must store the off before `cad doctor` starts the server"
    assert re.search(r"^    install_server ;;$", source, re.MULTILINE), "`server` / `setup` no longer run it"
    assert "CAD_TELEMETRY=0" in (CAD_DIR / ".env").read_text().splitlines()


def test_cad_automatic_updates_stay_off():
    """From cad 0.11 the server installs a release that breaks nothing by itself and moves every agent app's plugin to
    it, which takes the machine off the pin (docs/toolchain.md Pins). `./cadtool server` stores `cad update auto off`
    before `cad doctor` starts the server, as it does the telemetry off; `./cadtool doctor` reads it back."""
    source = (CAD_DIR / "cadtool").read_text()
    install = re.search(r"^install_server\(\) \{(?P<body>.*?)^\}$", source, re.MULTILINE | re.DOTALL)
    assert install, "cadtool has no install_server()"
    off, doctor = install["body"].find('"$cad" update auto off'), install["body"].find('"$cad" doctor')
    assert -1 < off < doctor, "./cadtool server must store automatic updates off before `cad doctor` starts the server"
    assert "update auto status" in source, "./cadtool doctor no longer reads the stored choice"


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
    ("robot/arm.sdf", "sdf"), ("009e0c63-bcc4-5f1c-9e2c-f9146bfc6a87", "model")])
def test_cadtool_names_cads_document_type(source, kind):
    """cad 0.11's verbs are `cad <document type> <operation> FILE`: `./cadtool snapshot` / `export` take the type from
    the file's suffix, and anything else is a document id, `model`'s."""
    proc = subprocess.run(["bash", "-c", 'source <(sed -n "/^doc_type() {/,/^}/p" cadtool); doc_type "$1"', "_", source],
                          cwd=CAD_DIR, capture_output=True, text=True, check=True)
    assert proc.stdout.strip() == kind


def code_files() -> list[pathlib.Path]:
    """The tracked files that are run: cadtool, the environment files, the hooks, the workflows and all Python."""
    root = CAD_DIR.parent
    names = [n for n in tracked_names() if n.startswith((".github/", ".githooks/", ".claude/hooks/", "cad/.env"))
             or n == "cad/cadtool" or n == "cad/pyproject.toml" or (n.startswith("cad/") and n.endswith(".py"))]
    return [root / n for n in names if not n.startswith("cad/reference/")]


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


@pytest.mark.parametrize("verb", ["setup", "update", "uninstall", "doctor", "run"])
def test_cadtool_refuses_what_moves_the_machine(verb):
    """`cad setup` / `update` take every agent app to the newest release whatever the pin says, `doctor` can register
    an installation and `run` builds outside this venv (docs/toolchain.md Gotchas). Refused before anything of cad runs."""
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


def test_claude_settings_are_the_same_in_the_root_and_in_cad():
    """A Claude Code session reads the shared .claude/settings.json of the directory it starts in only (it is not
    inherited like AGENTS.md), so cad/.claude/settings.json is a copy of the root one: the ruff hook and the
    text-to-cad@earthtojake plugin, whether the session starts in the repo root or in cad/."""
    root = json.loads((CAD_DIR.parent / ".claude" / "settings.json").read_text())
    cad = json.loads((CAD_DIR / ".claude" / "settings.json").read_text())
    assert cad == root, "the two .claude/settings.json differ - edit the root one and copy it to cad/.claude/"


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
