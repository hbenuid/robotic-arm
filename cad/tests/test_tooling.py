"""The local tooling stays healthy: the venv holds the cadgen and the OCP kernel pyproject.toml pins
(and only ONE OCP distribution), and ./cadtool inspect (tools/step_facts.py) agrees with the kernel."""
import json
import pathlib
import re
import subprocess
import tomllib
from importlib import metadata

import pytest

from lib.reference import CAD_DIR, describe, path_of
from tools import step_facts


def pinned(package: str) -> str:
    """The `==` pin of `package` in cad/pyproject.toml (extras ignored)."""
    dependencies = tomllib.loads((CAD_DIR / "pyproject.toml").read_text())["project"]["dependencies"]
    for spec in dependencies:
        match = re.fullmatch(rf"{re.escape(package)}(?:\[[^\]]*\])?==(?P<pin>\S+)", spec)
        if match:
            return match["pin"]
    raise AssertionError(f"{package} is not pinned with == in pyproject.toml")


def test_installed_cadgen_is_the_pinned_one():
    # The plugin's skills/cad/SKILL.md carries the same pin; ./cadtool doctor checks that side.
    assert metadata.version("cadgen") == pinned("cadgen"), "run ./cadtool setup"


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
    path = path_of("gripper_link_1")
    report, expected = step_facts.facts(path), describe(path)
    totals = report["totals"]
    assert totals["solids"] == expected["solids"]
    assert totals["volume"] == pytest.approx(expected["solid_volume"], abs=1e-3)
    assert totals["bbox_min"] == pytest.approx(expected["bbox_min"], abs=1e-3)
    assert totals["bbox_size"] == pytest.approx(expected["bbox_size"], abs=1e-3)
    assert [p["ref"] for p in step_facts.planes(path)], "a printed link has planar faces"
    assert step_facts.diff(path, path) == []


def test_daemon_stop_finds_the_daemon_on_both_platforms():
    """./cadtool daemon stop greps the process list for the venv interpreter running cadgen.daemon. That
    interpreter is named per platform - `.venv/bin/python3` on Linux, `.venv/bin/python` on macOS (where
    python3 is the symlink) - and a pattern that knows only one of them reports "no cadgen daemon running"
    while the daemon keeps its old code loaded (2026-09-21, the arm64 Mac). cadgen 0.7.13+ starts the daemon and
    its workers as `python -P -m …` (no cwd on their import path): the pattern takes both forms."""
    source = (CAD_DIR / "cadtool").read_text()
    match = re.search(r'^\s*daemon="\^\$CAD_DIR(?P<tail>[^"]+)"$', source, re.MULTILINE)
    assert match, "cadtool: the daemon= process pattern of `daemon stop` moved - update this test"
    pattern = re.compile("^/repo/cad" + match.group("tail"))
    for exe in ("python", "python3", "python3.12", "python -P", "python3 -P"):
        assert pattern.search(f"/repo/cad/.venv/bin/{exe} -m cadgen.daemon"), exe
        assert pattern.search(f"/repo/cad/.venv/bin/{exe} -m cadgen.daemon.worker"), exe
    assert not pattern.search("/repo/cad/.venv/bin/python -m cadgen viewer")
    assert not pattern.search("/other/cad/.venv/bin/python3 -m cadgen.daemon"), "another checkout's daemon"
    assert source.count("-m cadgen\\.daemon") == 1, "one pattern, reused by pgrep and both pkills"


def test_cadgen_telemetry_stays_off():
    """cadgen 0.7.16+ sends usage stats and crash reports by default once a cadgen command has shown its notice; this
    repo keeps them off (docs/toolchain.md Pins). In the environment of every cadtool command (and VS Code's, `.env`)
    nothing shows the notice or sends; `./cadtool setup` stores the "off" the plugin's server and the viewer go by."""
    source = (CAD_DIR / "cadtool").read_text()
    assert re.search(r"^export CADGEN_TELEMETRY=0$", source, re.MULTILINE), "cadtool no longer exports it"
    setup = re.search(r"^  setup\)\n(?P<body>.*?);;$", source, re.MULTILINE | re.DOTALL)
    assert setup and "cadgen telemetry off" in setup["body"], "./cadtool setup no longer stores the off"
    assert "CADGEN_TELEMETRY=0" in (CAD_DIR / ".env").read_text().splitlines()


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
    """git and the *.step / *.stl rules are case-sensitive, the Mac's filesystem is not: git there runs with
    core.ignorecase=true, so a case-only rename goes unnoticed on the Mac and arrives on Linux as a second file.
    Every tracked path is lowercase except the README / AGENTS.md docs (root AGENTS.md "Git workflow")."""
    names = tracked_names()
    checked = [n.rpartition("/")[0] if n.rpartition("/")[2] in ("README", "README.md", "AGENTS.md") else n for n in names]
    upper = [n for n, c in zip(names, checked, strict=True) if c != c.lower()]
    assert not upper, f"uppercase in tracked names (rename with git mv -f): {upper}"
