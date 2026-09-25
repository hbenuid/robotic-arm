"""The local tooling stays healthy: the venv holds the cadgen and the OCP kernel pyproject.toml pins
(and only ONE OCP distribution), and ./cadtool inspect (tools/step_facts.py) agrees with the kernel."""
import pathlib
import re
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
    # The plugin's skills/cad/requirements.txt carries the same pin; ./cadtool doctor checks that side.
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
    while the daemon keeps its old code loaded (2026-09-21, the arm64 Mac)."""
    source = (CAD_DIR / "cadtool").read_text()
    match = re.search(r'^\s*daemon="\^\$CAD_DIR(?P<tail>[^"]+)"$', source, re.MULTILINE)
    assert match, "cadtool: the daemon= process pattern of `daemon stop` moved - update this test"
    pattern = re.compile("^/repo/cad" + match.group("tail"))
    for exe in ("python", "python3", "python3.12"):
        assert pattern.search(f"/repo/cad/.venv/bin/{exe} -m cadgen.daemon"), exe
        assert pattern.search(f"/repo/cad/.venv/bin/{exe} -m cadgen.daemon.worker"), exe
    assert not pattern.search("/repo/cad/.venv/bin/python -m cadgen viewer")
    assert not pattern.search("/other/cad/.venv/bin/python3 -m cadgen.daemon"), "another checkout's daemon"
    assert source.count("-m cadgen\\.daemon") == 1, "one pattern, reused by pgrep and both pkills"
