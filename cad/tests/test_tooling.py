"""The local tooling stays healthy: the venv holds the cadgen and the OCP kernel pyproject.toml pins
(and only ONE OCP distribution), and ./cadtool inspect (tools/step_facts.py) agrees with the kernel."""
import importlib.metadata as metadata
import pathlib
import re
import tomllib

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
