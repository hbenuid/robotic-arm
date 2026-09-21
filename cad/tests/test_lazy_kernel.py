"""Importing a model file never loads the CAD kernel.

cadgen gates a model (freshness check, warm-daemon dispatch) BEFORE any OCP cost is paid - but only
if neither `build123d` nor `OCP` is in sys.modules when the model is called; otherwise every run,
even of a current model, pays the ~1.5 s import and prints the "kernel was imported" hint. So the
whole import closure of a model stays kernel-free: `from cadgen import build123d as bd` (a lazy
proxy) and `bd.<name>` inside function bodies only - no kernel object in a module-level constant, a
class body, a decorator or an argument default; frames a module declares are data (lib/datum.py).
Checked in a fresh interpreter, one import at a time, so the message names the first offender."""
import subprocess
import sys

from lib.reference import CAD_DIR

PROBE = r"""
import importlib, pathlib, sys
import parts

def loaded():
    return sorted(m for m in ("build123d", "OCP") if m in sys.modules)

modules = [f"parts._templates.{p.stem}" for p in sorted(pathlib.Path("parts/_templates").glob("*.py")) if p.stem != "__init__"]
modules += [parts.load(name).__name__ for name in parts.names()]
modules += ["assemblies.gripper", "assemblies.cycloidal_drive", "assemblies.arm", "assemblies.arm_no_caps",
            "assemblies.cycloidal_drive_make_buy"]
modules += [f"robot.links.{p.stem}" for p in sorted(pathlib.Path("robot/links").glob("*.py")) if p.stem != "__init__"]
assert not loaded(), f"the parts index itself loaded {loaded()}"
for name in modules:
    importlib.import_module(name)
    if loaded():
        print(f"{name} loaded {loaded()} at import")
        sys.exit(1)
print(len(modules))
"""


def test_no_model_module_imports_the_kernel():
    run = subprocess.run([sys.executable, "-c", PROBE], cwd=CAD_DIR, capture_output=True, text=True,
                         env={"PYTHONPATH": str(CAD_DIR), "PATH": ""})
    assert run.returncode == 0, (run.stdout + run.stderr).strip()
    assert int(run.stdout) >= 41 + 3 + 8, run.stdout
