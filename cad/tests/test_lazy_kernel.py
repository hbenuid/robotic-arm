"""Importing a model file never loads the CAD kernel.

A model run (`./cadtool gen`) hands its build to this venv's warm cadgen engine, which decides whether anything
has to run; the calling process loads the kernel only if the model's import closure does - and then every run,
even one the engine finds current, pays the kernel's import. So the whole import closure of a model stays
kernel-free: `import build123d as bd` (cadgen's import hook, `cadgen-lazy-kernel.pth`, binds a module the kernel
loads into on its first attribute) and `bd.<name>` inside function bodies only - no `from build123d import ...`,
no kernel object in a module-level constant, a class body, a decorator or an argument default; frames a module
declares are data (lib/datum.py). The lazy `build123d` / `OCP` modules sit in sys.modules from the first import:
the kernel is loaded once any of their submodules is. Checked in a fresh interpreter, one import at a time, so
the message names the first offender."""
import subprocess
import sys

from lib.reference import CAD_DIR

PROBE = r"""
import importlib, pathlib, sys
import parts

def loaded():
    return sorted({m.split(".")[0] for m in sys.modules if m.startswith(("build123d.", "OCP."))})

modules = [f"parts._templates.{p.stem}" for p in sorted(pathlib.Path("parts/_templates").glob("*.py")) if p.stem != "__init__"]
modules += [parts.load(name).__name__ for name in parts.names()]
modules += ["assemblies.gripper", "assemblies.cycloidal_drive", "assemblies.forearm_roll_drive", "assemblies.wrist_roll_drive",
            "assemblies.arm"]
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
    # -B: its bare env drops cadtool's PYTHONDONTWRITEBYTECODE, and no __pycache__/ belongs in the tree.
    run = subprocess.run([sys.executable, "-B", "-c", PROBE], cwd=CAD_DIR, capture_output=True, text=True,
                         check=False, env={"PYTHONPATH": str(CAD_DIR), "PATH": ""})
    assert run.returncode == 0, (run.stdout + run.stderr).strip()
    assert int(run.stdout) >= 41 + 3 + 8, run.stdout
