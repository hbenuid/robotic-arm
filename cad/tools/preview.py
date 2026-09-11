"""./cadtool show <model.py | dotted.name> - open a model's geometry in the OCP CAD Viewer
(VS Code) WITHOUT building it: the module is imported under its package name (parts.joints.x -
the same module object parts.load() returns) and lib.models.raw(model) is shown.

    ./cadtool show parts/joints/j3_coupler.py
    ./cadtool show assemblies.arm
"""
from __future__ import annotations

import importlib
import pathlib
import sys

from lib.models import model_of, raw

CAD_DIR = pathlib.Path(__file__).resolve().parent.parent


def dotted(target: str) -> str:
    """parts/joints/j3_coupler.py -> parts.joints.j3_coupler (a dotted name passes through)."""
    path = pathlib.Path(target)
    if path.suffix != ".py":
        return target
    rel = path.resolve().relative_to(CAD_DIR)
    return ".".join(rel.with_suffix("").parts)


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__)
        return 2
    module = importlib.import_module(dotted(argv[0]))
    shape = raw(model_of(module))
    from ocp_vscode import show

    show(shape)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
