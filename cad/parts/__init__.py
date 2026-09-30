"""parts - one module per part, grouped by subsystem: parts/<group>/<name>.py + <name>.step
(groups: base, joints, wrist, gripper, cycloidal). parts/_templates/ holds the three templates (not
discovered); every purchased part builds through lib/cots.py hybrid().
Every part module declares ONE cadgen model, `@step def <name>()` (NAME = stem = model name);
running the file builds it, model(name) hands the function to assemblies, build(name) runs its body.

A part's NAME is its module stem and must be unique across groups: it keys reference/manifest.json,
reference/<origin>/<name>.step, placements.json and robot/frames.py LINKS. Discovery is a directory
scan at import time; load(name) imports the module lazily, so `import parts` never pulls build123d.
`parts.base` is the GROUP package - the part is parts.load("base").
"""
from __future__ import annotations

import importlib
import pathlib
import pkgutil
from types import ModuleType

_ROOT = pathlib.Path(__file__).resolve().parent


def _scan() -> dict[str, str]:
    found: dict[str, str] = {}
    for group in pkgutil.iter_modules([str(_ROOT)]):
        if not group.ispkg or group.name.startswith("_"):
            continue
        for mi in pkgutil.iter_modules([str(_ROOT / group.name)]):
            if mi.ispkg or mi.name.startswith("_"):
                continue
            dotted = f"{__name__}.{group.name}.{mi.name}"
            if mi.name in found:
                raise ImportError(f"duplicate part name {mi.name!r}: {found[mi.name]} and {dotted}")
            found[mi.name] = dotted
    return dict(sorted(found.items()))


MODULES: dict[str, str] = _scan()                                           # name -> "parts.<group>.<name>"
GROUPS: dict[str, str] = {n: d.split(".")[1] for n, d in MODULES.items()}   # name -> group


def names() -> list[str]:
    """Every part name, sorted."""
    return list(MODULES)


def load(name: str) -> ModuleType:
    """Import parts/<group>/<name>.py (importlib caches it)."""
    if name not in MODULES:
        raise KeyError(f"no part {name!r} under parts/<group>/ (known: {', '.join(MODULES)})")
    return importlib.import_module(MODULES[name])


def bought(name: str) -> bool:
    """True for a purchased part (the module declares COTS = True), False for a printed one - the one
    make/buy label the lists (tools/bom.py), the assemblies' colors and the STL export read."""
    return bool(getattr(load(name), "COTS", False))


def unplaced() -> list[str]:
    """The parts modelled before any assembly places them (the module declares UNPLACED = "<why>"): the print and buy
    lists skip them, their order line stays a tools/bom.py EXTRAS row (parts/AGENTS.md "Modelled, not placed yet")."""
    return [name for name in MODULES if getattr(load(name), "UNPLACED", None) is not None]


def source_of(name: str) -> pathlib.Path:
    return _ROOT / GROUPS[name] / f"{name}.py"


def model(name: str):
    """The part's cadgen model: the `@step` function named after its file (parts.load(name).<name>)."""
    from lib.models import model_of

    return model_of(load(name), name)


def build(name: str):
    """The part's geometry built in-process - the model BODY: no freshness gate, no store, nothing
    written. What tests and tools want; assemblies call lib.models.geometry(model(name)) instead."""
    from lib.models import raw

    return raw(model(name))
