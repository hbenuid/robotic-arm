"""The package layering of cad/, locked by an AST scan of every source file (nothing is imported).

    lib  <-  parts  <-  assemblies  <-  robot  <-  tools  <-  tests

A package imports only itself and the ones to its left - so there is no cycle, `lib/` stays
importable on its own, and a part never depends on how it is assembled. Function-local imports
count too (they are still edges). Two finer rules ride along:
  * lib/cycloidal/, lib/forearm/ and lib/upper_arm/ never import lib/params.py: that module re-exports their
    interface values, so they take their globals from the leaves (lib/units.py, lib/motors.py, lib/belts.py,
    lib/geom.py) instead - and the leaves never import lib/params.py either;
  * nothing mutates sys.path (cadtool / pytest / .env put cad/ on the path) - except the one tool that
    runs in ANOTHER repo's venv.
"""
import ast
import pathlib

import pytest

CAD_DIR = pathlib.Path(__file__).resolve().parent.parent
ORDER = ["lib", "parts", "assemblies", "robot", "tools", "tests"]
ALLOWED = {pkg: set(ORDER[: i + 1]) for i, pkg in enumerate(ORDER)}

# tools/cycloidal/export_cadquery.py runs in the cycloidal_drive repo's CadQuery venv (never ours)
# and puts THAT repo's root on the path - a cross-repo bootstrap, not a cad/ import shim.
SYS_PATH_ALLOWED = {"tools/cycloidal/export_cadquery.py"}

# Packages that may name a part module directly (`parts.<group>.<name>`); everyone else - tests
# included - goes through parts.load() / parts.model() / parts.build() / parts.names().
PART_MODULE_IMPORT_ALLOWED = {"parts"}

# lib/ packages lib/params.py re-exports from: they take their globals from the leaves (lib/units.py, lib/motors.py,
# lib/belts.py) and never import lib.params back.
LEAF_PACKAGES = ("cycloidal", "forearm", "upper_arm")
# The leaves those packages import: importing lib.params from one would close the cycle.
LEAF_MODULES = ("units.py", "motors.py", "belts.py", "geom.py")

SOURCES = sorted(p for pkg in ORDER for p in (CAD_DIR / pkg).rglob("*.py") if "__pycache__" not in p.parts)


def _imports(tree: ast.AST) -> list[tuple[int, str, list[str]]]:
    """(line, dotted module, imported names) for every absolute import statement in the tree."""
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found += [(node.lineno, alias.name, []) for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            found.append((node.lineno, node.module, [alias.name for alias in node.names]))
    return found


def _rel(path: pathlib.Path) -> str:
    return path.relative_to(CAD_DIR).as_posix()


def test_sources_were_discovered():
    assert len(SOURCES) > 80 and {p.relative_to(CAD_DIR).parts[0] for p in SOURCES} == set(ORDER)


@pytest.mark.parametrize("path", SOURCES, ids=_rel)
def test_imports_respect_the_layering(path):
    pkg = path.relative_to(CAD_DIR).parts[0]
    tree = ast.parse(path.read_text(encoding="utf-8"))
    problems = []
    for line, module, names in _imports(tree):
        head = module.split(".")[0]
        if head in ALLOWED and head not in ALLOWED[pkg]:
            problems.append(f"{_rel(path)}:{line} imports {module} - {pkg}/ may only import {sorted(ALLOWED[pkg])}")
        if (any(path.is_relative_to(CAD_DIR / "lib" / leaf) for leaf in LEAF_PACKAGES)
                or path in [CAD_DIR / "lib" / leaf for leaf in LEAF_MODULES]) and (
                module == "lib.params" or (module == "lib" and "params" in names)):
            problems.append(f"{_rel(path)}:{line} imports lib.params - lib/{path.relative_to(CAD_DIR).parts[1]} takes its globals from the leaves (lib/units.py, lib/motors.py, lib/belts.py, lib/geom.py)")
        if pkg not in PART_MODULE_IMPORT_ALLOWED and (module.startswith("parts.") or (module == "parts" and names)):
            problems.append(f"{_rel(path)}:{line} imports {module} - reach parts through parts.load()/model()/build()")
    if _rel(path) not in SYS_PATH_ALLOWED:
        problems += [f"{_rel(path)}:{node.lineno} touches sys.path - cadtool/pytest/.env put cad/ on the path"
                     for node in ast.walk(tree)
                     if isinstance(node, ast.Attribute) and node.attr == "path"
                     and isinstance(node.value, ast.Name) and node.value.id == "sys"]
    assert not problems, "\n".join(problems)


def test_there_is_no_cad_package_root():
    """cad/__init__.py would make the repo root the package root (cadgen walks the __init__.py chain)."""
    assert not (CAD_DIR / "__init__.py").exists()
