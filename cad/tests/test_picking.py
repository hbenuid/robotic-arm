"""`./cadtool pytest`'s choice of the test files a change reaches (tests/run.py, tests/picking.py): a cache shared across
test files records through picking.cache (else a file taking a part another file built would not depend on it); a
subprocess makes a file depend on what it may run; a file's imports are read all the way down, a function's own imports
and relative ones too; a file's fingerprint moves with what it depends on, and only with that. No kernel: fast lane."""
import ast
from pathlib import Path

from tests import picking
from tests.run import Deps, _names_tests, files

TESTS = Path(__file__).resolve().parent


def test_shared_caches_record_through_picking():
    """A module of tests/ that is not a test file (its helpers, built.py) caches through picking.cache, never
    functools' own: what its first computation took by name is then recorded for every later caller."""
    offenders = []
    for path in TESTS.rglob("*.py"):
        if path.name.startswith("test_") or path.name == "picking.py":
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Attribute) and node.attr in ("cache", "lru_cache") and \
                    isinstance(node.value, ast.Name) and node.value.id == "functools":
                offenders.append(f"{path.relative_to(TESTS)}:{node.lineno}")
    assert not offenders, offenders


def test_a_cached_computation_is_recorded_for_every_caller():
    calls = []

    @picking.cache
    def build(name):
        calls.append(name)
        picking.note(f"mod:parts.{name}")
        return name.upper()

    first, second = set(), set()
    picking._current[0] = first
    assert build("x") == "X"
    picking._current[0] = second
    assert build("x") == "X"
    picking._current[0] = None
    assert calls == ["x"] and first == second == {"mod:parts.x"}


def test_subprocesses_add_what_they_may_run():
    deps = picking.subprocess_deps
    assert deps(["/venv/bin/python", "-P", "-m", "cad.engine.reader"]) == []
    assert deps(["git", "-C", ".", "ls-files", "-z"]) == []
    assert deps(["bash", "-c", 'f=$(sed -n "/^doc_type() {/,/^}/p" cadtool); eval "$f"; doc_type "$1"', "_", "a.stl"]) == []
    assert deps(["./cadtool", "validate", "robot/arm.urdf", "--strict"]) == []
    assert deps(["./cadtool", "cad", "update"]) == []
    assert deps(["./cadtool", "inspect", "a.step"]) == ["*py"]
    assert deps(["/venv/bin/python", "-B", "-c", "import sys, lib.gauges.fits, lib.gauges.layout"]) == [
        "mod:lib.gauges.fits", "mod:lib.gauges.layout"]
    assert deps(["/venv/bin/python", "-B", "-c", "import importlib; importlib.import_module(sys.argv[1])"]) == ["*py"]
    assert deps(["uv", "run", "python", "tools/step_facts.py"]) == ["*py"]


def test_imports_are_read_all_the_way_down():
    """built.py's closure: its own imports, theirs (assemblies/_occurrences imports lib), a function-local import
    (lib/models.py), the packages on the way."""
    deps = Deps(files())
    closure = deps.closure(["cad/tests/built.py"])
    for path in ("cad/tests/picking.py", "cad/parts/__init__.py", "cad/assemblies/_occurrences.py",
                 "cad/lib/placements.py", "cad/lib/models.py", "cad/lib/__init__.py", "cad/assemblies/__init__.py"):
        assert path in closure, path
    assert not any(p.startswith("cad/parts/") and p != "cad/parts/__init__.py" for p in closure)   # parts: by name


def test_a_fingerprint_moves_with_its_dependencies_only():
    ids = files()
    base = Deps(ids).fingerprint("tests/test_picking.py", [])
    assert Deps(dict(ids)).fingerprint("tests/test_picking.py", []) == base
    unrelated = next(p for p in ids if p.startswith("cad/parts/") and p.endswith(".py") and "__init__" not in p)
    assert Deps({**ids, unrelated: "0" * 40}).fingerprint("tests/test_picking.py", []) == base
    assert Deps({**ids, unrelated: "0" * 40}).fingerprint("tests/test_picking.py",
                                                          ["mod:" + unrelated[4:-3].replace("/", ".")]) != base
    assert Deps({**ids, "cad/tests/run.py": "0" * 40}).fingerprint("tests/test_picking.py", []) != base   # imported
    assert Deps({**ids, "cad/reference/placements.json": "0" * 40}).fingerprint("tests/test_picking.py", []) != base
    assert Deps({**ids, "cad/parts/joints/new_part.py": "0" * 40}).fingerprint("tests/test_picking.py", []) != base
    assert Deps(ids).fingerprint("tests/test_picking.py", ["file:" + unrelated[4:]]) != base


def test_named_tests_are_told_from_options():
    assert _names_tests(["tests/test_bom.py"]) and _names_tests(["-k", "belt"]) and _names_tests(["-m", "not slow"])
    assert _names_tests(["tests/test_bom.py::test_x"]) and _names_tests(["tests/forearm"])
    assert not _names_tests(["-n", "4", "-q", "-p", "no:cacheprovider", "--durations=15", "-x"])
