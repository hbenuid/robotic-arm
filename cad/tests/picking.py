"""What each test FILE depends on beyond its imports, recorded while it runs - for tests/run.py, which reruns a file
only when something it depends on changed since it last passed (tests/AGENTS.md "Running the suite").

A file's static imports (its own, its conftests', and theirs, all the way down) the runner reads from the source. What
it cannot read there is recorded here, per test file, in every worker (tests/conftest.py's hooks call these):
- a module imported by name (importlib.import_module: a part by `parts.load`, a designed module by tests/built.py),
  so the runner adds that module and its imports;
- a source file read as text (test_layering's scan, the source checks): that file;
- a subprocess started: what it may run (`subprocess_deps`) - nothing for one that runs no project code, the modules
  a `python -c` snippet imports, every .py file ("*py") for anything else (test_lazy_kernel's fresh interpreter).
A cache shared across test files (tests/built.py, tests/forearm/helpers.py) goes through `cache`: what its first
computation recorded is recorded again for every later caller, so a file that takes a part another file built still
depends on it. Recording is on only where `./cadtool pytest` asks (CAD_TEST_PICK_OUT, a folder for the workers' notes).
"""
from __future__ import annotations

import builtins
import functools
import importlib
import importlib.util
import io
import json
import os
import subprocess
import sys
from pathlib import Path

CAD = Path(__file__).resolve().parent.parent
PACKAGES = ("lib", "parts", "assemblies", "robot", "tools", "tests")
OUT = os.environ.get("CAD_TEST_PICK_OUT")

_current: list[set | None] = [None]   # the notes of the test file running now (a stack: a cache inside a cache)
_files: dict[str, dict] = {}          # test file -> {"notes": set, "passed": n, "failed": n}
_active = [False]                     # this process runs tests (a worker, or pytest without workers)


def note(dep: str) -> None:
    """Record `dep` for the test file running now (and for every cache computing inside it)."""
    for notes in _current:
        if notes is not None:
            notes.add(dep)


def cache(fn):
    """functools.cache that records again, on every call, what its first computation recorded (the module's
    docstring)."""
    recorded: dict = {}

    @functools.wraps(fn)
    def call(*args, **kwargs):
        key = (args, tuple(sorted(kwargs.items())))
        if key not in recorded:
            notes: set = set()
            _current.append(notes)
            try:
                value = fn(*args, **kwargs)
            finally:
                _current.pop()
            recorded[key] = (value, notes)
        value, notes = recorded[key]
        for dep in notes:
            note(dep)
        return value

    call.cache_clear = recorded.clear
    return call


def start_file(path: str | None) -> None:
    """The test file now running (None between files)."""
    if path is None or not _active[0]:
        _current[0] = None
        return
    entry = _files.setdefault(path, {"notes": set(), "passed": 0, "failed": 0})
    _current[0] = entry["notes"]


def outcome(path: str, passed: bool) -> None:
    if _active[0]:
        entry = _files.setdefault(path, {"notes": set(), "passed": 0, "failed": 0})
        entry["passed" if passed else "failed"] += 1


NO_CODE = ("git", "bash", "sh", "uname", "file", "sysctl", "vm_stat", "sw_vers", "which", "nproc")
CADTOOL_NO_CODE = ("validate", "help", "why", "compile", "store", "daemon", "cadgen", "parts")   # cad's, or a message


def subprocess_deps(argv: list[str]) -> list[str]:
    """What a subprocess makes a test file depend on, beyond every file's data: nothing where it runs no project code
    (cad's STEP reader, git, a shell running cadtool's functions - cadtool is data -, `cadtool validate` and the verbs
    that only print), the project modules a `python -c` snippet imports (all of them where it imports by name), every
    .py file ("*py") for anything else."""
    import ast

    line = " ".join(argv)
    exe = Path(argv[0].split()[0]).name if argv else ""
    if "cad.engine.reader" in line or exe in NO_CODE:
        return []
    if exe == "cadtool":
        words = line.split()
        verb = words[1] if len(words) > 1 else "help"
        return [] if verb in CADTOOL_NO_CODE or (verb == "cad" and len(words) > 2 and words[2] in
                                                  ("setup", "update", "uninstall", "doctor", "run")) else ["*py"]
    if exe.startswith("python") and "-c" in argv:
        code = argv[argv.index("-c") + 1] if argv.index("-c") + 1 < len(argv) else ""
        try:
            tree = ast.parse(code)
        except SyntaxError:
            return ["*py"]
        names = [n.id if isinstance(n, ast.Name) else n.attr for n in ast.walk(tree)
                 if isinstance(n, (ast.Name, ast.Attribute))]
        if {"import_module", "__import__", "exec", "eval", "run_path", "run_module", "spec_from_file_location"} & set(names):
            return ["*py"]
        mods = [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
        mods += [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module and not n.level]
        return [f"mod:{m}" for m in mods if m.split(".")[0] in PACKAGES]
    return ["*py"]


def _from_pytest() -> bool:
    """Called from pytest's assertion rewriting, which reads every test module's source as it imports it (a test runs
    under pytest's code too, so only that module counts)."""
    frame = sys._getframe(2)
    for _ in range(6):
        if frame is None:
            return False
        if frame.f_code.co_filename.endswith(os.path.join("_pytest", "assertion", "rewrite.py")):
            return True
        frame = frame.f_back
    return False


def install(active: bool) -> None:
    """Watch imports by name, source read as text and subprocesses (the module's docstring) - in a process that runs
    tests (`active`: a worker, or pytest without workers; not xdist's controller, which only hands them out)."""
    _active[0] = bool(OUT) and active
    if not _active[0]:
        return
    real_import, real_open, real_popen = importlib.import_module, builtins.open, subprocess.Popen.__init__

    def import_module(name, package=None):
        full = importlib.util.resolve_name(name, package) if name.startswith(".") else name
        if full.split(".")[0] in PACKAGES:
            note("mod:" + full)
        return real_import(name, package)

    def open_(file, *args, **kwargs):
        if isinstance(file, (str, os.PathLike)) and str(file).endswith(".py") and not _from_pytest():
            try:
                path = Path(file).resolve()
                if path.is_relative_to(CAD):
                    note("file:" + str(path.relative_to(CAD)))
            except (OSError, ValueError):
                pass
        return real_open(file, *args, **kwargs)

    def popen(self, args, *rest, **kwargs):
        for dep in subprocess_deps([args] if isinstance(args, str) else [str(a) for a in args]):
            note(dep)
        return real_popen(self, args, *rest, **kwargs)

    importlib.import_module, subprocess.Popen.__init__ = import_module, popen
    builtins.open = io.open = open_


def collected(ids) -> None:
    """Every test file's count of tests, as a worker collected them (every worker collects them all): all of them must
    have run for a file to count as passed."""
    if not _active[0]:
        return
    counts: dict[str, int] = {}
    for nodeid in ids:
        path = nodeid.split("::")[0]
        counts[path] = counts.get(path, 0) + 1
    Path(OUT, "collected.json").write_text(json.dumps(counts))


def finish() -> None:
    """Write this process's notes and outcomes for the runner."""
    if _active[0] and _files:
        data = {path: {**entry, "notes": sorted(entry["notes"])} for path, entry in _files.items()}
        Path(OUT, f"worker-{os.getpid()}.json").write_text(json.dumps(data))
