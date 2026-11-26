"""`./cadtool pytest [--full] [pytest args]`: the suite's runner (tests/AGENTS.md "Running the suite").

By default a run takes the test FILES a change can reach, on as many workers as the machine has room for:
- a file is run again when something it depends on changed since it last passed (or it never passed): its own source,
  what it and its conftests import (read from the source, all the way down), what tests/picking.py recorded while it
  last ran (a part or module it took by name - and what that imports -, a source file it read as text, what a
  subprocess it started may run), and for every file: every data file and input (whatever is not a .py of
  cad/: the committed STEPs, placements.json, the URDF, cadtool, the lock, software/ ...) and the list of every
  file's path (a part added or removed changes what parts.names() gives). Nothing reached: nothing runs. The record is
  .picked.json, one per checkout (git-ignored); a file that fails, or does not run all its tests, leaves it.
- --full runs every file (and records them); tests named on the command line (a path, a node, -k, -m) run as named,
  and a whole file run that way is recorded too.
- the workers: -n from the machine's performance cores and its free memory (WORKER_MEMORY each), never more than the
  files to run, unless -n is given.
On CI (the CI variable set) the run is pytest's own: what ci.yml names, nothing picked, nothing recorded.
Stdlib only, and no kernel: it runs before pytest does.
"""
from __future__ import annotations

import ast
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

CAD = Path(__file__).resolve().parent.parent
REPO = CAD.parent
PACKAGES = ("lib", "parts", "assemblies", "robot", "tools", "tests")
WORKER_MEMORY = 2 * 1024 ** 3   # bytes a worker may take (peak measured 1.7 GB on the Mac, -n 8 and -n 12)
STATE = CAD / ".picked.json"


def _git(*args: str, data: bytes | None = None) -> bytes:
    return subprocess.run(["git", *args], cwd=REPO, input=data, capture_output=True, check=True).stdout


def files() -> dict[str, str]:
    """Every file git tracks or sees untracked (not ignored), repo-relative -> its content's blob id as the work tree
    has it (a deleted one left out)."""
    ids = {}
    for entry in _git("ls-files", "-s", "-z").split(b"\0"):
        if entry:
            meta, path = entry.split(b"\t", 1)
            ids[path.decode()] = meta.split()[1].decode()
    for path in _git("ls-files", "-d", "-z").decode().split("\0"):
        ids.pop(path, None)
    changed = [p for p in (_git("ls-files", "-m", "-z") + _git("ls-files", "-o", "--exclude-standard", "-z"))
               .decode().split("\0") if p and (REPO / p).is_file()]
    if changed:
        blobs = _git("hash-object", "--stdin-paths", data="\n".join(changed).encode()).decode().split()
        ids.update(zip(changed, blobs, strict=True))
    return ids


class Deps:
    """What a test file depends on, from the tree as it is now (`ids`, files())."""

    def __init__(self, ids: dict[str, str]):
        self.ids = ids
        self.imports: dict[str, set[str]] = {}
        h = hashlib.sha256(f"{sys.version}|{platform.platform()}|{platform.machine()}".encode())
        for path in sorted(ids):   # every data file and input, and every path (the module's docstring)
            code = path.endswith(".py") and path.startswith("cad/")
            h.update(f"{path}\0{'' if code else ids[path]}\n".encode())
        self.common = h.hexdigest()

    def module_file(self, name: str) -> str | None:
        """cad/<a>/<b>.py or cad/<a>/<b>/__init__.py of the dotted module name, as git has it."""
        base = "cad/" + name.replace(".", "/")
        for path in (base + ".py", base + "/__init__.py"):
            if path in self.ids:
                return path
        return None

    def _imported(self, path: str) -> set[str]:
        """The project files `path` imports directly (anywhere in it: a function's imports count), with the packages
        on their way."""
        if path in self.imports:
            return self.imports[path]
        found: set[str] = set()
        try:
            tree = ast.parse((REPO / path).read_bytes())
        except (OSError, SyntaxError, ValueError):
            tree = None
        package = path[len("cad/"):-len(".py")].replace("/", ".")
        package = package.removesuffix(".__init__") if path.endswith("__init__.py") else package.rpartition(".")[0]
        names: list[str] = []
        for node in ast.walk(tree) if tree is not None else ():
            if isinstance(node, ast.Import):
                names += [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                base = node.module or ""
                if node.level:
                    parent = package.split(".")[: len(package.split(".")) - node.level + 1]
                    base = ".".join([*parent, base] if base else parent)
                names.append(base)
                names += [f"{base}.{a.name}" for a in node.names]
        for name in names:
            if name.split(".")[0] in PACKAGES:
                parts = name.split(".")
                for i in range(1, len(parts) + 1):
                    hit = self.module_file(".".join(parts[:i]))
                    if hit:
                        found.add(hit)
        self.imports[path] = found
        return found

    def closure(self, roots) -> set[str]:
        seen, todo = set(), [r for r in roots if r]
        while todo:
            path = todo.pop()
            if path not in seen:
                seen.add(path)
                todo += self._imported(path)
        return seen

    def fingerprint(self, test: str, notes) -> str:
        """The fingerprint of everything `test` (cad/-relative) depends on, given what picking recorded for it: its
        and its conftests' imports, the modules it took by name and theirs, the source files it read as text (every
        .py file where it ran a subprocess that may run any of them), and every file's data (`common`)."""
        conftests = [str(Path("cad", *Path(test).parts[:i], "conftest.py")) for i in range(1, len(Path(test).parts))]
        roots: list[str | None] = ["cad/" + test, *(c for c in conftests if c in self.ids)]   # closure() drops a None
        roots += [self.module_file(n[len("mod:"):]) for n in notes if n.startswith("mod:")]
        deps = self.closure(roots) | {"cad/" + n[len("file:"):] for n in notes if n.startswith("file:")}
        if "*py" in notes:
            deps |= {p for p in self.ids if p.endswith(".py") and p.startswith("cad/")}
        h = hashlib.sha256(self.common.encode())
        for path in sorted(deps):
            h.update(f"{path}\0{self.ids.get(path, '-')}\n".encode())
        return h.hexdigest()


def _cores() -> int:
    if sys.platform == "darwin":   # the performance cores (an Apple chip's efficiency cores are much slower)
        for key in ("hw.perflevel0.physicalcpu", "hw.physicalcpu"):
            out = subprocess.run(["sysctl", "-n", key], capture_output=True, text=True, check=False).stdout.strip()
            if out.isdigit():
                return int(out)
    try:                           # Linux: physical cores (a core's second thread adds little to this work)
        cpuinfo = Path("/proc/cpuinfo").read_text()
        pairs = set(re.findall(r"physical id\s*:\s*(\d+).*?core id\s*:\s*(\d+)", cpuinfo, re.DOTALL))
        if pairs:
            return len(pairs)
    except OSError:
        pass
    return os.cpu_count() or 1


def _free_memory() -> int:
    if sys.platform == "darwin":
        out = subprocess.run(["vm_stat"], capture_output=True, text=True, check=False).stdout
        page = int(re.search(r"page size of (\d+)", out).group(1))
        pages = sum(int(re.search(rf"Pages {k}:\s+(\d+)", out).group(1)) for k in ("free", "inactive", "speculative"))
        return page * pages
    meminfo = Path("/proc/meminfo").read_text()
    return int(re.search(r"MemAvailable:\s+(\d+) kB", meminfo).group(1)) * 1024


def workers() -> int:
    """As many as the performance cores, and as the free memory takes at WORKER_MEMORY each: a second session's
    suite, already running, leaves less free and gets fewer."""
    return max(1, min(_cores(), _free_memory() // WORKER_MEMORY))


def test_files() -> list[str]:
    """Every test file of the suite, cad/-relative (pytest's testpaths and python_files)."""
    return sorted(str(p.relative_to(CAD)) for p in (CAD / "tests").rglob("test_*.py"))


def _names_tests(args: list[str]) -> bool:
    """Tests named on the command line: a test file or folder, a node (file.py::test), -k or -m."""
    return any(a in ("-k", "-m") or a.startswith(("-k", "-m")) or (not a.startswith("-") and
               ("::" in a or (CAD / a.split("::")[0]).exists())) for a in args)


def _record(out: Path, deps: Deps, state: dict, ran: list[str]) -> None:
    """Keep the files of this run that passed - every test they hold run and none failed -, drop those that did not."""
    counts = json.loads((out / "collected.json").read_text()) if (out / "collected.json").exists() else {}
    merged: dict[str, dict] = {}
    for worker in out.glob("worker-*.json"):
        for path, entry in json.loads(worker.read_text()).items():
            m = merged.setdefault(path, {"notes": set(), "passed": 0, "failed": 0})
            m["notes"] |= set(entry["notes"])
            m["passed"] += entry["passed"]
            m["failed"] += entry["failed"]
    for path in ran:
        state.pop(path, None)
    for path, m in merged.items():
        if m["failed"] == 0 and counts.get(path) and m["passed"] == counts[path]:
            state[path] = {"fingerprint": deps.fingerprint(path, m["notes"]), "notes": sorted(m["notes"])}
        else:
            state.pop(path, None)
    STATE.write_text(json.dumps(state, indent=0, sort_keys=True))


def main(argv: list[str]) -> int:
    full = "--full" in argv
    args = [a for a in argv if a != "--full"]
    if os.environ.get("CI"):
        return subprocess.run([sys.executable, "-m", "pytest", *args], check=False).returncode
    named = _names_tests(args)
    whole = named and not any(a in ("-k", "-m") or a.startswith(("-k", "-m")) or "::" in a for a in args)
    deps = Deps(files())
    try:
        state = json.loads(STATE.read_text()) if STATE.exists() else {}
    except ValueError:
        state = {}
    every = test_files()
    if full:
        picked, ran, what = [], every, "every test file"
    elif named:
        given = [a.rstrip("/") for a in args if not a.startswith("-") and (CAD / a.split("::")[0]).exists()]
        ran = [t for t in every if any(t == g or t.startswith(g + "/") for g in given)]
        picked, what = [], "the tests named"
    else:
        picked = [t for t in every if t not in state or state[t]["fingerprint"] != deps.fingerprint(t, state[t]["notes"])]
        ran, what = picked, f"{len(picked)} of {len(every)} test files (each changed or reaching a change since it last passed)"
        if not picked:
            print(f"cadtool pytest: nothing to run - every test file passed since its last change ({STATE.name}; "
                  "--full runs them all)")
            return 0
    if not any(a.startswith(("-n", "--numprocesses")) for a in args):
        args = ["-n", str(min(workers(), len(picked) if picked else len(every))), *args]
    print(f"cadtool pytest: {what}, {args[1] if args[0] == '-n' else 'the given'} workers", flush=True)
    out = Path(tempfile.mkdtemp(prefix="cad-pick-"))
    env = {**os.environ, "CAD_TEST_PICK_OUT": str(out)} if (whole or full or not named) else dict(os.environ)
    try:
        code = subprocess.run([sys.executable, "-m", "pytest", *args, *picked], env=env, check=False).returncode
        if "CAD_TEST_PICK_OUT" in env:
            _record(out, deps, state, ran)
    finally:
        shutil.rmtree(out, ignore_errors=True)
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
