"""Source-level checks shared by the convention tests (AST only - nothing is imported or built)."""
from __future__ import annotations

import ast
import pathlib


def runs_its_model(path: pathlib.Path, name: str) -> bool:
    """True if the file's LAST top-level statement is `if __name__ == "__main__": <name>()`.

    That call IS the build: `./cadtool gen <file>` just runs the file, so a model file without it
    builds nothing and says nothing."""
    body = ast.parse(path.read_text(encoding="utf-8")).body
    if not body or not isinstance(body[-1], ast.If):
        return False
    guard = body[-1]
    test = guard.test
    is_main = (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name) and test.left.id == "__name__"
               and len(test.ops) == 1 and isinstance(test.ops[0], ast.Eq)
               and isinstance(test.comparators[0], ast.Constant) and test.comparators[0].value == "__main__")
    calls = [s.value for s in guard.body if isinstance(s, ast.Expr) and isinstance(s.value, ast.Call)]
    return is_main and any(isinstance(c.func, ast.Name) and c.func.id == name and not c.args and not c.keywords
                           for c in calls)
