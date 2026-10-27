"""cadgen model access - the one place that knows what a model function is.

Every parts/<group>/<name>.py, assemblies/<name>.py and robot/links/<link>.py declares ONE
`@step(out="<name>.step")` model (cadgen.step) named after its file stem; running the file builds it
(writes the sibling STEP). Nothing in this module builds: `raw()` runs the decorated body in-process -
tests, tools, previews: no engine, nothing written - and `geometry()` is what an assembly body calls for a
child: the child model's result while a cadgen build is running on this thread, the raw body otherwise.
"""
from __future__ import annotations

import pathlib
from types import ModuleType


def is_model(obj) -> bool:
    """True for a cadgen model: a function under @step / @model / @stl ... (cadgen.engine.model.Model, which
    marks itself `__xbf_model__` and keeps the body as `.function`)."""
    return getattr(obj, "__xbf_model__", False) is True and callable(getattr(obj, "function", None))


def model_of(module: ModuleType, name: str | None = None):
    """The module's model: the attribute named after its file stem (or `name`)."""
    stem = name or pathlib.Path(module.__file__).stem
    fn = getattr(module, stem, None)
    if not is_model(fn):
        raise TypeError(f"{module.__name__}: expected `@step(out=\"{stem}.step\") def {stem}()` - a model named "
                        "after its file")
    return fn


def raw(model):
    """The model body's geometry built in this process: no engine, nothing written (`Model.function`)."""
    return model.function()


def building() -> bool:
    """True while a cadgen build runs on this thread (cadgen 0.8 keeps it on the private thread-local
    `cadgen.engine.model._CONTEXT.build`: docs/toolchain.md lists the private names this repo leans on)."""
    from cadgen.engine.model import _CONTEXT

    return getattr(_CONTEXT, "build", None) is not None


def geometry(model):
    """A child for a composing body.

    While a cadgen build runs on this thread the child model is CALLED: it runs as the build's child (replayed
    when its code and inputs are unchanged) and writes its own declared file - the part's git-ignored STEP - and
    its result comes back as a new placement the parent owns (re-colouring its leaves touches no other
    placement). Outside a build (tests, tools, previews) it is raw(): nothing starts a build."""
    if not building():
        return raw(model)
    return model().shape
