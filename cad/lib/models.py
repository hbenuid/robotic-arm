"""cadgen model access - the one place that knows what a model function is.

Every parts/<group>/<name>.py, assemblies/<name>.py and robot/links/<link>.py declares ONE
`@step` model (cadgen.step) named after its file stem; running the file builds it (writes the
sibling STEP). Nothing in this module builds: `raw()` runs the decorated body in-process - tests,
tools, previews: no freshness gate, no store, nothing written - and `geometry()` is what an
assembly body calls for a child: the linked child while a cadgen build is running on this thread,
the raw body otherwise.
"""
from __future__ import annotations

import pathlib
from types import ModuleType


def is_model(obj) -> bool:
    """True for a function carrying cadgen's @step / @stl / @glb / @threemf decorator."""
    return callable(obj) and callable(getattr(getattr(obj, "__cadgen_model__", None), "func", None))


def model_of(module: ModuleType, name: str | None = None):
    """The module's model: the attribute named after its file stem (or `name`)."""
    stem = name or pathlib.Path(module.__file__).stem
    fn = getattr(module, stem, None)
    if not is_model(fn):
        raise TypeError(f"{module.__name__}: expected `@step def {stem}()` - a model named after its file")
    return fn


def raw(model):
    """The model body's geometry built in this process: no gate, no store, nothing written. The body is the
    model's `ModelDef.func` (cadgen 0.7.5 took `__wrapped__` off the wrapper; reaching the body through
    `__cadgen_model__` makes cadgen hash the child's file as the caller's source)."""
    return model.__cadgen_model__.func()


def geometry(model, *, inline: bool = False):
    """A child for a composing body.

    While a cadgen build runs on this thread the child model is CALLED: its job is submitted
    (built in parallel, its own outputs - the part's git-ignored STEP - rewritten when stale) and the
    parent pins its result. By default the call's LazyCompound is returned: deferred placement,
    and the parent's STEP links the child's tree. With `inline=True` the parent gets the body
    built in-process instead (a copy it owns - needed to recolour the leaves: a linked child keeps
    its own colours) while the call still pins and rebuilds the child. Outside a build (tests,
    tools, previews) both are raw(): nothing starts a build."""
    from cadgen.authoring import build_in_progress

    if not build_in_progress():
        return raw(model)
    linked = model()
    return raw(model) if inline else linked
