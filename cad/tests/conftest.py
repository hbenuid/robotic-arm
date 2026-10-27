"""Session guards for the CAD suite.

Tests call model BODIES (lib.models.raw / parts.build), never a model: calling a `@step` model
outside a cadgen build runs it through the engine - it starts this venv's engine and rewrites the part
STEP. `Model.build` (what such a call runs) is replaced so an accidental top-level call fails loudly instead.

The shapes tests/built.py caches are shared by every later test: a test that changed one (its label, colour, tree or
location) fails at its teardown.
"""
import sys

import pytest


@pytest.fixture(autouse=True)
def _shared_shapes_stay_as_built():
    yield
    built = sys.modules.get("tests.built")
    changed = built.changed() if built is not None else []
    assert not changed, "this test changed a shape tests/built.py shares (they are read-only):\n" + "\n".join(changed)


@pytest.fixture(autouse=True, scope="session")
def _no_top_level_builds():
    from cadgen.engine.model import Model

    def refuse(self, **_):
        raise RuntimeError(
            f"{self.script.name}::{self.name} called at top level under pytest - use "
            "lib.models.raw(model) / parts.build(name); builds are `./cadtool gen <model.py>`"
        )

    original, Model.build = Model.build, refuse
    yield
    Model.build = original
