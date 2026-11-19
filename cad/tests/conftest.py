"""Session guards for the CAD suite.

Tests call model BODIES (lib.models.raw / parts.build), never a model: calling a `@step` model
outside a cad build runs it through the machine's server and rewrites the part
STEP. `Model.build` (what such a call runs) is replaced so an accidental top-level call fails loudly instead.

The shapes tests/built.py caches are shared by every later test: a test that changed one (its label, colour, tree or
location) fails at its teardown.

A test process reads its STEPs through one reader process, as a build does (`_one_step_reader`).
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
def _one_step_reader():
    """Every STEP the test process reads (a vendor part's, through lib/cots.hybrid) parsed by one reader process, as a
    build's are (cad.engine.reader.session) - outside a build cad starts a reader for each read and stops it after: a
    Python started and the kernel imported for every STEP. It starts on the first read, so a run that reads none pays
    nothing."""
    from cad.engine.reader import session

    with session():
        yield


@pytest.fixture(autouse=True, scope="session")
def _no_top_level_builds():
    from cad.engine.model import Model

    def refuse(self, **_):
        raise RuntimeError(
            f"{self.script.name}::{self.name} called at top level under pytest - use "
            "lib.models.raw(model) / parts.build(name); builds are `./cadtool gen <model.py>`"
        )

    original, Model.build = Model.build, refuse
    yield
    Model.build = original
