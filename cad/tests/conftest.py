"""Session guards for the CAD suite.

Tests call model BODIES (lib.models.raw / parts.build), never a model: calling a `@step` model
outside a cadgen build runs the whole pipeline - it rewrites the part STEP and talks to the
warm daemon. CADGEN_DAEMON=0 keeps any build transient (belt) and `_build` is replaced so an
accidental top-level call fails loudly instead (braces).
"""
import os

import pytest

os.environ.setdefault("CADGEN_DAEMON", "0")


@pytest.fixture(autouse=True, scope="session")
def _no_top_level_builds():
    from cadgen import authoring

    def refuse(defn):
        raise RuntimeError(
            f"{defn.script_path.name}::{defn.func.__name__} called at top level under pytest - use "
            "lib.models.raw(model) / parts.build(name); builds are `./cadtool gen <model.py>`"
        )

    original, authoring._build = authoring._build, refuse
    yield
    authoring._build = original
