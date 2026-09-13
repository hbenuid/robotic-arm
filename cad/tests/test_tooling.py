"""The local tooling stays healthy: the installed cadgen runtime carries the patch its URDF renderer
needs (tools/cadgen_patches.py) - without it the viewer and the snapshots draw robots as a pile."""
import pytest

from tools import cadgen_patches


def test_cadgen_urdf_render_patch_applied():
    states = cadgen_patches.check()
    assert states, "no patches defined"
    for name, per_bundle in states.items():
        assert per_bundle, f"{name}: no cadgen runtime bundle found under {cadgen_patches.runtime_dir()}"
        for bundle, state in per_bundle.items():
            if state == cadgen_patches.NOT_APPLICABLE:
                pytest.skip(f"{name}: {bundle} has neither the patch nor its anchor (cadgen "
                            f"{cadgen_patches.cadgen_version()}: bundle changed or fixed upstream) - check that "
                            "`./cadtool snapshot robot/arm.urdf` renders an arm, then retire or update the patch")
            assert state == cadgen_patches.PATCHED, f"{name}: {bundle} is {state} - run ./cadtool patch"
