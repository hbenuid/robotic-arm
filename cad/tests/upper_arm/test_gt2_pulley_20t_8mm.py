"""gt2_pulley_20t_8mm, the elbow's 20T on its gearbox's Ø8 shaft: a purchased part with no reference (lib/reference.py
NO_REFERENCE) - the 5 mm 20T's vendor geometry (vendor/gt2_pulley_20t.step) bored out to the gearbox's shaft. THIS file
locks its numbers (measured on the build and written in) and what it is: the 5 mm pulley's outside, an 8 mm bore, its
set screws' tips on the bore. Its place on the shaft - the band level with the 90T's, a set screw over the keyway - is
tests/test_mounts.py."""
import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from tests import built
from tests.helpers import is_inside

NAME = "gt2_pulley_20t_8mm"
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers - the body and its two set screws
LOCK = (3, 1452.551, (0.0, -8.0, -8.0), (14.45, 16.0, 16.0))


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_it_is_a_purchased_part_with_no_reference_in_the_joints_group():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "joints" and parts.bought(NAME) and mod.PURCHASE_QTY == 1
    assert R.NO_REFERENCE[NAME] == f"parts/joints/{NAME}.py:_envelope()"
    assert mod.BORE == PARAMS.EG17_G5.shaft_dia == 8.0 and mod.PURCHASE_SPEC == "GT2 20T timing pulley, 8 mm bore, 6 mm belt"
    assert mod.FIVE_MM == parts.load("gt2_pulley_20t").VENDOR_STEP and mod.FIVE_MM.exists()


@pytest.mark.slow
def test_the_build_matches_its_numbers():
    solids, volume, lo, size = LOCK
    shape = built.part(NAME)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)


@pytest.mark.slow
def test_the_5mm_pulley_bored_out():
    """The same outside as the 5 mm pulley (its bounding box), less the ring between the two bores along the whole pulley;
    the bore Ø8 through, the hub and the tooth band's root solid round it; the set screws (in the hub along +Y and +Z,
    4 from its face) end on the bore."""
    five, eight = built.part("gt2_pulley_20t"), built.part(NAME)
    assert R.bbox_min(eight) == pytest.approx(R.bbox_min(five), abs=1e-3) and R.bbox_size(eight) == pytest.approx(R.bbox_size(five), abs=1e-3)
    for x in (1.0, 6.0, 10.0, 14.0):
        assert not _inside(eight, x, 0.0, 3.9) and not _inside(eight, x, 2.6, 0.0)          # the Ø8 bore (the 5 mm one solid)
        assert _inside(five, x, 0.0, 3.0) or x < 0.5
        assert _inside(eight, x, 0.0, -4.2) and _inside(eight, x, -4.2, 0.0)                # solid round it
    for y, z in ((4.2, 0.0), (0.0, 4.2)):
        assert _inside(eight, 4.0, y, z) and not _inside(eight, 4.0, y * 0.9, z * 0.9)      # a set screw's tip on the bore
