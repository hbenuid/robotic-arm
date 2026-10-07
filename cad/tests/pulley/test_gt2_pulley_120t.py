"""gt2_pulley_120t, the base_yaw joint's driven pulley (lib/pulley/params.py YAW): a part with no reference file
(lib/reference.py NO_REFERENCE), so THIS file locks its numbers - the volume and the bounding box, measured on the build
and written in - and what it is: the 90T's DEFAULT with 120 teeth, its rim grown with them and the 90T's wall kept under
them, every other feature - the hub, the web's height, the bore, the bolt holes and their counterbores - the 90T's,
probed by name."""
import math
from dataclasses import replace

import pytest

from lib import belts
from lib import params as PARAMS
from lib import reference as R
from lib.pulley import DEFAULT, YAW
from tests import built
from tests.helpers import is_inside
from tests.pulley.helpers import at

VOLUME = 37067.892                                   # mm^3, the build's
BBOX_MIN = (-39.143186, -13.2, -39.143186)
BBOX_SIZE = (78.286373, 21.4, 78.286373)


def test_params():
    assert YAW.teeth == belts.GT2_PULLEY_120T_TEETH == 120
    assert YAW == replace(DEFAULT, teeth=YAW.teeth, rim_dia=YAW.rim_dia)          # nothing else differs from the 90T's
    assert belts.pulley_od(YAW.teeth) - YAW.rim_dia == pytest.approx(belts.pulley_od(DEFAULT.teeth) - DEFAULT.rim_dia, abs=1e-6)
    assert YAW.flange_dia == pytest.approx(78.286, abs=1e-3) and YAW.flange_dia == pytest.approx(BBOX_SIZE[0], abs=1e-6)
    assert R.NO_REFERENCE["gt2_pulley_120t"] == "lib/pulley/body.py:build_pulley(YAW)"
    assert PARAMS.BASE_YAW_RATIO == YAW.teeth / PARAMS.GT2_PULLEY_20T_TEETH


@pytest.fixture(scope="module")
def pulley():
    return built.part("gt2_pulley_120t")


@pytest.mark.slow
def test_the_build_matches_its_numbers(pulley):
    assert pulley.is_valid and len(pulley.solids()) == 1
    assert R.solid_volume(pulley) == pytest.approx(VOLUME, abs=0.01)
    assert R.bbox_min(pulley) == pytest.approx(BBOX_MIN, abs=1e-3) and R.bbox_size(pulley) == pytest.approx(BBOX_SIZE, abs=1e-3)


@pytest.mark.slow
def test_the_hub_web_and_holes_are_the_90ts(pulley):
    """The same probes as the 90T's (tests/pulley/test_gt2_pulley_90t.py), off the axes where the bolts are."""
    p = pulley
    assert is_inside(p, *at(14.8, -10.0)) and not is_inside(p, *at(15.2, -10.0))            # the journal
    assert is_inside(p, *at(17.2, -5.5)) and not is_inside(p, *at(17.2, -6.4))              # the ring ...
    assert not is_inside(p, *at(17.2, -4.5)) and not is_inside(p, *at(17.5, -5.5))          # ... Ø34.76, 1.5 high
    assert is_inside(p, *at(17.3, 1.3)) and not is_inside(p, *at(17.7, 1.3))                # the step
    assert not is_inside(p, *at(30.0, 1.6)) and is_inside(p, *at(30.0, 2.0))                # open under the web, farther out
    assert not is_inside(p, *at(6.0, 0.0)) and is_inside(p, *at(6.5, 0.0))                  # the bore
    for x, z in YAW.bolt_points():
        u = (x / YAW.bolt_r, z / YAW.bolt_r)
        for y in (YAW.end_y + 0.5, 0.0, YAW.face_y - YAW.counterbore[1] - 0.5):            # M4 clearance, end to counterbore
            assert not is_inside(p, x + 2.1 * u[0], y, z + 2.1 * u[1]) and is_inside(p, x + 2.3 * u[0], y, z + 2.3 * u[1])
        for y in (YAW.face_y - YAW.counterbore[1] + 0.5, YAW.face_y - 0.5):                # the head's counterbore, Ø7.4
            assert not is_inside(p, x + 3.6 * u[0], y, z + 3.6 * u[1]) and is_inside(p, x + 3.8 * u[0], y, z + 3.8 * u[1])


@pytest.mark.slow
def test_the_rim_carries_120_teeth(pulley):
    p, r_land = pulley, belts.pulley_od(YAW.teeth) / 2.0
    r_in = YAW.rim_dia / 2.0
    assert is_inside(p, *at(r_in + 0.2, -1.0)) and not is_inside(p, *at(r_in - 0.2, -1.0))  # the rim's inside
    r_flange = YAW.flange_dia / 2.0
    assert is_inside(p, *at(r_flange - 0.1, -0.6)) and not is_inside(p, *at(r_flange + 0.1, -0.6))   # the lower flange
    assert is_inside(p, *at(r_flange - 0.1, 7.8)) and not is_inside(p, *at(r_flange - 0.1, 7.1))    # the upper, chamfered
    pitch = 360.0 / YAW.teeth
    for k in range(YAW.teeth):                                                              # a groove every pitch ...
        a = k * pitch
        assert not is_inside(p, *at(r_land - 0.5, 3.5, a)) and is_inside(p, *at(r_land - 0.9, 3.5, a)), k
        assert is_inside(p, *at(r_land - 0.1, 3.5, a + pitch / 2.0)), k                     # ... a land between two
    assert not is_inside(p, *at(r_land + 0.1, 3.5, pitch / 2.0))
    assert math.isclose(r_land, 37.943186, abs_tol=1e-5)
