"""gt2_pulley_90t_wrist, the wrist belt's driven pulley (lib/pulley/params.py WRIST): a part with no reference file
(lib/reference.py NO_REFERENCE), so THIS file locks its numbers - the volume and the bounding box, measured on the build
and written in - and what it is: the elbow's 90T (DEFAULT) with its 4 head counterbores GT2_PULLEY_90T_WRIST_HEAD_SEAT
deep, 2 shallower, every other feature the 90T's."""
import math
from dataclasses import replace

import pytest

from lib import belts
from lib import reference as R
from lib.fasteners import M4_HEAD_BORE_FINE
from lib.pulley import DEFAULT, WRIST
from tests import built
from tests.helpers import is_inside

VOLUME = 25742.688                                   # mm^3, the build's
BBOX_MIN = (-29.59389, -13.2, -29.59389)
BBOX_SIZE = (59.18778, 21.4, 59.18778)


def test_params():
    assert WRIST == replace(DEFAULT, counterbore=(M4_HEAD_BORE_FINE, belts.GT2_PULLEY_90T_WRIST_HEAD_SEAT))   # nothing else differs
    assert DEFAULT.counterbore[1] - WRIST.counterbore[1] == 2.0
    assert R.NO_REFERENCE["gt2_pulley_90t_wrist"] == "lib/pulley/body.py:build_pulley(WRIST)"


@pytest.fixture(scope="module")
def pulley():
    return built.part("gt2_pulley_90t_wrist")


@pytest.mark.slow
def test_the_build_matches_its_numbers(pulley):
    assert pulley.is_valid and len(pulley.solids()) == 1
    assert R.solid_volume(pulley) == pytest.approx(VOLUME, abs=0.01)
    assert R.bbox_min(pulley) == pytest.approx(BBOX_MIN, abs=1e-3) and R.bbox_size(pulley) == pytest.approx(BBOX_SIZE, abs=1e-3)


@pytest.mark.slow
def test_only_the_counterbores_differ_from_the_90t(pulley):
    """The elbow's 90T less 2 of each counterbore: the same box, the volume 4 counterbore rings 2 deep larger; at each
    bolt the head's bore down to GT2_PULLEY_90T_WRIST_HEAD_SEAT, the shank's hole below it."""
    elbow = built.part("gt2_pulley_90t")
    (cb_dia, depth), r_hole = WRIST.counterbore, WRIST.hole_dia / 2.0
    rings = 4 * math.pi * ((cb_dia / 2.0) ** 2 - r_hole ** 2) * (DEFAULT.counterbore[1] - depth)
    assert R.solid_volume(pulley) - R.solid_volume(elbow) == pytest.approx(rings, abs=1e-3)
    assert R.bbox_min(pulley) == pytest.approx(R.bbox_min(elbow), abs=1e-6)
    assert R.bbox_size(pulley) == pytest.approx(R.bbox_size(elbow), abs=1e-6)
    floor = WRIST.face_y - depth
    for x, z in WRIST.bolt_points():
        u = (x / WRIST.bolt_r, z / WRIST.bolt_r)
        for y in (floor + 0.5, WRIST.face_y - 0.5):                                       # the head's bore, Ø7.05
            assert not is_inside(pulley, x + 3.475 * u[0], y, z + 3.475 * u[1]) and is_inside(pulley, x + 3.575 * u[0], y, z + 3.575 * u[1])
        for y in (floor - 0.5, DEFAULT.face_y - DEFAULT.counterbore[1] + 0.5):           # under it, the shank's hole, Ø4.2 ...
            assert not is_inside(pulley, x + 2.05 * u[0], y, z + 2.05 * u[1]) and is_inside(pulley, x + 2.15 * u[0], y, z + 2.15 * u[1])
            assert is_inside(pulley, x + 3.0 * u[0], y, z + 3.0 * u[1])                  # ... where the 90T's bore was
