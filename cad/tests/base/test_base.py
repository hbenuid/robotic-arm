"""base, parametric (lib/base/): the LEGACY configuration reproduces the SolidWorks part feature by feature (the
volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression names
what moved), and the interface values lib/params.py and lib/datum.py take from it are the ones the rest of the arm
uses (the base_yaw motor's seat, the bottom face)."""
import pytest
from build123d import Location

import parts
from lib import mounts
from lib import params as PARAMS
from lib import placements as P
from lib.base import DEFAULT, LEGACY, chamfer_inset, motor_holes, side_stub_x
from lib.datum import BASE_BOTTOM_Y, to_location
from lib.motors import NEMA17_BOLT_SP, NEMA17_FACE
from tests.helpers import is_inside


def test_layout():
    assert len(motor_holes(LEGACY)) == 4
    assert side_stub_x(LEGACY) == pytest.approx(22.547607, abs=1e-6)       # the stubs' flat end (measured)
    assert chamfer_inset(LEGACY) == pytest.approx(10.0)
    m = LEGACY.motor
    assert m.slot_x1 == pytest.approx(m.centre[0] + m.rim_half)             # the belt slot runs to the rim's +X inside
    assert 2.0 * m.rim_half > NEMA17_FACE                                    # the rim takes the motor's face


def test_interface_values_come_from_the_base():
    assert PARAMS.BASE_MOTOR_PATTERN_CENTRE == (DEFAULT.motor.centre[0], DEFAULT.plate.y[0], DEFAULT.motor.centre[1])
    assert BASE_BOTTOM_Y == DEFAULT.shell.y0 == P.OCCURRENCES["base#1"]["world_bbox_min"][1]
    assert P.OCCURRENCES["base#1"]["world"]["position"] == [0.0, 0.0, 0.0]   # the part frame IS the capture frame


def test_motor_holes_are_the_base_yaw_motors_bolts():
    """The 48 mm motor (lib/mounts.py nema17_48mm#1: face on the plate's underside) has its 4 bolts on the NEMA 17
    square in its own frame (face z = 0): each one is a hole's axis."""
    frame = to_location(mounts.BY_KEY["nema17_48mm#1"].frame)
    half = NEMA17_BOLT_SP / 2.0
    bolts = [(frame * Location((sx * half, sy * half, 0.0))).position for sx in (1, -1) for sy in (1, -1)]
    assert sorted((round(p.X, 6), round(p.Z, 6)) for p in bolts) == sorted((round(x, 6), round(z, 6)) for x, z in motor_holes(DEFAULT))
    assert all(p.Y == pytest.approx(DEFAULT.plate.y[0]) for p in bolts)


@pytest.fixture(scope="module")
def legacy():
    from lib.base.body import build_base
    return build_base(LEGACY)


@pytest.fixture(scope="module")
def base():
    return parts.build("base")


@pytest.mark.slow
def test_legacy_features(legacy):
    leg = legacy
    assert leg.is_valid and len(leg.solids()) == 1
    assert is_inside(leg, -51, -60, 0) and not is_inside(leg, -47, -60, 0)           # the wall's round half, the cavity
    assert is_inside(leg, 112, -60, 30) and is_inside(leg, 50, -60, -51)             # the flat +X end, a straight side
    assert not is_inside(leg, 112, -90, 0) and is_inside(leg, 112, -90, 20)          # the cable notch
    assert not is_inside(leg, 112, -74, 0) and is_inside(leg, 112, -72, 0)           # ... up to notch_y1
    assert is_inside(leg, -51, -30, 0) and is_inside(leg, 21, -30, 51)               # above the plate: the round half, a stub
    assert not is_inside(leg, 24, -30, 51) and not is_inside(leg, 60, -30, 0)        # ... which stops; open over the plate
    assert is_inside(leg, 60, -42, 30) and not is_inside(leg, 0, -42, 0)             # the plate, its central opening
    assert not is_inside(leg, -30, -42, 36) and is_inside(leg, -7.8, -42, -44.3)     # open to the wall on -X; the sliver
    assert is_inside(leg, 45, -42, 0) and not is_inside(leg, 49.34, -42, 0)          # the curved slot ...
    assert not is_inside(leg, 34.3, -42, 35.46) and is_inside(leg, 30, -42, 40)      # ... its round end
    for x, z in motor_holes(LEGACY):
        assert not is_inside(leg, x, -42, z)                                         # the motor's holes
    assert not is_inside(leg, 79, -42, 15) and not is_inside(leg, 60, -42, 0)        # the window, the belt slot
    assert is_inside(leg, 52, -42, 0)                                                # the bridge: curved slot .. belt slot
    assert is_inside(leg, 101.3, -48, 0) and not is_inside(leg, 99, -48, 0)          # the rim, its pocket
    assert is_inside(leg, 50, -48, 22.5) and not is_inside(leg, 47, -48, 22.5)       # the rim's end on the outer round
    assert not is_inside(leg, 101.3, -52.5, 0)                                       # ... 7 deep
    assert is_inside(leg, -45, -20, 0) and not is_inside(leg, -43, -20, 0)           # the chamfer under the cap
    assert is_inside(leg, 50, -14, 0) and not is_inside(leg, 50, -15, 0)             # the cap's flat underside on +X
    assert is_inside(leg, 0, -20, 31) and not is_inside(leg, 0, -20, 32.2)           # the boss
    assert not is_inside(leg, 0, -10, 21) and is_inside(leg, 0, -10, 21.4)           # the upper seat
    assert is_inside(leg, 0, -13.8, 16.2) and not is_inside(leg, 0, -13.8, 15.6)     # the lip
    assert not is_inside(leg, 0, -18, 21.5) and is_inside(leg, 0, -18, 22)           # the lower seat
    assert is_inside(leg, 0, -7, 30) and not is_inside(leg, 0, -7, 40)               # the ring, the groove
    assert is_inside(leg, 0, -5.3, 30) and not is_inside(leg, 0, -5.3, 50)           # the ring proud of the top face
    assert is_inside(leg, 0, -5.7, 50)


@pytest.mark.slow
def test_default_is_legacy(base, legacy):
    assert DEFAULT == LEGACY
    assert base.volume == pytest.approx(legacy.volume, abs=1e-6)
