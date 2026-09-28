"""base, parametric (lib/base/): the LEGACY configuration reproduces the SolidWorks part feature by feature (the
volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression names
what moved), the interface values lib/params.py and lib/datum.py take from it are the ones the rest of the arm
uses (the base_yaw motor's seat, the bottom face), and DEFAULT - what the part builds - changes the bearing bore (the
6806-2RS pair's seats and lip, the wrist's) and the seat ring (inside the thrust bearing's bore), and cuts the motor
lobe off at the joint face (base_motor_mount, its seat slotted where the stock belt puts the motor:
tests/base/test_motor_mount.py)."""
from dataclasses import replace

import pytest
from build123d import Location

import parts
from lib import mounts
from lib import params as PARAMS
from lib import placements as P
from lib.base import DEFAULT, LEGACY, chamfer_inset, motor_holes, side_stub_x
from lib.base.params import RING_CLEAR
from lib.bearings import BEARING_6806_OD, BEARING_6806_WIDTH, THRUST_BORE, THRUST_OD
from lib.datum import BASE_BOTTOM_Y, to_location
from lib.forearm import DEFAULT as FOREARM
from lib.motors import NEMA17_BOLT_SP, NEMA17_FACE
from tests.helpers import is_inside


def test_layout():
    assert len(motor_holes(LEGACY)) == 4
    assert side_stub_x(LEGACY) == pytest.approx(22.547607, abs=1e-6)       # the stubs' flat end (measured)
    assert chamfer_inset(LEGACY) == pytest.approx(10.0)
    m = LEGACY.motor
    assert m.slot_x1 == pytest.approx(m.centre[0] + m.rim_half)             # the belt slot runs to the rim's +X inside
    assert 2.0 * m.rim_half > NEMA17_FACE                                    # the rim takes the motor's face
    d = DEFAULT.motor                                                        # ... the slots' far end with DEFAULT's travel
    assert d.slot_x1 == pytest.approx(d.centre[0] + d.rim_half + d.travel) and LEGACY.motor.travel == 0.0


def test_interface_values_come_from_the_base():
    assert PARAMS.BASE_MOTOR_PATTERN_CENTRE == (DEFAULT.motor.centre[0], DEFAULT.plate.y[0], DEFAULT.motor.centre[1])
    assert BASE_BOTTOM_Y == DEFAULT.shell.y0 == P.OCCURRENCES["base#1"]["world_bbox_min"][1]
    assert P.OCCURRENCES["base#1"]["world"]["position"] == [0.0, 0.0, 0.0]   # the part frame IS the capture frame


def test_motor_holes_are_the_base_yaw_motors_bolts():
    """The 48 mm motor (lib/mounts.py nema17_48mm#1: face on the motor mount's plate's underside, in the base's frame)
    has its 4 bolts on the NEMA 17 square in its own frame (face z = 0): each one is a hole's axis - the slots' middle."""
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


def test_default_bore_takes_the_6806_pair():
    b = DEFAULT.bore
    assert replace(DEFAULT, bore=LEGACY.bore, cap=LEGACY.cap, motor=LEGACY.motor, joint=None) == LEGACY
    assert (b.upper_dia, b.lower_dia, b.lip_dia) == (FOREARM.boss.seat_dia, FOREARM.boss.seat_dia, FOREARM.boss.lip_dia)
    assert b.upper_dia > BEARING_6806_OD and LEGACY.bore.lower_dia - BEARING_6806_OD > 1.0   # the SolidWorks lower seat: 1.4 over
    c = DEFAULT.cap
    assert c.ring_top_y - b.lip_y[1] > BEARING_6806_WIDTH and b.lip_y[0] - c.boss_y0 > BEARING_6806_WIDTH   # a bearing fits each side


def test_default_ring_centres_the_thrust_bearing():
    """The seat ring inside the thrust bearing's bore (the SolidWorks Ø65.1 was over it), the groove round the stack."""
    ring, groove = DEFAULT.cap.groove_r
    assert DEFAULT.cap == replace(LEGACY.cap, groove_r=(ring, LEGACY.cap.groove_r[1]))
    assert THRUST_BORE / 2.0 - ring == pytest.approx(RING_CLEAR) and LEGACY.cap.groove_r[0] > THRUST_BORE / 2.0
    assert groove > THRUST_OD / 2.0


@pytest.mark.slow
def test_default_changes_the_bore_and_the_ring_and_ends_at_the_joint(base, legacy):
    assert base.is_valid and len(base.solids()) == 1
    assert tuple(base.bounding_box().min) == pytest.approx(tuple(legacy.bounding_box().min), abs=1e-6)
    lmax = legacy.bounding_box().max
    assert tuple(base.bounding_box().max) == pytest.approx((DEFAULT.joint.split_x, lmax.Y, lmax.Z), abs=1e-6)   # the lobe is the mount's
    assert is_inside(base, 0, -10, 21.15) and not is_inside(legacy, 0, -10, 21.15)   # the upper seat Ø42.4 -> Ø42.2
    assert is_inside(base, 0, -18, 21.5) and not is_inside(legacy, 0, -18, 21.5)     # the lower seat Ø43.4 -> Ø42.2
    assert not is_inside(base, 0, -13.8, 17) and is_inside(legacy, 0, -13.8, 17)     # the lip Ø31.73 -> Ø37.65
    assert is_inside(base, 0, -13.8, 19) and not is_inside(base, 0, -10, 21.05)
    assert not is_inside(base, 0, -7, 32.45) and is_inside(legacy, 0, -7, 32.45)     # the ring Ø65.1 -> Ø64.8
    assert is_inside(base, 0, -7, 32.3)
    assert is_inside(base, 52, -42, 30) and is_inside(legacy, 52, -42, 30)             # the plate's neck as before
    assert is_inside(base, 54, -42, 0) and not is_inside(legacy, 54, -42, 0)          # its belt slot is the mount's
