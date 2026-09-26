"""j1_link, parametric (lib/upper_arm/): the LEGACY configuration reproduces the SolidWorks part feature by feature
(the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression
names what moved), and DEFAULT - what the part builds - changes exactly four things: no cap sockets, the NEMA 17
holes on the motor's pattern, the hub holes on the cycloidal drive's bolts, and the elbow block's clearance (the
relief round the elbow axis, the recess floor deeper)."""
import math
from dataclasses import replace

import pytest
from build123d import Location, Rot

import parts
from lib import mounts
from lib import params as PARAMS
from lib import placements as P
from lib.cycloidal.layout import arm_mount_points
from lib.cycloidal.params import DEFAULT_CONFIG
from lib.datum import to_location
from lib.motors import NEMA17_BOLT_SP
from lib.upper_arm import DEFAULT, LEGACY, hub_bolt_points, pad_holes, socket_points
from tests.helpers import interference, is_inside


def _in_link(key: str, local=(0.0, 0.0, 0.0)) -> Location:
    """A point of occurrence `key` (in its own frame) in j1_link#1's frame."""
    return P.location("j1_link#1", "world").inverse() * P.location(key, "world") * Location(local)


def test_layout():
    assert len(hub_bolt_points(LEGACY)) == len(hub_bolt_points(DEFAULT)) == 4
    assert [len(s) for s in socket_points(LEGACY)] == [7, 3] and socket_points(DEFAULT) == ([], [])
    assert DEFAULT.slab == LEGACY.slab and DEFAULT.slots == LEGACY.slots
    assert LEGACY.elbow.relief_r == 0.0 and DEFAULT.elbow.relief_r > 0.0 and DEFAULT.elbow.recess_y < LEGACY.elbow.recess_y
    assert replace(DEFAULT.elbow, relief_r=LEGACY.elbow.relief_r, relief_y=LEGACY.elbow.relief_y,
                   recess_y=LEGACY.elbow.recess_y) == LEGACY.elbow
    assert PARAMS.J1_MOTOR_PAD_FACE_Y == DEFAULT.pad.face_y == -32.5


def test_default_pad_holes_are_the_motors_pattern():
    """The elbow motor (lib/mounts.py nema17_40mm#2: face on the pad, shaft +N on the shoulder axis) has its 4 bolts
    on the NEMA 17 square in its own frame (face z = 0): each one is a DEFAULT hole's axis."""
    m = mounts.BY_KEY["nema17_40mm#2"]
    frame = to_location(m.frame)
    half = NEMA17_BOLT_SP / 2.0
    bolts = sorted((round(p.X, 6), round(p.Z, 6)) for p in (
        (frame * Location((sx * half, sy * half, 0.0))).position for sx in (1, -1) for sy in (1, -1)))
    holes = sorted((round(x, 6), round(z, 6)) for x, z, _ in pad_holes(DEFAULT))
    assert bolts == holes
    assert all((frame * Location((sx * half, sy * half, 0.0))).position.Y == pytest.approx(DEFAULT.pad.face_y)
               for sx in (1, -1) for sy in (1, -1))
    # LEGACY's holes were 0.38 off the axis and uneven - the reason DEFAULT moves them
    cx = sum(x for x, _, _ in pad_holes(LEGACY)) / 4.0
    cz = sum(z for _, z, _ in pad_holes(LEGACY)) / 4.0
    assert 0.35 < math.hypot(cx, cz) < 0.4


def test_default_hub_holes_are_the_drives_bolts():
    """The drive's 4 arm-mount bolts (lib/cycloidal/layout.py arm_mount_points on the hub's arm-mount face), placed by
    placements.json, run along -Y through j1_link on DEFAULT's holes (the [REFERENCE] bolt angle re-derived)."""
    face_z = PARAMS.CYCLOIDAL_OUTPUT_FACE_Z
    holes = hub_bolt_points(DEFAULT)
    for x, y in arm_mount_points(DEFAULT_CONFIG):
        head = _in_link("cycloidal_drive#1", (x, y, face_z)).position
        tip = _in_link("cycloidal_drive#1", (x, y, face_z + 10.0)).position
        assert abs(abs((tip - head).Y) - 10.0) < 1e-6                              # along the link's Y
        assert head.Y == pytest.approx(DEFAULT.slab.lip_top, abs=1e-5)            # the hub face on the top face
        assert min(math.hypot(head.X - hx, head.Z - hz) for hx, hz in holes) < 1e-4
    # the SolidWorks holes missed them by 3.36 degrees (1.47 mm at r 25: an M4 does not pass a Ø4.4 hole)
    assert DEFAULT.hub.bolt_angle_deg - LEGACY.hub.bolt_angle_deg == pytest.approx(-3.359167)
    assert DEFAULT_CONFIG.output_hub.arm_mount_bolt_circle_dia == DEFAULT.hub.bolt_circle_dia


@pytest.fixture(scope="module")
def legacy():
    from lib.upper_arm.link import build_link
    return build_link(LEGACY)


@pytest.fixture(scope="module")
def link():
    return parts.build("j1_link")


@pytest.mark.slow
def test_legacy_features(legacy):
    leg = legacy
    assert leg.is_valid and len(leg.solids()) == 1
    assert is_inside(leg, 50, -5, 0) and is_inside(leg, 50, 1.0, 0)                  # the plate and its lip
    assert not is_inside(leg, 50, 1.0, 44.75) and is_inside(leg, 50, -1.0, 44.75)     # the lip is inset 0.5
    assert not is_inside(leg, 50, -12.5, 0) and is_inside(leg, 50, -11.5, 0)          # the shoulder half's underside
    assert is_inside(leg, 149.5, -15.5, 30) and not is_inside(leg, 149.5, -16.0, 30)  # the 45 degree chamfer
    assert is_inside(leg, 180, -23.5, 30) and not is_inside(leg, 150, -23.5, 30)      # the elbow half's step down
    assert not is_inside(leg, 0, -5, 0) and is_inside(leg, -22, -5, 0)                # the square opening
    for x, z in hub_bolt_points(LEGACY):
        assert not is_inside(leg, x, -5, z)                                           # the hub bolts
    assert is_inside(leg, 20, -32, 20) and not is_inside(leg, 20, -33, 20)            # the pad's face
    assert not is_inside(leg, 0, -28, 0) and is_inside(leg, -16, -28, 0)              # the pilot opening, the -X floor
    assert not is_inside(leg, 18, -28, 0)                                             # the +X window through the floor
    assert not is_inside(leg, 0, -20, 15) and is_inside(leg, 15, -20, 21)             # the cavity, a Z wall
    assert not is_inside(leg, 0, -20, 21) and not is_inside(leg, -22, -20, 0)         # the Z and -X windows
    assert is_inside(leg, -22, -20, 15)                                               # the -X wall beside its window
    for x, z, _ in pad_holes(LEGACY):
        assert not is_inside(leg, x, -28, z)                                          # the NEMA 17 holes
    assert is_inside(leg, 25.5, -12.5, 20) and not is_inside(leg, 25.5, -14.0, 20)    # the root flare
    assert is_inside(leg, -20.5, -9, 15) and not is_inside(leg, -20.5, -4, 15)        # the -X cove
    assert is_inside(leg, 15, -10.5, 20.5) and not is_inside(leg, 15, -9, 20.5)       # a Z cove
    assert not is_inside(leg, 128, -3, 10) and is_inside(leg, 128, -7, 6)             # x 128: the top seat, the web ...
    assert not is_inside(leg, 128, -7, 0)                                             # ... its hole ...
    assert is_inside(leg, 128, -14, 15) and not is_inside(leg, 128, -14, 21)          # ... the boss round the lower seat
    assert not is_inside(leg, 70.5, -5, 0) and not is_inside(leg, 100.5, -5, 25)      # the through slots ...
    assert is_inside(leg, 70.5, -5, 27)                                               # ... end at z +/- 26
    assert not is_inside(leg, 163, 0, 0) and not is_inside(leg, 160, -15, 0)          # the stepped slot and its counterbore
    assert is_inside(leg, 160, -5, 0)
    assert not is_inside(leg, 210, 0, 39) and is_inside(leg, 210, -6, 30)             # the elbow: recess ...
    assert not is_inside(leg, 210, -6, 20) and is_inside(leg, 210, -14, 20)           # ... bore, lip ...
    assert not is_inside(leg, 210, -20, 21)                                           # ... seat
    shoulder, elbow = socket_points(LEGACY)
    for x, z in shoulder:
        assert not is_inside(leg, x, -11.0, z) and is_inside(leg, x, -9.5, z)         # the cap's sockets
    for x, z in elbow:
        assert not is_inside(leg, x, -23.0, z) and is_inside(leg, x, -21.5, z)


@pytest.mark.slow
def test_default_changes_only_the_holes_the_sockets_and_the_elbow(link, legacy):
    from lib.upper_arm.link import _bore

    assert link.is_valid and len(link.solids()) == 1
    assert tuple(link.bounding_box().min) == pytest.approx(tuple(legacy.bounding_box().min), abs=1e-6)
    assert tuple(link.bounding_box().max) == pytest.approx(tuple(legacy.bounding_box().max), abs=1e-6)
    shoulder, elbow = socket_points(LEGACY)
    for x, z in shoulder:
        assert is_inside(link, x, -11.0, z)                                           # no sockets
    for x, z in elbow:
        assert is_inside(link, x, -23.0, z)
    for x, z, _ in pad_holes(DEFAULT):
        assert not is_inside(link, x, -28, z)
    for x, z in hub_bolt_points(DEFAULT):
        assert not is_inside(link, x, -5, z)
    # the elbow: the top face down to relief_y within relief_r of the axis, the lip still there beyond; the recess floor
    s, e = DEFAULT.slab, DEFAULT.elbow
    x_edge = s.elbow_x - e.relief_r
    assert not is_inside(link, x_edge + 1.0, e.relief_y + 0.5, 0) and is_inside(link, x_edge + 1.0, e.relief_y - 0.5, 0)
    assert is_inside(link, x_edge - 1.0, s.y1 + 1.0, 0) and is_inside(legacy, x_edge + 1.0, s.y1 + 1.0, 0)
    r_floor = (e.recess_dia + e.bore_dia) / 4.0
    assert not is_inside(link, s.elbow_x, e.recess_y + 0.5, r_floor) and is_inside(link, s.elbow_x, e.recess_y - 0.5, r_floor)
    # the 10 sockets filled (+416.6 mm^3), the fourth NEMA hole Ø3.0 -> Ø3.2 through the 9 mm floor (-8.8); moving
    # the holes changes nothing else; the relief takes the lip inside relief_r, the recess the ring between the bore
    # and the recess 1 mm deeper
    floor = DEFAULT.pad.floor_y - DEFAULT.pad.face_y
    relief = interference(legacy, Rot(-90.0, 0.0, 0.0) * _bore(e.relief_r, e.relief_y, s.lip_top + 1.0, s.elbow_x))
    ring = math.pi * ((e.recess_dia / 2.0) ** 2 - (e.bore_dia / 2.0) ** 2) * (LEGACY.elbow.recess_y - e.recess_y)
    assert relief > 0.0
    assert link.volume - legacy.volume == pytest.approx(10 * math.pi * 2.575 ** 2 * 2.0 - floor * math.pi * (1.6 ** 2 - 1.5 ** 2)
                                                        - relief - ring, abs=0.5)
