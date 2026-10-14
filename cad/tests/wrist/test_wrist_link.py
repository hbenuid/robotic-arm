"""wrist_link, parametric (lib/wrist/): the LEGACY configuration reproduces the SolidWorks part feature by feature (the
volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression names
what moved), its seat is j3_coupler's flange (lib/coupler/params.py) and its end face the NEMA 17 pattern, and DEFAULT
- what the part builds - is the plate's round end with that seat and, on its underside, the wrist roll drive's frame
(the drive itself: tests/wrist/test_roll_drive.py)."""
import math
from dataclasses import replace

import pytest

from lib.coupler import LEGACY as COUPLER
from lib.motors import NEMA17_BOLT_SP
from lib.wrist import DEFAULT, LEGACY, end_face_holes, seat_bolt_points, slope_x
from lib.wrist.layout import module_frame_in_host, stack_positions
from tests import built
from tests.helpers import is_inside

P, S, T, E = LEGACY.plate, LEGACY.seat, LEGACY.tower, LEGACY.end_face


def test_the_seat_is_the_couplers_flange():
    # the plate's round end is the flange's Ø (SolidWorks drew it 0.0007 over), the bolts the flange's, on its axes
    assert 2.0 * P.pitch_r == pytest.approx(COUPLER.flange_dia, abs=0.002)
    assert (S.bolt_r, S.bolt_dia, S.nut_af) == (COUPLER.flange_bolt_r, COUPLER.flange_bolt_dia, COUPLER.nut_af)
    for x, y, a in seat_bolt_points(LEGACY):
        assert a % 90.0 == 0.0 and math.hypot(x - P.pitch_x, y) == pytest.approx(S.bolt_r)   # on the axes
        assert (x - P.pitch_x, y) == pytest.approx((S.bolt_r * math.cos(math.radians(a)), S.bolt_r * math.sin(math.radians(a))))


def test_the_end_face_is_centred_on_the_roll_axis():
    m3, m4 = end_face_holes(LEGACY)
    assert E.m3_sp == NEMA17_BOLT_SP
    assert sum(z for _, z in m3) / 4.0 == sum(z for _, z in m4) / 4.0 == E.axis_z == sum(T.block_z) / 2.0
    assert sum(y for y, _ in m3) == sum(y for y, _ in m4) == 0.0
    # the slope runs from the plate's top to the tower's top, where it touches the bore
    assert slope_x(P.thickness, LEGACY) == T.slope_x0 and slope_x(T.z1, LEGACY) == pytest.approx(T.bore_r)


def _fillet_probe(phi_deg: float, depth_frac: float) -> tuple[float, float, float]:
    """A point on the bisector of the slope / bore corner at phi_deg round the bore, depth_frac of the way from the
    corner to the R fillet's surface (< 1: inside the fillet, > 1: in the bore)."""
    phi, r = math.radians(phi_deg), T.fillet_r
    x, y = T.bore_r * math.cos(phi), T.bore_r * math.sin(phi)
    z = P.thickness + (x - T.slope_x0) * (T.z1 - P.thickness) / (T.slope_x1 - T.slope_x0)
    ns = (-(T.z1 - P.thickness), 0.0, T.slope_x1 - T.slope_x0)
    ns = tuple(c / math.hypot(*ns) for c in ns)                    # the slope's normal, into the bore
    nw = (-math.cos(phi), -math.sin(phi), 0.0)                     # the bore wall's, into the bore
    b = tuple(p + q for p, q in zip(ns, nw, strict=True))
    b = tuple(c / math.hypot(*b) for c in b)
    half = (math.pi - math.acos(sum(p * q for p, q in zip(ns, nw, strict=True)))) / 2.0
    d = depth_frac * (r / math.sin(half) - r)
    return (x + d * b[0], y + d * b[1], z + d * b[2])


@pytest.fixture(scope="module")
def legacy():
    from lib.wrist.link import build_wrist_link
    return build_wrist_link(LEGACY)


@pytest.fixture(scope="module")
def wrist():
    return built.part("wrist_link")


@pytest.mark.slow
def test_legacy_plate_and_seat(legacy):
    leg = legacy
    assert leg.is_valid and len(leg.solids()) == 1
    assert is_inside(leg, -20, 38.5, 2.5) and not is_inside(leg, -20, 39.5, 2.5)            # the flats
    assert is_inside(leg, -20, 5, 4.9) and not is_inside(leg, -20, 5, 5.1)                  # the plate, 5 thick
    assert is_inside(leg, P.pitch_x - P.pitch_r + 0.1, 0, 2.5) and not is_inside(leg, P.pitch_x - P.pitch_r - 0.1, 0, 2.5)
    r = S.hole_dia / 2.0
    assert not is_inside(leg, P.pitch_x + r - 0.1, 0, 2.5) and is_inside(leg, P.pitch_x + r + 0.1, 0, 2.5)   # the hole
    for x, y, a in seat_bolt_points(LEGACY):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        assert not is_inside(leg, x, y, 3.5) and not is_inside(leg, x, y, 1.0)               # the bolt, its nut pocket:
        assert not is_inside(leg, x - 3.3 * c, y - 3.3 * s, 1.0) and is_inside(leg, x - 3.6 * c, y - 3.6 * s, 1.0)   # a flat
        assert not is_inside(leg, x - 3.8 * s, y + 3.8 * c, 1.0) and is_inside(leg, x - 4.1 * s, y + 4.1 * c, 1.0)   # a corner
        assert is_inside(leg, x - 3.3 * c, y - 3.3 * s, 2.5)                                  # ... 2 deep
    # the end face's z 6.5 holes, drilled through the whole part, score the plate's top
    assert not is_inside(leg, -20, E.m4_y, 4.5) and is_inside(leg, -20, E.m4_y, 4.3)
    assert not is_inside(leg, -20, E.m3_sp / 2.0, 4.95) and is_inside(leg, -20, E.m3_sp / 2.0, 4.85)


@pytest.mark.slow
def test_legacy_tower(legacy):
    leg = legacy
    assert not is_inside(leg, T.x0 - 0.4, 35, 20) and is_inside(leg, T.x0 + 0.4, 35, 20)   # the wall's back face
    assert is_inside(leg, 20, 30, T.z1 - 0.1) and not is_inside(leg, 20, 30, T.z1 + 0.1)    # its top
    assert not is_inside(leg, 10, 0, 30) and is_inside(leg, 22, 0, 30)                      # the bore, the wedge
    for s in (-1.0, 1.0):
        assert not is_inside(leg, *_fillet_probe(s * 40.0, 1.5)) and is_inside(leg, *_fillet_probe(s * 40.0, 0.5))
        assert is_inside(leg, 10, s * 25, T.cheek_z1 - 0.1) and not is_inside(leg, 10, s * 25, T.cheek_z1 + 0.1)
        assert not is_inside(leg, 10, s * (T.slot_half - 0.2), 30) and is_inside(leg, 10, s * (T.slot_half + 0.2), 30)
        assert not is_inside(leg, T.cheek_x - 0.2, s * 25, 30) and is_inside(leg, T.cheek_x + 0.2, s * 25, 30)
        assert is_inside(leg, 39, s * (T.block_half_width - 0.2), 2) and not is_inside(leg, 39, s * (T.block_half_width + 0.2), 2)
    assert is_inside(leg, 39.5, 0, 1.5) and not is_inside(leg, 39.5, 0, 0.5)                # the block's underside
    # the back opening: out to back_y on +Y, only the bore on -Y
    assert not is_inside(leg, 3, 29.93, 30) and is_inside(leg, 3, -29.93, 30)
    m3, m4 = end_face_holes(LEGACY)
    for y, z in m3:
        assert not is_inside(leg, 35, y, z) and is_inside(leg, 35, y, z + E.m3_dia / 2.0 + 0.1)
    for y, z in m4:
        assert not is_inside(leg, 35, y, z) and is_inside(leg, 35, y, z + E.m4_dia / 2.0 + 0.1)


def test_default_is_the_roll_frame():
    """DEFAULT differs from LEGACY in exactly the wrist roll drive (its frame on the plate) and the connector's hub; the
    drive's frame (module) sits on the pitch axis' station, centre_distance across, axis_h over the coupler face."""
    assert LEGACY.roll is None and DEFAULT.roll is not None and DEFAULT.connector.hub and not LEGACY.connector.hub
    assert DEFAULT == replace(LEGACY, roll=DEFAULT.roll, connector=replace(LEGACY.connector, hub=True))
    d = DEFAULT.roll
    assert module_frame_in_host(DEFAULT) == ((P.pitch_x, d.centre_distance, P.thickness - d.axis_h), (0.0, 90.0, 0.0))


def _part(mx: float, my: float, mz: float) -> tuple[float, float, float]:
    """A point of the drive's module frame in wrist_link's part frame (module +Z = part +X, +X = part -Z, +Y = part +Y)."""
    d = DEFAULT.roll
    return (P.pitch_x + mz, d.centre_distance + my, (P.thickness - d.axis_h) - mx)


@pytest.mark.slow
def test_the_part_is_the_roll_frame(wrist, legacy):
    """The built DEFAULT: the plate's round end with the coupler's seat, the frame on its underside - the pancake's
    pocket and the plate in front of it, the tower's seats and lip, the bay open on the far face with the stop post, the
    coupler nut the tower covers reached from the bay."""
    d, ST = DEFAULT.roll, stack_positions(DEFAULT)
    assert wrist.is_valid and len(wrist.solids()) == 1
    assert wrist.volume < legacy.volume
    # the old tower and its end face are gone; the plate's round end on the pitch axis stays
    assert is_inside(legacy, 20.0, 0.0, 22.0) and not is_inside(wrist, 20.0, 0.0, 22.0)
    c = math.sqrt(0.5)   # between the coupler's bolts (on the axes)
    assert is_inside(wrist, P.pitch_x + 38.5 * c, -38.5 * c, 2.5) and not is_inside(wrist, P.pitch_x + 39.5 * c, -39.5 * c, 2.5)
    # the pancake's pocket on the pitch axis, its floor the plate's free face; the plate in front of it, the shaft's slit
    ym = ST["y_motor"]
    assert not is_inside(wrist, *_part(0.0, ym, 0.0))
    assert not is_inside(wrist, *_part(ST["x_free"] + 0.1, ym + 18.0, 0.0)) and is_inside(wrist, *_part(ST["x_free"] - 0.1, ym + 18.0, 0.0))
    assert is_inside(wrist, *_part(0.0, ym + 18.0, (ST["z_motor_face"] + ST["z_front"]) / 2.0))
    assert not is_inside(wrist, *_part(10.0, ym, (ST["z_motor_face"] + ST["z_front"]) / 2.0))
    # the tower round R': bearing 1's seat, the lip, the boss's top
    r_seat = (d.bearing_od + d.seat_add) / 2.0
    z1 = (ST["z_bearing_1"] + ST["z_lip"]) / 2.0
    assert not is_inside(wrist, *_part(0.0, r_seat - 0.1, z1)) and is_inside(wrist, *_part(0.0, r_seat + 0.1, z1))
    assert not is_inside(wrist, *_part(0.0, d.lip_id / 2.0 - 0.2, ST["z_meet"])) and is_inside(wrist, *_part(0.0, d.lip_id / 2.0 + 0.2, ST["z_meet"]))
    assert is_inside(wrist, *_part(0.0, 24.0, ST["z_boss"] - 0.2)) and not is_inside(wrist, *_part(0.0, 24.0, ST["z_boss"] + 0.2))
    # the bay behind bearing 1, open on +X; the post at -X inside it, the lug's room at +X
    zb = (ST["z_stop_post"] + ST["z_bearing_1"]) / 2.0
    assert not is_inside(wrist, *_part(d.frame_r - 1.0, 0.0, (ST["z_bay"] + ST["z_bearing_1"]) / 2.0))
    assert is_inside(wrist, *_part(-21.0, 0.0, zb)) and not is_inside(wrist, *_part(21.0, 0.0, zb))
    assert is_inside(wrist, *_part(-(d.bay_r + 0.3), 0.0, ST["z_bay"] + 1.0))
    # the coupler nut under the tower: its hex channel from the plate's pocket up into the bay
    covered = [(x, y) for x, y, _ in seat_bolt_points(DEFAULT) if y > 1.0]
    assert len(covered) == 1
    x, y = covered[0]
    assert not is_inside(wrist, x, y, 1.0) and not is_inside(wrist, x, y, -2.0)
