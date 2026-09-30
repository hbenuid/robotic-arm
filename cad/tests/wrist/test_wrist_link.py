"""wrist_link, parametric (lib/wrist/): the LEGACY configuration reproduces the SolidWorks part feature by feature (the
volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression names
what moved), its seat is j3_coupler's flange (lib/coupler/params.py) and its end face the NEMA 17 pattern, and DEFAULT
- what the part builds - is the tower SHORTENING shorter: the end face nearer the pitch axis, the weight-saving pocket
kept inside the shorter block, the plate's round end clipped at the face, the upper M3 pair gone."""
import math
from dataclasses import replace

import pytest

from lib.coupler import LEGACY as COUPLER
from lib.motors import NEMA17_BOLT_SP
from lib.wrist import DEFAULT, LEGACY, end_face_holes, seat_bolt_points, slope_x
from lib.wrist.params import SHORTENING
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


def test_default_is_the_short_tower():
    """DEFAULT differs from LEGACY in exactly the end face's place (SHORTENING), the slope and the M3 rows."""
    t, e = DEFAULT.tower, DEFAULT.end_face
    assert SHORTENING < 0.0 and t.block_x1 == T.block_x1 + SHORTENING
    assert DEFAULT == replace(LEGACY, tower=replace(T, block_x1=t.block_x1, slope_x1=t.slope_x1),
                              end_face=replace(E, m3_rows=e.m3_rows))
    # the slope's cut stays inside the fillet's core (lib/wrist/link.py: 1 past the end face, _CORE_ABOVE over the top)
    from lib.wrist.link import _CORE_ABOVE

    assert slope_x(T.z1 + _CORE_ABOVE, DEFAULT) < t.block_x1 + 1.0
    m3, m4 = end_face_holes(DEFAULT)
    assert len(m3) == 2 and all(z < e.axis_z for _, z in m3) and m4 == end_face_holes(LEGACY)[1]


@pytest.mark.slow
def test_the_part_is_the_short_tower(wrist, legacy):
    t, e = DEFAULT.tower, DEFAULT.end_face
    assert wrist.is_valid and len(wrist.solids()) == 1
    assert wrist.bounding_box().max.X == pytest.approx(t.block_x1, abs=1e-6)
    assert wrist.volume < legacy.volume
    # nothing past the end face: the wall and the plate's round end are clipped there (both reached x 39)
    assert not is_inside(wrist, t.block_x1 + 0.5, 0, 22.0) and not is_inside(wrist, t.block_x1 + 2.0, 0, 2.5)
    assert is_inside(legacy, t.block_x1 + 2.0, 0, 2.5)
    # the block: solid in front of the pocket up to its top; the lower M3 and the 4 M4 through, the upper M3 not drilled
    assert is_inside(wrist, t.block_x1 - 1.0, 0, 40.0) and is_inside(wrist, t.block_x1 - 5.0, 0, 40.0)
    m3, m4 = end_face_holes(DEFAULT)
    for y, z in m3 + m4:
        assert not is_inside(wrist, t.block_x1 - 1.0, y, z)
    for sy in (-1.0, 1.0):
        assert is_inside(wrist, t.block_x1 - 1.0, sy * e.m3_sp / 2.0, e.axis_z + e.m3_sp / 2.0)
    # the pocket: still open behind the slope
    assert not is_inside(wrist, 8.0, 0.0, 35.0)
