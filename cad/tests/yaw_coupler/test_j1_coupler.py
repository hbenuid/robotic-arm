"""j1_coupler, parametric (lib/yaw_coupler/): the LEGACY configuration reproduces the SolidWorks part feature by
feature (the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a
regression names what moved), the yoke's shapes are the drive's housing's - its bore, its od, its pillars' sides,
its bolt circle and nut pockets (lib/cycloidal/params.py HousingParams) -, and DEFAULT - what the part builds - changes
its underside, its hub and the yoke's hold on the housing: the seat on the thrust bearing, the rim clear of the base
(tests/test_mounts.py checks the stack in place), the stub on to the lip's lower face and drilled for the base_yaw 90T,
the sockets round the 6-pillar housing's two pillars at +/-30 degrees (tests/cycloidal/test_assembly.py checks the
drive in place)."""
import math
from dataclasses import replace

import pytest

from lib.base import DEFAULT as BASE
from lib.bearings import BEARING_6806_BORE, THRUST_OD, THRUST_STACK
from lib.belts import GT2_PULLEY_90T_BOLT_R
from lib.coupler import DEFAULT as J3_COUPLER
from lib.cycloidal import compute_housing_bolt_angles
from lib.cycloidal.layout import PILLAR_OVERSHOOT
from lib.cycloidal.params import DEFAULT_CONFIG as DRIVE
from lib.fasteners import M4_CLEAR, M4_NUT
from lib.geom import hex_circumdiameter
from lib.yaw_coupler import DEFAULT, LEGACY, hole_points, nut_centres, od_point, socket_outline
from lib.yaw_coupler.layout import below_axis, pillar_point
from tests import built
from tests.helpers import is_inside


def _polar(y: float, z: float, deg: float) -> tuple[float, float]:
    """(along, across) of a (y, z) point from the drive's axis, against the pillar at deg from straight down."""
    k, a = LEGACY.yoke, math.radians(deg)
    u = k.axis_y - y
    return (u * math.cos(a) + z * math.sin(a), abs(-u * math.sin(a) + z * math.cos(a)))


def _pillar_half_w(along: float) -> float:
    """The housing pillar's half width at `along` from the drive's axis (lib/cycloidal/housing.py's trapezoid)."""
    h = DRIVE.housing
    r0, r1 = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + PILLAR_OVERSHOOT
    return (h.pillar_inner_w + (h.pillar_outer_w - h.pillar_inner_w) * (along - r0) / (r1 - r0)) / 2.0


def test_the_yoke_is_the_housings_shape():
    k, h = LEGACY.yoke, DRIVE.housing
    assert (2.0 * k.cradle_r, 2.0 * k.od_r, 2.0 * k.bolt_circle_r) == (h.bore_dia, h.od, h.bolt_circle_dia)
    assert k.nut_af == pytest.approx(h.bolt_nut_pocket_af, abs=0.01) and k.nut_depth == h.bolt_nut_depth
    near, end = k.groove_z
    c = (below_axis(LEGACY, k.cradle_r, near), near)
    d = (below_axis(LEGACY, k.od_r, end), end)
    a = (below_axis(LEGACY, k.cradle_r, k.cheek_top_z), k.cheek_top_z)
    b = od_point(LEGACY)
    # the V-groove's near side (C -> D) is the 45 degree pillar's side, no clearance; the cheek's top (B -> A) its
    # far side; D and B, C and A mirror each other about the pillar's centre line (to the census's 1e-4)
    for p in (c, d, a, b):
        along, across = _polar(*p, 45.0)
        assert across == pytest.approx(_pillar_half_w(along), abs=0.01)
    assert (k.axis_y - b[0], b[1]) == pytest.approx((d[1], k.axis_y - d[0]), abs=1e-4)
    assert (k.axis_y - a[0], a[1]) == pytest.approx((c[1], k.axis_y - c[0]), abs=1e-4)
    # the channel: its floor corners on the od, its walls the bottom pillar's sides 0.2 out
    corner_along = k.axis_y - k.channel_floor_y
    assert math.hypot(corner_along, _pillar_half_w(corner_along)) == pytest.approx(k.od_r, abs=0.01)
    assert k.channel_slope == pytest.approx(_pillar_half_w(60.0) - _pillar_half_w(61.0))
    gap = (k.channel_half_z - _pillar_half_w(corner_along)) / math.hypot(1.0, k.channel_slope)
    assert gap == pytest.approx(0.2, abs=0.005)
    # the nut pockets: the bottom housing bolt and the two at +/-45 degrees, on the bolt circle
    assert [math.hypot(k.axis_y - y, z) for y, z in nut_centres(LEGACY)] == pytest.approx([k.bolt_circle_r] * 3)


def _from_bottom(drive_rad: float) -> float:
    """A drive angle (radians from the drive's +X) as the yoke's: degrees from straight down, toward +Z
    (placements.json turns the drive's +X onto the coupler's +Z, its +Y onto +Y)."""
    return (math.degrees(drive_rad) - 270.0 + 180.0) % 360.0 - 180.0


def test_default_yoke_holds_the_housings_pillars():
    """DEFAULT's yoke on the drive's housing (lib/cycloidal/params.py DEFAULT_CONFIG): a socket round each pillar it
    holds - on a housing bolt, its walls and floor socket_clear off the pillar -, the nut pockets under those bolts,
    and every other pillar clear of the yoke (past the cheek's top on the cradle and the od point on the od)."""
    k, h = DEFAULT.yoke, DRIVE.housing
    pillars = [_from_bottom(a) for a in compute_housing_bolt_angles(DRIVE)]
    assert k.socket_deg and k.bolt_deg == k.socket_deg and k.groove_z is None and not k.channel
    assert all(any(abs(d - p) < 1e-9 for p in pillars) for d in k.socket_deg)
    reach = max(math.degrees(math.asin(k.cheek_top_z / k.cradle_r)), math.degrees(math.asin(od_point(DEFAULT)[1] / k.od_r)))
    half = math.degrees(math.asin(_pillar_half_w(k.cradle_r) / k.cradle_r))      # the pillar's widest, at the bore
    for p in pillars:
        if not any(abs(d - p) < 1e-9 for d in k.socket_deg):
            assert abs(p) - half > reach + 1.0, f"the pillar at {p:.0f} degrees reaches into the yoke"
    grow = math.hypot(1.0, _pillar_half_w(1.0) - _pillar_half_w(0.0))
    for d in k.socket_deg:
        outline = socket_outline(DEFAULT, d)
        for y, z in outline:
            along, across = _polar(y, z, d)
            assert across == pytest.approx(_pillar_half_w(along) + k.socket_clear * grow, abs=1e-9)   # the walls
        floor = sorted(_polar(y, z, d)[0] for y, z in outline)[2:]
        assert floor == pytest.approx([h.od / 2.0 + k.socket_clear] * 2)                            # the floor
        assert max(math.hypot(*_polar(y, z, d)) for y, z in outline if _polar(y, z, d)[0] < k.cradle_r) < k.cradle_r
        assert pillar_point(DEFAULT, d, k.bolt_circle_r, 0.0) in [pytest.approx(c) for c in nut_centres(DEFAULT)]


@pytest.fixture(scope="module")
def legacy():
    return built.legacy("j1_coupler")


@pytest.fixture(scope="module")
def coupler():
    return built.part("j1_coupler")


@pytest.mark.slow
def test_legacy_disc(legacy):
    leg = legacy
    assert leg.is_valid and len(leg.solids()) == 1
    assert is_inside(leg, 0, -0.2, 52.8) and not is_inside(leg, 0, -0.2, 53.3)       # the band ...
    assert is_inside(leg, 0, 19.8, 50.6) and not is_inside(leg, 0, 19.8, 51.1)       # ... drafted in to the top face
    assert not is_inside(leg, 0, -0.5, 50) and is_inside(leg, 0, -0.3, 50)           # the rim's underside
    assert not is_inside(leg, 0, 0.3, 44.5) and is_inside(leg, 0, 0.7, 44.5)         # the recess, 0.9 deep ...
    assert is_inside(leg, 0, 0.3, 45.5)                                              # ... Ø90.05
    assert is_inside(leg, 39.5, 12, 0) and not is_inside(leg, 40.5, 12, 0)           # the flats
    assert is_inside(leg, 47.5, 5, 0) and not is_inside(leg, 48.5, 5, 0)             # the ears ...
    assert not is_inside(leg, 45, 8.5, 0)                                            # ... up to y 8
    assert is_inside(leg, -32.5, 22, 20) and not is_inside(leg, -33.0, 22, 20)       # the ring, from the cheek's face ...
    assert is_inside(leg, 0, 22, 40.5) and not is_inside(leg, 0, 22, 41.0)           # ... Ø81.46


@pytest.mark.slow
def test_legacy_yoke(legacy):
    leg = legacy
    assert is_inside(leg, 0, 26, 43.5) and not is_inside(leg, 0, 26, 44.5)           # the flare
    assert is_inside(leg, 0, 40, 52.9) and not is_inside(leg, 0, 40, 53.4)           # the outer faces
    assert is_inside(leg, 20, 34.5, 20) and not is_inside(leg, 20, 36.5, 20)         # the cradle
    pillar = (LEGACY.yoke.axis_y - 64 * math.cos(math.radians(45)), 64 * math.sin(math.radians(45)))
    assert not is_inside(leg, 0, *pillar)                                            # the V-groove round the pillar ...
    assert is_inside(leg, 0, 38.0, 33.77)                                            # ... inside its near side
    assert is_inside(leg, -31, 45.8, 49.2)                                           # the cheek, across the pillar
    assert not is_inside(leg, 25, 21, 0) and is_inside(leg, 25, 20.0, 0)             # the channel's floor ...
    assert is_inside(leg, 25, 21, 6.0)                                               # ... its wall
    assert not is_inside(leg, 32.4, 21, 0) and is_inside(leg, 33.0, 21, 0)           # ... its +X end
    assert not is_inside(leg, 35, 23.5, 0) and is_inside(leg, 35, 22.3, 0)           # the notch
    assert not is_inside(leg, 15.0, 10, 30.8) and is_inside(leg, 15.5, 10, 0)        # the pocket ...
    assert is_inside(leg, 0, 10, 31.3)
    assert is_inside(leg, 0, 7.8, 20) and not is_inside(leg, 0, 8.2, 20)             # ... its floor
    for y, z in nut_centres(LEGACY):
        assert not is_inside(leg, -31, y, z) and not is_inside(leg, -28.7, y, z)     # the nut pocket, the hole ...
        assert is_inside(leg, -28.7, y + 2.5, z)                                     # ... 4 deep
    y, z = nut_centres(LEGACY)[1]
    assert is_inside(leg, -31, y + 3.8, z) and not is_inside(leg, -31, y + 3.4, z)   # flats across y ...
    assert not is_inside(leg, -31, y, z + 4.0) and is_inside(leg, -31, y, z + 4.3)   # ... a corner along z


@pytest.mark.slow
def test_legacy_hub(legacy):
    leg = legacy
    assert is_inside(leg, 0, -5, 14.7) and not is_inside(leg, 0, -5, 15.1)           # the stub
    assert is_inside(leg, 0, -8.1, 12) and not is_inside(leg, 0, -8.3, 12)           # its end ...
    assert not is_inside(leg, 0, -8.1, 14.8) and is_inside(leg, 0, -7.3, 14.8)       # ... the R1 round
    assert not is_inside(leg, 0, 2, 7.3) and is_inside(leg, 0, 2, 7.7)               # the bore
    for x, z in hole_points(LEGACY):
        assert not is_inside(leg, x, 2, z) and is_inside(leg, 1.2 * x, 2, 1.2 * z)   # the 4 holes on the diagonals
        assert not is_inside(leg, x, -8.0, z)                                        # ... from the stub's end


def test_default_stands_on_the_thrust_bearing():
    """The seat on the upper washer: the groove's floor (the base's frame; this part's origin at its ring_top_y) plus
    the stack; the recess round the stack's OD; the rim over the base's top face - nothing else changes (but the
    yoke's hold on the housing: test_default_yoke_holds_the_housings_pillars)."""
    h, c = DEFAULT.hub, BASE.cap
    assert replace(DEFAULT, hub=LEGACY.hub, disc=replace(DEFAULT.disc, y0=LEGACY.disc.y0), yoke=LEGACY.yoke) == LEGACY
    assert h.recess_y1 == pytest.approx(c.groove_y0 - c.ring_top_y + THRUST_STACK)
    assert h.recess_dia > THRUST_OD and h.recess_dia / 2.0 > c.groove_r[1]          # the rim wholly over the top face
    assert DEFAULT.disc.y0 > c.top_y - c.ring_top_y and LEGACY.disc.y0 == pytest.approx(c.top_y - c.ring_top_y)


def test_default_hub_takes_the_base_yaw_90t():
    """The stub the bearings' bore, its end on the lip's lower face where the 90T's hub end meets it (as at the wrist;
    the SolidWorks end stopped 0.2 into the lip); the 90T's 4 bolts on its bolt circle at the diagonals (the 90T turned
    45 degrees on its own pattern), M4 clearance, their nuts flush in the pocket's floor clear of its walls and of the
    bore; the 90T's bore, 2.5 of wall to the holes (the SolidWorks Ø15 left 1.45)."""
    h, b, c = DEFAULT.hub, BASE.bore, BASE.cap
    assert h.stub_dia == BEARING_6806_BORE and (h.stub_round, h.hole_deg) == (LEGACY.hub.stub_round, LEGACY.hub.hole_deg)
    assert h.stub_y0 == pytest.approx(b.lip_y[0] - c.ring_top_y) and LEGACY.hub.stub_y0 == pytest.approx(b.lip_y[1] - c.ring_top_y - 0.2)
    assert (h.hole_dia, h.hole_r, h.bore_dia) == (M4_CLEAR, GT2_PULLEY_90T_BOLT_R, J3_COUPLER.bore_dia)
    assert sorted(round(math.degrees(math.atan2(z, x))) % 360 for x, z in hole_points(DEFAULT)) == [45, 135, 225, 315]
    assert (h.nut_af, h.nut_depth) == (J3_COUPLER.nut_af, M4_NUT.h) and LEGACY.hub.nut_af is None
    corner = hex_circumdiameter(h.nut_af) / 2.0
    (px, pz), k = DEFAULT.yoke.pocket_half, DEFAULT.yoke
    for x, z in hole_points(DEFAULT):
        assert abs(x) + corner < px - 3.0 and abs(z) + corner < pz - 3.0                 # clear of the pocket's walls
        assert math.hypot(x, z) - corner > h.bore_dia / 2.0 + 0.5                        # ... and of the bore
    assert k.pocket_y0 - h.nut_depth > h.recess_y1 + 3.0                                 # material under the nuts
    assert h.hole_r - (h.hole_dia + h.bore_dia) / 2.0 == pytest.approx(2.55)


@pytest.mark.slow
def test_default_changes_the_hub_the_underside_and_the_yoke(coupler, legacy):
    assert coupler.label == "j1_coupler" and coupler.is_valid and len(coupler.solids()) == 1
    bb, lb = coupler.bounding_box(), legacy.bounding_box()
    assert (bb.min.X, bb.min.Z, *tuple(bb.max)) == pytest.approx((lb.min.X, lb.min.Z, *tuple(lb.max)), abs=1e-6)
    assert bb.min.Y == pytest.approx(DEFAULT.hub.stub_y0, abs=1e-6)                           # the stub's end, lower
    assert is_inside(coupler, 0, -9.2, 12) and not is_inside(coupler, 0, -9.4, 12) and not is_inside(legacy, 0, -9.2, 12)
    assert is_inside(coupler, 0, -5, 14.95) and not is_inside(coupler, 0, -5, 15.05)          # the stub Ø30 ...
    assert not is_inside(legacy, 0, -5, 14.95)                                                # ... was Ø29.8
    assert is_inside(coupler, 0, 2, 7.0) and not is_inside(coupler, 0, 2, 6.1) and not is_inside(legacy, 0, 2, 7.0)   # the bore
    for x, z in hole_points(DEFAULT):
        assert not is_inside(coupler, x, 2, z) and not is_inside(coupler, x, -9.2, z)         # the 90T's holes, end to end ...
        edge = (x * 13.0 / 11.0, 2, z * 13.0 / 11.0)
        assert not is_inside(coupler, *edge) and is_inside(legacy, *edge)                     # ... M4 clearance
        assert not is_inside(coupler, x + 3.0, 6.0, z) and is_inside(legacy, x + 3.0, 6.0, z)   # the nut pocket: flats across x ...
        assert is_inside(coupler, x + 3.6, 6.0, z) and not is_inside(coupler, x, 6.0, z + 3.8)  # ... a corner along z ...
        assert is_inside(coupler, x + 3.0, 4.6, z)                                            # ... 3.2 deep
    y1 = DEFAULT.hub.recess_y1
    assert not is_inside(coupler, 0, y1 - 0.05, 44) and is_inside(coupler, 0, y1 + 0.05, 44)   # the seat, 1.1 higher
    assert is_inside(legacy, 0, y1 - 0.05, 44)
    assert not is_inside(coupler, 0, 1.0, 45.1) and is_inside(coupler, 0, 1.0, 45.3)          # the recess round the stack
    y0 = DEFAULT.disc.y0
    assert not is_inside(coupler, 0, y0 - 0.05, 50) and is_inside(coupler, 0, y0 + 0.05, 50)   # the rim, lifted
    assert is_inside(legacy, 0, y0 - 0.05, 50)
    assert not is_inside(coupler, 47.5, y0 - 0.05, 0) and is_inside(coupler, 47.5, y0 + 0.05, 0)   # the ears with it
    assert is_inside(coupler, 0, y1 - 0.05, 14.5) and is_inside(coupler, 0, -5, 14.7)          # the stub up into the seat
    for probe in ((0, 40, 52.9), (20, 34.5, 20), (-31, 45.8, 49.2), (25, 20.0, 0), (0, 2, 7.7)):
        assert is_inside(coupler, *probe) == is_inside(legacy, *probe)                         # the rest as before
    k = DEFAULT.yoke
    flare = DEFAULT.disc.ring_r / (DEFAULT.disc.ring_y1 - k.flare_apex_y)     # the flare's dr / dy
    for d in k.socket_deg:
        hw = _pillar_half_w(64.0)
        for x in (-28.3, 0.0, 22.0, 31.3):
            assert not is_inside(coupler, x, *pillar_point(DEFAULT, d, 64.0, 0.0))            # the socket, end to end ...
        assert is_inside(legacy, 22.0, *pillar_point(DEFAULT, d, 64.0, 0.0))
        assert not is_inside(coupler, 22.0, *pillar_point(DEFAULT, d, 64.0, hw + 0.1))        # ... its walls 0.2 off ...
        assert is_inside(coupler, 22.0, *pillar_point(DEFAULT, d, 64.0, hw + 0.35))
        assert not is_inside(coupler, 22.0, *pillar_point(DEFAULT, d, 70.1, 0.0))             # ... its floor 0.2 off the od
        assert is_inside(coupler, 22.0, *pillar_point(DEFAULT, d, 70.35, 0.0))
        assert is_inside(coupler, -29.0, *pillar_point(DEFAULT, d, 67.0, 0.0))                # ... stopped by the cheek
        # its wall under the floor's corner nearest the ring, socket_wall thick even at the ends, where the flare
        # (the cone out of the ring's top edge) would leave under 1 mm: a rib out of it
        y, z = min(socket_outline(DEFAULT, d), key=lambda p: p[0])
        n = (-flare, math.copysign(1.0, z))
        n = (n[0] / math.hypot(*n), n[1] / math.hypot(*n))
        for x in (-28.3, 31.3):
            assert is_inside(coupler, x, y + n[0] * (k.socket_wall - 0.1), z + n[1] * (k.socket_wall - 0.1))
            assert not is_inside(coupler, x, y + n[0] * (k.socket_wall + 0.3), z + n[1] * (k.socket_wall + 0.3))
    for y, z in nut_centres(DEFAULT):
        assert not is_inside(coupler, -31, y, z) and not is_inside(coupler, -28.7, y, z)      # the nut pockets at +/-30 ...
        assert is_inside(legacy, -31, y, z)
    for y, z in nut_centres(LEGACY):
        assert is_inside(coupler, -31, y, z)                                                  # ... not at 0 and +/-45
    pillar = (k.axis_y - 64 * math.cos(math.radians(45)), 64 * math.sin(math.radians(45)))
    assert is_inside(coupler, 0, *pillar) and not is_inside(legacy, 0, *pillar)              # the V-groove filled
    assert is_inside(coupler, 25, 21, 0) and not is_inside(legacy, 25, 21, 0)                # the channel filled
    assert is_inside(coupler, 35, 23.5, 0) and not is_inside(legacy, 35, 23.5, 0)            # the notch filled
