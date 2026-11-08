"""wrist_link, parametric (lib/wrist/): the LEGACY configuration reproduces the SolidWorks part feature by feature (the
volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression names
what moved), its seat is j3_coupler's flange (lib/coupler/params.py) and its end face the NEMA 17 pattern, and DEFAULT
- what the part builds - is the wrist roll drive's frame (the drive itself: tests/wrist/test_roll_drive.py), the plate
only its wall on the forearm's side, with the wrist_pitch output flange on it (j3_coupler#2 retired): its post cut flush
with the frame, nothing past the frame's outline."""
import math
from dataclasses import replace

import pytest
from build123d import Align, Box, Plane, Pos, extrude

from lib.coupler import LEGACY as COUPLER
from lib.geom import hex_prism
from lib.motors import NEMA17_BOLT_SP
from lib.wrist import DEFAULT, LEGACY, FlangeParams, end_face_holes, pulley_bolt_points, seat_bolt_points, slope_x
from lib.wrist.layout import module_frame_in_host, side_wall_x, stack_positions, toward_motor
from lib.wrist.params import PITCH_INSET
from lib.wrist.roll import FAR, build_frame, module_to_host, side_profile
from tests import built
from tests.helpers import is_inside

P, S, T, E = LEGACY.plate, LEGACY.seat, LEGACY.tower, LEGACY.end_face
NUT_WALL = 3.0   # [DESIGN] the least wall round each 90T nut's channel (3.9 measured, to the post's front flat)
SHOULDER_WALL = 1.3   # [DESIGN] ... and in the Ø33 shoulder the nut's top 0.7 stands in: the hex grown 1.3 reaches 1.5 out at its
#                       corners, the corner's radial wall 1.55 (docs/open_issues.md)


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
    """DEFAULT differs from LEGACY in exactly the wrist roll drive (its frame on the plate), the wrist_pitch output flange
    in place of j3_coupler's seat, the connector's hub and the part's inset toward the forearm (PITCH_INSET); the
    drive's frame (module) sits on the pitch axis' station, centre_distance across on its side (-Y), axis_h over the
    coupler face."""
    assert LEGACY.roll is None and DEFAULT.roll is not None and DEFAULT.connector.hub and not LEGACY.connector.hub
    assert DEFAULT == replace(LEGACY, seat=None, flange=FlangeParams(), roll=DEFAULT.roll, connector=replace(LEGACY.connector, hub=True),
                              pitch_inset=PITCH_INSET)
    assert (LEGACY.pitch_inset, PITCH_INSET) == (0.0, 14.0)
    d = DEFAULT.roll
    assert module_frame_in_host(DEFAULT) == ((P.pitch_x, -d.centre_distance, P.thickness - d.axis_h), (0.0, 90.0, 0.0))


def _part(mx: float, my: float, mz: float) -> tuple[float, float, float]:
    """A point of the drive's module frame in wrist_link's part frame (module +Z = part +X, +X = part -Z, +Y = part +Y)."""
    d = DEFAULT.roll
    return (P.pitch_x + mz, d.side * d.centre_distance + my, (P.thickness - d.axis_h) - mx)


@pytest.mark.slow
def test_the_part_is_the_roll_frame(wrist, legacy):
    """The built DEFAULT: the roll frame, its wall on the forearm's side the plate (no plate past the frame) with the
    wrist_pitch output flange - the pancake's pocket (open down through the bottom, the far face's wall closed over it,
    its plug's groove inside the wall) and the motor plate in front of it, the front face's facets, the tower's seats and
    lip, the bay bored through the rear with the stop post; the flange's post (flush with the frame's front face and rear),
    shoulder and stub, the 90T's nuts in channels open to the pancake's pocket."""
    d, ST = DEFAULT.roll, stack_positions(DEFAULT)
    assert wrist.is_valid and len(wrist.solids()) == 1
    assert wrist.volume < legacy.volume
    # the old tower and its end face are gone; so is the plate's octagon past the frame, in front of it and behind it -
    # the frame's octagonal end about the pitch axis stays (the side away from R')
    assert is_inside(legacy, 20.0, 0.0, 22.0) and not is_inside(wrist, 20.0, 0.0, 22.0)
    for x in (ST["z_front"] + 2.0, 30.0, d.rear_low - 2.0, -30.0):
        assert not is_inside(wrist, P.pitch_x + x, 25.0, 2.5), x
    assert is_inside(wrist, P.pitch_x, P.pitch_r - 0.5, 2.5) and not is_inside(wrist, P.pitch_x, P.pitch_r + 0.5, 2.5)
    # the pancake's pocket on the pitch axis, its floor the plate's free face; the plate in front of it, the shaft's slit
    ym = ST["y_motor"]
    assert not is_inside(wrist, *_part(0.0, ym, 0.0))
    assert not is_inside(wrist, *_part(ST["x_free"] + 0.1, ym + 18.0, 0.0)) and is_inside(wrist, *_part(ST["x_free"] - 0.1, ym + 18.0, 0.0))
    assert is_inside(wrist, *_part(10.0, ym + 18.0, (ST["z_motor_face"] + ST["z_front"]) / 2.0))   # beside the slit
    assert not is_inside(wrist, *_part(10.0, ym, (ST["z_motor_face"] + ST["z_front"]) / 2.0))
    # the tower round R': bearing 1's seat, the lip, the boss's top
    r_seat = (d.bearing_od + d.seat_add) / 2.0
    z1 = (ST["z_bearing_1"] + ST["z_lip"]) / 2.0
    assert not is_inside(wrist, *_part(0.0, r_seat - 0.1, z1)) and is_inside(wrist, *_part(0.0, r_seat + 0.1, z1))
    assert not is_inside(wrist, *_part(0.0, d.lip_id / 2.0 - 0.2, ST["z_meet"])) and is_inside(wrist, *_part(0.0, d.lip_id / 2.0 + 0.2, ST["z_meet"]))
    assert is_inside(wrist, *_part(0.0, 24.0, ST["z_boss"] - 0.2)) and not is_inside(wrist, *_part(0.0, 24.0, ST["z_boss"] + 0.2))
    # the pocket on down out through the bottom (the pancake slides in from there), the far face's wall closed over it,
    # the plug's groove inside the wall (a skin outside it), the shaft's slit out through the bottom too
    s, xw = toward_motor(DEFAULT), side_wall_x(DEFAULT)
    assert not is_inside(wrist, *_part(0.0, ym + s * 30.0, 0.0)) and not is_inside(wrist, *_part(0.0, ym + s * 30.0, (ST["z_motor_face"] + ST["z_front"]) / 2.0))
    assert is_inside(wrist, *_part(xw + 1.0, ym, 0.0)) and is_inside(wrist, *_part(d.frame_r - 1.0, ym, 0.0))
    assert not is_inside(wrist, *_part(xw + 2.0, ym, -7.0)) and is_inside(wrist, *_part(d.frame_r - 1.0, ym, -7.0))
    # the front face's two long edges faceted front_facet at 45 deg: material just inside the far face's facet, none past it
    mid = (d.frame_r - d.front_facet / 2.0, ST["z_front"] - d.front_facet / 2.0)
    assert is_inside(wrist, *_part(mid[0] - 0.35, ym, mid[1] - 0.35)) and not is_inside(wrist, *_part(mid[0] + 0.35, ym, mid[1] + 0.35))
    # the bay behind bearing 1, bored on through the rear (under the back panel), the far face's wall closed beside it; the
    # post at -X inside it, the lug's room at +X
    zb = (ST["z_stop_post"] + ST["z_bearing_1"]) / 2.0
    assert is_inside(wrist, *_part(d.frame_r - 1.0, 0.0, (ST["z_bay"] + ST["z_bearing_1"]) / 2.0))
    assert not is_inside(wrist, *_part(0.0, 0.0, ST["z_bay"])) and not is_inside(wrist, *_part(0.0, 0.0, ST["z_rear"] - 1.0))
    assert is_inside(wrist, *_part(-21.0, 0.0, zb)) and not is_inside(wrist, *_part(21.0, 0.0, zb))
    assert is_inside(wrist, *_part(-(d.bay_r + 0.3), 0.0, ST["z_bay"] + 1.0))
    # the wrist_pitch output flange on the plate's coupler face (FlangeParams, j3_coupler#2 retired): the post inside its
    # radius only, cut flat at the frame's front face and its rear, its front flat carried on down past the free face into
    # the front facet's foot (beside the post the facet stays), its foot chamfered, the shoulder, the stub; the pin bore
    # blind from the stub's end; each 90T nut's hex channel - the pattern turned pulley_bolt_deg - from its seat down
    # through the plate and open into the pancake's pocket below it (the nut drops in from there), the M4's clearance on
    # up from the seat
    f, px = DEFAULT.flange, P.pitch_x
    assert DEFAULT.seat is None
    zp, zs, zh = sum(f.post_z) / 2.0, sum(f.stub_z) / 2.0, sum(f.step_z) / 2.0
    assert is_inside(wrist, px, f.post_dia / 2.0 - 1.0, zp) and not is_inside(wrist, px, f.post_dia / 2.0 + 1.0, zp)
    assert not is_inside(wrist, px, f.post_dia / 2.0 - f.post_chamfer / 2.0 + 0.2, f.post_z[1] - f.post_chamfer / 4.0)   # the chamfer
    assert f.post_z[1] - f.post_z[0] == pytest.approx(1.3) and f.post_chamfer == 1.0   # the 2 mm gap's post (PITCH_INSET)
    for x0, s in ((ST["z_front"], 1.0), (d.rear_low, -1.0)):
        assert is_inside(wrist, px + x0 - 0.3 * s, 0.0, zp) and not is_inside(wrist, px + x0 + 0.3 * s, 0.0, zp), x0
    assert is_inside(wrist, px + ST["z_front"] - 0.3, 0.0, -1.0) and not is_inside(wrist, px + ST["z_front"] - 0.3, 18.0, -1.0)
    assert is_inside(wrist, px, f.step_dia / 2.0 - 0.3, zh) and not is_inside(wrist, px, f.step_dia / 2.0 + 0.3, zh)
    assert is_inside(wrist, px, f.stub_dia / 2.0 - 1.0, zs) and not is_inside(wrist, px, f.stub_dia / 2.0 + 0.5, zs)
    assert not is_inside(wrist, px, 0.0, zs) and is_inside(wrist, px, 0.0, f.nut_seat_z + f.pin_bore_past - 0.5)
    for x, y in pulley_bolt_points(DEFAULT):
        flat = math.radians(f.pulley_bolt_deg + f.nut_corner_deg + 30.0)  # toward a flat: 3.0 is inside the hex, outside the M4
        fx, fy = x + 3.0 * math.cos(flat), y + 3.0 * math.sin(flat)
        assert not is_inside(wrist, fx, fy, f.nut_seat_z - 0.5) and is_inside(wrist, fx, fy, f.nut_seat_z + 0.5)   # the seat
        assert not is_inside(wrist, x, y, f.nut_seat_z + 2.0) and not is_inside(wrist, x, y, 2.5)
        assert not is_inside(wrist, x, y, -f.channel_past - 1.0)


@pytest.mark.slow
def test_the_flange_stands_inside_the_frame(wrist):
    """No plate past the roll frame: below j2_link (the post's top) everything the flange adds to the frame lies inside
    the frame's outline seen along the pitch axis (side_profile) - the post's flats on the frame's front face and rear -,
    and every 90T nut's channel keeps at least NUT_WALL of wall round it in the post and the frame (the pattern turned
    pulley_bolt_deg: on the axes the front nut came 0.55 off the post's front flat) - its seat level with the bearing's
    top, as the elbow's, so the nut's top 0.7 stands in the Ø33 shoulder, SHOULDER_WALL round it there."""
    f, px = DEFAULT.flange, P.pitch_x
    host = module_to_host(DEFAULT)
    frame = build_frame(DEFAULT).moved(host)
    side = Plane(origin=(-FAR, 0.0, 0.0), x_dir=(0.0, 1.0, 0.0), z_dir=(1.0, 0.0, 0.0))   # module (y, z) -> along module X
    outline = extrude(side * side_profile(DEFAULT), amount=2.0 * FAR).moved(host)
    below = Pos(0.0, 0.0, -FAR) * Box(4 * FAR, 4 * FAR, FAR + f.post_z[1], align=(Align.CENTER, Align.CENTER, Align.MIN))
    assert ((wrist - frame) & below).volume > 1000.0                  # the post, carried down past the coupler face
    assert (((wrist - frame) & below) - outline).volume < 1e-3
    z0, z1 = 1.0, f.post_z[1]                                         # over the pancake's pocket, up to the shoulder
    assert f.step_z == (f.post_z[1], f.nut_seat_z)
    corner = math.radians(f.pulley_bolt_deg + f.nut_corner_deg)
    for x, y in pulley_bolt_points(DEFAULT):
        for (za, zb), wall in (((z0, z1), NUT_WALL), ((z1, f.nut_seat_z - 0.05), SHOULDER_WALL)):
            nut = Pos(x, y, za) * hex_prism(f.nut_af, corner, zb - za)
            ring = Pos(x, y, za) * hex_prism(f.nut_af + 2.0 * wall, corner, zb - za) - nut
            assert (ring - wrist).volume < 1e-3, (round(x - px, 2), round(y, 2), wall)
