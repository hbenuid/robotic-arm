"""wrist_flange, the wrist_pitch output flange - its own part on wrist_link's coupler face (lib/wrist/link.py
build_wrist_flange): a part with no reference file (lib/reference.py NO_REFERENCE), so THIS file locks its numbers - the
volume and the bounding box, measured on the build and written in - and what it is: the post (cut flat at wrist_link's
front face and rear, its foot toward the bearing chamfered), the shoulder and the stub about the pitch axis, the pin
bore, the wrist 90T's 4x M4 through it, the recess in its foot over wrist_link's key; how it sits on wrist_link (on the
coupler face, the key in its recess, no overlap); and the print rule it exists for: wrist_link keeps nothing round about
the pitch axis but the M4 clearance holes (each printed part prints its circles about one axis)."""
import math

import pytest
from build123d import GeomType

from lib import reference as R
from lib.params import M4_CLEAR
from lib.wrist import DEFAULT, pulley_bolt_points
from lib.wrist.layout import stack_positions
from tests import built
from tests.helpers import interference, is_inside

F, P, D = DEFAULT.flange, DEFAULT.plate, DEFAULT.roll
PX = P.pitch_x
VOLUME = 6586.428                                    # mm^3, the build's
BBOX_MIN = (-63.945774, -20.0, 5.0)                  # wrist_link's part frame (the mount places it on wrist_link as it stands)
BBOX_SIZE = (35.0, 40.0, 11.0)


def test_params():
    """The flange spans the coupler face to the stub's end; the key inside the bolt circle and under the pin bore; the
    nuts 1 of the plate under the face; the 90T's screws M4 x 33, bought as M4 x 35."""
    assert R.NO_REFERENCE["wrist_flange"] == "lib/wrist/link.py:build_wrist_flange(DEFAULT)"
    assert (F.post_z[0], F.stub_z[1]) == (P.thickness, BBOX_MIN[2] + BBOX_SIZE[2])
    assert F.key_af / 2.0 + F.key_fit + 1.0 <= F.pulley_bolt_r - F.pulley_bolt_dia / 2.0   # 1 of wall to the M4 holes
    assert F.post_z[0] + F.key_h < F.pin_bore_z1 and F.pin_bore_z1 == F.stub_z[0] + 1.0
    assert F.nut_seat_z == F.post_z[0] - 1.0 and (F.pulley_screw_len, F.pulley_screw_bought) == (33.0, 35.0)


@pytest.fixture(scope="module")
def flange():
    return built.part("wrist_flange")


@pytest.mark.slow
def test_the_build_matches_its_numbers(flange):
    assert flange.is_valid and len(flange.solids()) == 1
    assert R.solid_volume(flange) == pytest.approx(VOLUME, abs=0.01)
    assert R.bbox_min(flange) == pytest.approx(BBOX_MIN, abs=1e-3) and R.bbox_size(flange) == pytest.approx(BBOX_SIZE, abs=1e-3)


@pytest.mark.slow
def test_post_shoulder_stub_and_bores(flange):
    """About the pitch axis (part (pitch_x, 0), along Z): the post Ø post_dia, cut flat at wrist_link's front face and
    rear, its foot toward the bearing chamfered; the shoulder; the stub; the pin bore open at the stub's end and blind;
    the 4 M4 clearance holes end to end; the octagonal recess in its foot, key_fit over the key, a flat on each axis."""
    f, S = flange, stack_positions(DEFAULT)
    zp, zh, zs = sum(F.post_z) / 2.0 - 0.2, sum(F.step_z) / 2.0, sum(F.stub_z) / 2.0
    assert is_inside(f, PX, F.post_dia / 2.0 - 0.3, zp) and not is_inside(f, PX, F.post_dia / 2.0 + 0.3, zp)
    for x0, s in ((S["z_front"], 1.0), (D.rear_low, -1.0)):                                  # the flats
        assert is_inside(f, PX + x0 - 0.3 * s, 0.0, zp) and not is_inside(f, PX + x0 + 0.3 * s, 0.0, zp), x0
    assert not is_inside(f, PX, F.post_dia / 2.0 - F.post_chamfer / 2.0 + 0.2, F.post_z[1] - F.post_chamfer / 4.0)   # the chamfer
    assert is_inside(f, PX, F.step_dia / 2.0 - 0.2, zh) and not is_inside(f, PX, F.step_dia / 2.0 + 0.2, zh)
    assert is_inside(f, PX, F.stub_dia / 2.0 - 0.3, zs) and not is_inside(f, PX, F.stub_dia / 2.0 + 0.3, zs)
    assert not is_inside(f, PX, 0.0, zs) and not is_inside(f, PX, F.pin_bore_dia / 2.0 - 0.3, zs)   # the pin bore ...
    assert is_inside(f, PX, 0.0, F.pin_bore_z1 - 0.5)                                             # ... blind
    for x, y in pulley_bolt_points(DEFAULT):
        for z in (F.post_z[0] + 0.5, zh, zs, F.stub_z[1] - 0.5):
            assert not is_inside(f, x, y, z), (x, y, z)
        r = math.hypot(x - PX, y)
        assert is_inside(f, PX + (x - PX) * (r + 3.0) / r, y * (r + 3.0) / r, zs), (x, y)        # the stub's wall beside it
    zr, flat = F.post_z[0] + F.key_h / 2.0, F.key_af / 2.0 + F.key_fit
    for dx, dy in ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)):
        assert not is_inside(f, PX + dx * (flat - 0.1), dy * (flat - 0.1), zr)
        assert is_inside(f, PX + dx * (flat + 0.2), dy * (flat + 0.2), zr)


@pytest.mark.slow
def test_it_sits_on_wrist_link():
    """In the arm: the flange on wrist_link's coupler face (its foot there, wrist_link's top the key's), the key in its
    recess - key_fit round it, no overlap; the wrist 90T's screws (tests/test_mounts.py) clamp them."""
    flange, wrist = built.placed("wrist_flange#1"), built.placed("wrist_link#1")
    assert interference(flange, wrist) < 1e-3
    part, frame = built.part("wrist_flange"), built.part("wrist_link")
    assert math.isclose(part.bounding_box().min.Z, F.post_z[0], abs_tol=1e-6)
    assert math.isclose(frame.bounding_box().max.Z, F.post_z[0] + F.key_h, abs_tol=1e-6)


def _round_faces_about(shape, axis) -> set[float]:
    """The radii of the shape's cylindrical and conical faces whose axis runs along `axis` (a unit vector)."""
    out = set()
    for face in shape.faces():
        if face.geom_type not in (GeomType.CYLINDER, GeomType.CONE):
            continue
        d = face.axis_of_rotation.direction
        if abs(abs(d.X * axis[0] + d.Y * axis[1] + d.Z * axis[2]) - 1.0) < 1e-6:
            out.add(round(face.radius, 3) if face.geom_type == GeomType.CYLINDER else -1.0)
    return out


@pytest.mark.slow
def test_wrist_link_keeps_nothing_round_about_the_pitch_axis():
    """One print direction per part: wrist_link prints its bearing seats upright, so of everything round about the pitch
    axis it keeps only the 90T's M4 clearance holes - the post, the shoulder, the stub and the foot's old arc are gone
    (the foot an octagon); the flange carries them all."""
    assert _round_faces_about(built.part("wrist_link"), (0.0, 0.0, 1.0)) == {round(M4_CLEAR / 2.0, 3)}
    assert {round(r, 1) for r in _round_faces_about(built.part("wrist_flange"), (0.0, 0.0, 1.0))} >= {
        round(F.post_dia / 2.0, 1), round(F.step_dia / 2.0, 1), round(F.stub_dia / 2.0, 1)}
