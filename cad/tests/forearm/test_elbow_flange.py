"""elbow_flange, the elbow's output flange - its own part under the forearm roll frame's underside (lib/forearm/roll.py
build_elbow_flange): a part with no reference file (lib/reference.py NO_REFERENCE), so THIS file locks its numbers - the
volume and the bounding box, measured on the build and written in - and what it is: the post, the shoulder and the stub
about the elbow axis, the pin bore, the elbow 90T's 4x M4 through it, the recess in its top over the frame's spigot; and
how it sits in the drive: on the frame's underside, its recess spigot_fit round the spigot, nothing of the two in each
other, the holes on the frame's nut channels."""
import math

import pytest
from build123d import GeomType

from assemblies import forearm_roll_drive as M
from lib import reference as R
from lib.forearm import DEFAULT, pulley_bolt_points, stack_positions
from tests import built
from tests.helpers import interference, is_inside

D, S = DEFAULT.drive, stack_positions(DEFAULT)
YE = S["y_elbow"]
VOLUME = 9981.77                                     # mm^3, the build's
BBOX_MIN = (-45.0, -94.0, -20.0)                     # the module frame (assemblies/forearm_roll_drive.py places it at 0)
BBOX_SIZE = (15.0, 40.0, 40.0)


def _corner(af: float) -> float:
    """An octagon's corner radius, af across flats."""
    return af / 2.0 / math.cos(math.pi / D.end_sides)


def test_params():
    """The flange spans the post's foot to the stub's end; its top is the frame's underside, the spigot's recess in it -
    inside the post all round, the nuts' channels inside the spigot -; the pin bore blind under the recess."""
    assert R.NO_REFERENCE["elbow_flange"] == "lib/forearm/roll.py:build_elbow_flange(DEFAULT)"
    assert D.post_x[1] == D.block_x[0] and D.stub_x[0] == BBOX_MIN[0] and BBOX_SIZE[0] == D.block_x[0] - D.stub_x[0]
    assert D.spigot_h < D.post_x[1] - D.post_x[0] - 3.0                         # the recess in the post alone, 3.3 of it left under
    assert D.post_dia / 2.0 - _corner(D.spigot_af + 2.0 * D.spigot_fit) >= 2.5   # the recess's corners inside the post
    assert D.pulley_bolt_r + D.nut_af / 2.0 <= D.spigot_af / 2.0 - 1.5           # each nut's outer flat inside the spigot's
    assert D.pin_bore_x[1] <= D.block_x[0] - D.spigot_h - 3.0 and D.pin_bore_x[0] == D.stub_x[0]
    assert D.nut_seat_x == D.block_x[0] - D.spigot_h + 1.0                       # in the spigot, 1 of it under each nut


@pytest.fixture(scope="module")
def flange():
    return built.part("elbow_flange")


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


@pytest.mark.slow
def test_the_build_matches_its_numbers(flange):
    assert flange.is_valid and len(flange.solids()) == 1
    assert R.solid_volume(flange) == pytest.approx(VOLUME, abs=0.01)
    assert R.bbox_min(flange) == pytest.approx(BBOX_MIN, abs=1e-3) and R.bbox_size(flange) == pytest.approx(BBOX_SIZE, abs=1e-3)


@pytest.mark.slow
def test_post_shoulder_stub_and_bores(flange):
    """About the elbow axis (module (y_elbow, 0), along X): the post Ø post_dia, its foot chamfered; the shoulder; the
    stub; the pin bore open at the stub's end and blind; the 4 M4 clearance holes end to end; the octagonal recess in
    the top, spigot_fit over the spigot, a flat on each axis."""
    f = flange
    xp, xs = (D.post_x[0] + D.post_x[1]) / 2.0 - 1.0, (D.stub_x[0] + D.stub_x[1]) / 2.0
    assert is_inside(f, xp, YE, D.post_dia / 2.0 - 0.3) and not is_inside(f, xp, YE, D.post_dia / 2.0 + 0.3)
    assert not is_inside(f, D.post_x[0] + 0.2, YE, D.post_dia / 2.0 - 0.3)                    # the foot's chamfer
    assert is_inside(f, (D.step_x[0] + D.step_x[1]) / 2.0, YE, D.step_dia / 2.0 - 0.2)
    assert not is_inside(f, (D.step_x[0] + D.step_x[1]) / 2.0, YE, D.step_dia / 2.0 + 0.2)
    assert is_inside(f, xs, YE, D.stub_dia / 2.0 - 0.3) and not is_inside(f, xs, YE, D.stub_dia / 2.0 + 0.3)
    assert not is_inside(f, xs, YE, 0.0) and not is_inside(f, xs, YE, D.pin_bore_dia / 2.0 - 0.3)   # the pin bore ...
    assert is_inside(f, D.pin_bore_x[1] + 0.5, YE, 0.0)                                          # ... blind
    for y, z in pulley_bolt_points(DEFAULT):
        for x in (D.stub_x[0] + 0.5, xs, xp, D.block_x[0] - D.spigot_h - 0.5):
            assert not is_inside(f, x, YE + y, z), (y, z, x)
        r = D.pulley_bolt_r
        assert is_inside(f, xs, YE + y * (r + 3.0) / r, z * (r + 3.0) / r), (y, z)                  # the stub's wall beside it
    xr, flat = D.block_x[0] - D.spigot_h / 2.0, D.spigot_af / 2.0 + D.spigot_fit
    for dy, dz in ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)):
        assert not is_inside(f, xr, YE + dy * (flat - 0.1), dz * (flat - 0.1))
        assert is_inside(f, xr, YE + dy * (flat + 0.2), dz * (flat + 0.2))
    assert is_inside(f, D.block_x[0] - D.spigot_h - 0.5, YE, D.spigot_af / 2.0 - 1.0)            # the recess's floor


@pytest.mark.slow
def test_the_frame_keeps_nothing_round_about_the_elbow_axis():
    """One print direction per part: the roll frame prints its 6806 seats upright (front face down), so of everything
    round about the elbow axis (module X) it keeps only the 90T's M4 clearance holes; the flange carries the rest."""
    radii = set()
    for face in built.part("forearm_roll_block").faces():
        if face.geom_type in (GeomType.CYLINDER, GeomType.CONE) and abs(abs(face.axis_of_rotation.direction.X) - 1.0) < 1e-6:
            radii.add(round(face.radius, 3) if face.geom_type == GeomType.CYLINDER else -1.0)
    assert radii == {round(D.pulley_bolt_dia / 2.0, 3)}


@pytest.mark.slow
def test_it_sits_on_the_frame(module):
    """In the drive: the flange's top on the frame's underside, the spigot in its recess - spigot_fit round it, no
    overlap -, the frame's nut channels over the flange's holes; the elbow 90T's screws (tests/test_mounts.py) clamp them."""
    flange, block = _leaf(module, "elbow_flange"), _leaf(module, "forearm_roll_block")
    assert interference(flange, block) < 1e-3
    assert math.isclose(flange.bounding_box().max.X, D.block_x[0], abs_tol=1e-6)
    assert math.isclose(block.bounding_box().min.X, D.block_x[0] - D.spigot_h, abs_tol=1e-6)
    xr = D.block_x[0] - D.spigot_h / 2.0
    for dy, dz in ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)):               # the spigot's flats, the gap round them
        assert is_inside(block, xr, YE + dy * (D.spigot_af / 2.0 - 0.1), dz * (D.spigot_af / 2.0 - 0.1))
        assert not is_inside(block, xr, YE + dy * (D.spigot_af / 2.0 + 0.1), dz * (D.spigot_af / 2.0 + 0.1))
        assert not is_inside(flange, xr, YE + dy * (D.spigot_af / 2.0 + 0.1), dz * (D.spigot_af / 2.0 + 0.1))
