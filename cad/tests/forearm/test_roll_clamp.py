"""The forearm roll rotor's clamp (lib/forearm/params.py RollDriveParams clamp_*): 4x M3 SHCS (roll_clamp_screws) from
the pulley's front face through both pieces and both 6806 inner rings into 4 M3 nuts (roll_clamp_nuts) in the shaft's
flange - the pulley, the inner rings and the shaft one body. Purchased parts with no reference (lib/reference.py
NO_REFERENCE), which keep no reference file - THIS file locks their numbers (the pieces, the modelled volume and the
bounding box, measured on the build and written in) and what they are: the screw clamp_screw_len under the head, on
the clamp's circle; the nut a corner radial. The holes, counterbores and nut pockets are the printed parts'
(tests/forearm/test_roll_drive.py probes them). In the module: on the rotor; the heads on the counterbores' floors
under the pulley's face, the tips at the shaft's rear face, the nuts on their pockets' ends 2 pitches short of the tips,
held from turning by the pockets' flats - and clear of everything else in the drive."""
import math

import pytest
from build123d import Axis
from cadgen.geometry import closest_points

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.fasteners import nut_volume, shcs_volume
from lib.forearm import DEFAULT, clamp_points, stack_positions
from tests import built
from tests.helpers import interference

D, S = DEFAULT.drive, stack_positions(DEFAULT)
SCREWS, NUTS = "roll_clamp_screws", "roll_clamp_nuts"
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    SCREWS: (4, 1529.299, (-11.412058, -11.412058, -3.0), (22.824116, 22.824116, 48.0)),
    NUTS: (4, 183.635, (-11.729484, -11.729484, 0.0), (23.458968, 23.458968, 2.4)),
}


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def test_they_are_purchased_parts_with_no_reference_on_the_rotor():
    for name in (SCREWS, NUTS):
        mod = parts.load(name)
        assert parts.GROUPS[name] == "joints" and parts.bought(name) and mod.PURCHASE_QTY == len(clamp_points(DEFAULT)) == LOCKS[name][0]
        assert R.NO_REFERENCE[name] == f"parts/joints/{name}.py:_envelope()" and name not in parts.unplaced()
        assert name in M.BODIES["rotor"], name   # they turn with the pulley and the shaft: forearm_link
    # one row each in the drive: the screws turned over (the heads up the roll axis) on the counterbores' floor, the
    # nuts at their far faces' station
    rows = {part: (role, pos) for part, role, pos in M.OCCURRENCES if part in (SCREWS, NUTS)}
    assert rows == {SCREWS: (None, ((0.0, 0.0, S["z_clamp_head"]), (180.0, 0.0, 0.0))), NUTS: (None, (0.0, 0.0, S["z_clamp_nut"]))}


def test_the_clamps_screw_and_nut():
    """ISO 4762 M3, clamp_screw_len under the head; ISO 4032 M3; both on the clamp's circle (the screws' pattern
    mirrored in y: the row turns it over about X); the masses the modelled steel."""
    screws, nuts = parts.load(SCREWS), parts.load(NUTS)
    assert screws.SCREW == D.clamp_screw == PARAMS.M3_SHCS and nuts.NUT == D.clamp_nut == PARAMS.M3_NUT
    assert screws.LENGTH == D.clamp_screw_len == 45.0
    assert screws.PURCHASE_SPEC == "M3 x 45 socket head cap screw (ISO 4762)" and nuts.PURCHASE_SPEC == "M3 hex nut (ISO 4032)"
    assert sorted((x, -y) for x, y in screws.POINTS) == sorted(clamp_points(DEFAULT)) == sorted(nuts.POINTS)
    assert all(math.isclose(math.hypot(x, y), D.clamp_circle_dia / 2.0) for x, y in nuts.POINTS)
    n = LOCKS[SCREWS][0]
    assert screws.MASS_G == PARAMS.ROLL_CLAMP_SCREWS_MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * n * shcs_volume(D.clamp_screw, 45.0))
    assert nuts.MASS_G == PARAMS.ROLL_CLAMP_NUTS_MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * n * nut_volume(D.clamp_nut))


def test_the_stations():
    """The heads' bearing faces on the counterbores' floor, their tops clamp_head_clear under the pulley's front face;
    the tips at the shaft's rear face; each nut 2 pitches short of its tip, its bearing face on its pocket's end."""
    head_top = S["z_clamp_head"] + D.clamp_screw.head_h
    assert math.isclose(S["z_spigot_end"] - head_top, D.clamp_head_clear)
    assert math.isclose(S["z_clamp_head"] - D.clamp_screw_len, S["z_shaft_end"])
    assert math.isclose(S["z_clamp_nut"] - S["z_shaft_end"], 2 * PARAMS.M3_PITCH)
    assert math.isclose(S["z_clamp_nut"] + D.clamp_nut.h, S["z_clamp_nut_seat"])


@pytest.mark.slow
@pytest.mark.parametrize("name", [SCREWS, NUTS])
def test_the_build_matches_its_numbers(name):
    solids, volume, lo, size = LOCKS[name]
    shape = built.part(name)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    one = shcs_volume(D.clamp_screw, D.clamp_screw_len) if name == SCREWS else nut_volume(D.clamp_nut)
    assert R.solid_volume(shape) == pytest.approx(solids * one, rel=1e-6)


@pytest.mark.slow
def test_in_place_the_clamp_sits_on_its_seats_clear_of_the_drive(module):
    """The placed patterns: the heads from the counterbores' floors to clamp_head_clear under the pulley's face, the
    tips at the shaft's rear face, the nuts between their stations; the heads bear on the pulley, the nuts on the
    shaft, the shanks clear of the holes; no interference with anything else in the drive."""
    screws, nuts = _leaf(module, SCREWS), _leaf(module, NUTS)
    pulley, shaft = _leaf(module, "forearm_roll_pulley"), _leaf(module, "forearm_roll_shaft")
    bb = screws.bounding_box()
    assert bb.max.Z == pytest.approx(S["z_spigot_end"] - D.clamp_head_clear, abs=1e-6) and bb.min.Z == pytest.approx(S["z_shaft_end"], abs=1e-6)
    bb = nuts.bounding_box()
    assert bb.min.Z == pytest.approx(S["z_clamp_nut"], abs=1e-6) and bb.max.Z == pytest.approx(S["z_clamp_nut_seat"], abs=1e-6)
    assert closest_points(screws, pulley).distance < 0.01 and closest_points(nuts, shaft).distance < 0.01
    hole_gap = (DEFAULT.roll_end.bolt_dia - D.clamp_screw.d) / 2.0
    assert closest_points(screws, shaft).distance == pytest.approx(hole_gap, abs=0.01)
    for part in (screws, nuts):
        for other in module.children:
            if other is not part:
                assert interference(part, other) == pytest.approx(0.0, abs=1e-6), (part.label, other.label)


@pytest.mark.slow
def test_the_pockets_hold_the_nuts_from_turning(module):
    """Each nut a corner radial in its pocket's slot, af wide: a few degrees about its own axis runs its corners into
    the shaft's flange."""
    nuts, shaft = _leaf(module, NUTS), _leaf(module, "forearm_roll_shaft")
    for solid, (x, y) in zip(sorted(nuts.solids(), key=lambda s: math.atan2(s.center().Y, s.center().X)),
                             sorted(clamp_points(DEFAULT), key=lambda p: math.atan2(p[1], p[0])), strict=True):
        assert math.isclose(solid.center().X, x, abs_tol=1e-6) and math.isclose(solid.center().Y, y, abs_tol=1e-6)
        turned = solid.rotate(Axis((x, y, 0.0), (0.0, 0.0, 1.0)), 5.0)
        assert interference(turned, shaft) > 0.1, (x, y)
