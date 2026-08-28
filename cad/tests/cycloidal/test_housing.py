"""The shared housing profile (ported from cycloidal_drive tests/test_housing_profile.py).

lib/cycloidal/housing.py defines the 8-pillar / 8-window outer profile shared by the motor
plate and the ring gear body and the outer-silhouette chamfer. Any change to the profile
(pillar dims, bolt angles, bore radii) is caught here before the dependent part tests.
"""
import math

import pytest

from tests.cycloidal.helpers import is_inside, no_chamfer
from lib import reference as R
from lib.cycloidal import DEFAULT_CONFIG, compute_housing_bolt_angles
from lib.cycloidal.housing import chamfer_outer_silhouette, reveal_window_cutter
from parts import cycloidal_motor_plate, cycloidal_ring_gear_body

CFG = DEFAULT_CONFIG


@pytest.fixture(scope="module")
def cutter():
    return reveal_window_cutter(CFG, height=10.0)


@pytest.fixture(scope="module")
def sample_parts():
    # (name, part, z near mid-thickness of the pillar zone)
    return [("motor_plate", cycloidal_motor_plate.build(), 5.0), ("ring_gear_body", cycloidal_ring_gear_body.build(), 23.5)]


@pytest.fixture(scope="module")
def chamfer_parts():
    # name -> (build fn, thickness, external_z, [mating_z, ...])
    return {
        "motor_plate": (cycloidal_motor_plate.build, 9.0, 0.0, [9.0]),
        "ring_gear_body": (cycloidal_ring_gear_body.build, 51.0, 51.0, [0.0]),
    }


@pytest.mark.slow
class TestCutter:

    def test_solid_is_valid(self, cutter):
        """8 pillars subtracted from the annulus leave 8 disjoint window solids."""
        solids = cutter.solids()
        assert len(solids) == CFG.housing.bolt_count, f"Expected {CFG.housing.bolt_count} solids, got {len(solids)}"
        for s in solids:
            assert s.is_valid

    def test_bounding_box(self, cutter):
        """XY span between the bore and the OD overshoot (the pillars eat into the outer ring at the
        bolt axes); Z span exact."""
        size = cutter.bounding_box().size
        assert CFG.housing.bore_dia < size.X < CFG.housing.od + 0.2
        assert CFG.housing.bore_dia < size.Y < CFG.housing.od + 0.2
        assert abs(size.Z - 10.0) < 0.05

    def test_volume_sanity(self, cutter):
        """Cutter volume ~ annular wall - 8 trapezoidal pillars (15 % margin)."""
        h = CFG.housing
        housing_r, bore_r, height = h.od / 2.0, h.bore_dia / 2.0, 10.0
        annular = math.pi * (housing_r ** 2 - bore_r ** 2) * height
        pillar_area = (h.pillar_inner_w + h.pillar_outer_w) / 2.0 * (housing_r - bore_r)
        expected = annular - h.bolt_count * pillar_area * height
        vol = R.solid_volume(cutter)
        assert abs(vol - expected) / expected < 0.15, f"Cutter volume {vol:.0f}mm^3 vs expected {expected:.0f}mm^3"


@pytest.mark.slow
class TestProfileAlignment:
    """Both housing parts show solid at the pillar centres and void at the window centres at
    the same (r, theta)."""

    def test_pillar_centres_are_solid(self, sample_parts):
        parts = sample_parts
        """On the bolt circle at every bolt angle, 4 mm tangentially off the M4 hole: material."""
        bolt_r, offset = CFG.housing.bolt_circle_dia / 2.0, 4.0
        for angle in compute_housing_bolt_angles(CFG):
            x = bolt_r * math.cos(angle) - offset * math.sin(angle)
            y = bolt_r * math.sin(angle) + offset * math.cos(angle)
            for name, part, z in parts:
                assert is_inside(part, x, y, z), f"{name}: expected solid at pillar centre angle={math.degrees(angle):.1f} deg"

    def test_window_centres_are_void(self, sample_parts):
        parts = sample_parts
        """Midway between bolt angles at r=64 (pillar_inner_r 57 < r < housing_r 70): void."""
        step, sample_r = 2 * math.pi / CFG.housing.bolt_count, 64.0
        for angle in compute_housing_bolt_angles(CFG):
            mid = angle + step / 2.0
            x, y = sample_r * math.cos(mid), sample_r * math.sin(mid)
            for name, part, z in parts:
                assert not is_inside(part, x, y, z), f"{name}: expected void at window centre angle={math.degrees(mid):.1f} deg"


@pytest.mark.slow
class TestOuterChamfer:
    """Both parts bevel their outer silhouette while the faces that mate against a neighbour
    stay sharp: the motor plate's motor face (z=0) and the ring gear body's output face (z=51)
    bevel their whole perimeter; the inner mating faces stay sharp."""

    PILLAR_TIP_R = 69.5   # just inside the 70 mm pillar outer face, at a pillar centre

    def test_edge_chamfer_param(self):
        assert CFG.housing.edge_chamfer == 1.5

    def test_parts_valid_od_thickness(self, chamfer_parts):
        parts = chamfer_parts
        """One valid solid; OD and thickness unchanged - only the corners are broken."""
        for name, (fn, th, ext, mates) in parts.items():
            part = fn()
            assert len(part.solids()) == 1 and part.is_valid, f"{name}: not a single valid solid"
            size = part.bounding_box().size
            assert abs(size.X - CFG.housing.od) < 0.2, f"{name}: OD {size.X:.2f}"
            assert abs(size.Z - th) < 0.05, f"{name}: thickness {size.Z:.2f}"

    def test_chamfer_reduces_volume(self, chamfer_parts):
        parts = chamfer_parts
        cfg0 = no_chamfer(CFG)
        for name, (fn, th, ext, mates) in parts.items():
            assert R.solid_volume(fn()) < R.solid_volume(fn(cfg0)), f"{name}: chamfer removed no material"

    def test_external_rim_beveled_but_mating_face_sharp(self, chamfer_parts):
        parts = chamfer_parts
        for name, (fn, th, ext, mates) in parts.items():
            part = fn()
            z_ext = ext + 0.1 if ext == 0 else ext - 0.1
            assert not is_inside(part, self.PILLAR_TIP_R, 0.0, z_ext), f"{name}: external rim should be beveled at the pillar tip"
            mz = mates[0]
            z_mate = mz + 0.1 if mz == 0 else mz - 0.1
            assert is_inside(part, self.PILLAR_TIP_R, 0.0, z_mate), f"{name}: mating face must stay sharp"

    def test_ring_gear_body_mating_face_sharp(self, chamfer_parts):
        parts = chamfer_parts
        fn, th, ext, mates = parts["ring_gear_body"]
        assert is_inside(fn(), self.PILLAR_TIP_R, 0.0, mates[0] + 0.1)

    def test_full_external_perimeter_beveled(self):
        """Passing external_z bevels the whole perimeter (pillar sides + inner arcs), removing
        materially more than the barrel verticals alone."""
        cfg0 = no_chamfer(CFG)
        for fn, external_z in ((cycloidal_motor_plate.build, 0.0), (cycloidal_ring_gear_body.build, 51.0)):
            base = fn(cfg0)
            verticals_only = chamfer_outer_silhouette(base, CFG, external_z=None)
            full = chamfer_outer_silhouette(base, CFG, external_z=external_z)
            assert R.solid_volume(full) < R.solid_volume(verticals_only) - 50.0

    def test_chamfer_can_be_disabled(self):
        part = cycloidal_motor_plate.build(no_chamfer(CFG))
        assert is_inside(part, self.PILLAR_TIP_R, 0.0, 0.1)
