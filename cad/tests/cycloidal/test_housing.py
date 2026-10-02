"""The shared housing profile (ported from cycloidal_drive tests/test_housing_profile.py).

lib/cycloidal/housing.py defines the pillar / window outer profile (one per housing bolt) shared by the housing parts
- the port's motor plate and ring gear body, the turning shell's shell ring and body (build_shell_body, printed with
j1_link) - and the outer-silhouette chamfer. Any change to the profile (pillar dims, bolt angles, bore radii) is caught
here before the dependent part tests; DEFAULT's two shell parts stand for them.
"""
import math

import pytest
from build123d import Axis

import parts
from lib import reference as R
from lib.cycloidal import arm_zone, compute_housing_bolt_angles, shell_ends
from lib.cycloidal.housing import CUTTER_OVERSHOOT, build_shell_body, chamfer_outer_silhouette, reveal_window_cutter
from tests.cycloidal.helpers import CFG, no_chamfer
from tests.helpers import is_inside

cycloidal_shell_ring = parts.load("cycloidal_shell_ring")
(END_0, END_1), JOINT = shell_ends(CFG), arm_zone(CFG)[0]       # -13, 61: the shell's ends; 9: the ring on the body
START = math.radians(CFG.housing.bolt_start_deg)                 # the first pillar, on the upper arm's centreline
STEP = 2 * math.pi / CFG.housing.bolt_count


def _aligned(part):
    """`part` turned so its first pillar lies on +X (and so, with 6, one on -X): its X span is the od."""
    return part.rotate(Axis.Z, -CFG.housing.bolt_start_deg)


def _under_the_arm(mid: float) -> bool:
    """The window centred at `mid` is one the body keeps solid under the upper arm (ShellParams.arm_windows)."""
    off = math.remainder(mid - START, 2 * math.pi)
    return abs(off) < CFG.shell.arm_windows / 2.0 * STEP


@pytest.fixture(scope="module")
def cutter():
    return reveal_window_cutter(CFG, height=10.0)


@pytest.fixture(scope="module")
def sample_parts():
    # (name, part, z near mid-thickness of the pillar zone)
    return [("shell_ring", cycloidal_shell_ring.build(), 5.0), ("body", build_shell_body(), 24.0)]


@pytest.fixture(scope="module")
def chamfer_parts():
    # name -> (build fn, thickness, (external_z, its inward sense), (mating_z, its inward sense))
    return {
        "shell_ring": (cycloidal_shell_ring.build, JOINT - END_0, (END_0, 1.0), (JOINT, -1.0)),
        "body": (build_shell_body, END_1 - JOINT, (END_1, -1.0), (JOINT, 1.0)),
    }


@pytest.mark.slow
class TestCutter:

    def test_solid_is_valid(self, cutter):
        """bolt_count pillars subtracted from the annulus leave as many disjoint window solids."""
        solids = cutter.solids()
        assert len(solids) == CFG.housing.bolt_count, f"Expected {CFG.housing.bolt_count} solids, got {len(solids)}"
        for s in solids:
            assert s.is_valid

    def test_bounding_box(self, cutter):
        """XY span between the bore and the OD overshoot (turned so the first pillar is on X: the pillars eat into the
        outer ring at the bolt axes - on X; a window on Y reaches the overshoot); Z span exact."""
        size = _aligned(cutter).bounding_box().size
        span = CFG.housing.od + 2.0 * CUTTER_OVERSHOOT
        assert CFG.housing.bore_dia < size.X < span
        assert CFG.housing.bore_dia < size.Y <= span + 1e-6
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
        """Midway between bolt angles at r=64 (bore 54 < r < od 64.6): void - but the body's windows under the upper arm,
        solid (ShellParams.arm_windows)."""
        sample_r = 64.0
        for angle in compute_housing_bolt_angles(CFG):
            mid = angle + STEP / 2.0
            x, y = sample_r * math.cos(mid), sample_r * math.sin(mid)
            for name, part, z in parts:
                solid = name == "body" and _under_the_arm(mid)
                assert is_inside(part, x, y, z) == solid, f"{name}: window centre angle={math.degrees(mid):.1f} deg, solid {solid}"


@pytest.mark.slow
class TestOuterChamfer:
    """Both parts bevel their outer silhouette while the faces that mate against a neighbour
    stay sharp: the shell's two ends (the shell ring's z=-13, the body's z=61) bevel their whole perimeter; the faces
    where they meet (z=9) stay sharp."""

    PILLAR_TIP_R = CFG.housing.od / 2.0 - 0.5   # just inside the pillar's outer face, at a pillar centre

    def tip(self, z: float) -> tuple:
        """The first pillar's tip at height z."""
        return self.PILLAR_TIP_R * math.cos(START), self.PILLAR_TIP_R * math.sin(START), z

    def test_edge_chamfer_param(self):
        assert CFG.housing.edge_chamfer == 1.0

    def test_parts_valid_od_thickness(self, chamfer_parts):
        parts = chamfer_parts
        """One valid solid; OD and thickness unchanged - only the corners are broken."""
        for name, (fn, th, _ext, _mates) in parts.items():
            part = fn()
            assert len(part.solids()) == 1 and part.is_valid, f"{name}: not a single valid solid"
            size = _aligned(part).bounding_box().size
            assert abs(size.X - CFG.housing.od) < 0.2, f"{name}: OD {size.X:.2f}"
            assert abs(size.Z - th) < 0.05, f"{name}: thickness {size.Z:.2f}"

    def test_chamfer_reduces_volume(self, chamfer_parts):
        parts = chamfer_parts
        cfg0 = no_chamfer(CFG)
        for name, (fn, _th, _ext, _mates) in parts.items():
            assert R.solid_volume(fn()) < R.solid_volume(fn(cfg0)), f"{name}: chamfer removed no material"

    def test_external_rim_beveled_but_mating_face_sharp(self, chamfer_parts):
        parts = chamfer_parts
        for name, (fn, _th, (ext, ext_in), (mate, mate_in)) in parts.items():
            part = fn()
            assert not is_inside(part, *self.tip(ext + 0.1 * ext_in)), f"{name}: external rim should be beveled at the pillar tip"
            assert is_inside(part, *self.tip(mate + 0.1 * mate_in)), f"{name}: mating face must stay sharp"

    def test_full_external_perimeter_beveled(self):
        """Passing external_z bevels the whole perimeter (pillar sides + inner arcs), removing
        materially more than the barrel verticals alone."""
        cfg0 = no_chamfer(CFG)
        for fn, external_z in ((cycloidal_shell_ring.build, END_0), (build_shell_body, END_1)):
            base = fn(cfg0)
            verticals_only = chamfer_outer_silhouette(base, CFG, external_z=None)
            full = chamfer_outer_silhouette(base, CFG, external_z=external_z)
            assert R.solid_volume(full) < R.solid_volume(verticals_only) - 50.0

    def test_chamfer_can_be_disabled(self):
        part = cycloidal_shell_ring.build(no_chamfer(CFG))
        assert is_inside(part, *self.tip(END_0 + 0.1))
