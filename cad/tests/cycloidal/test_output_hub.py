"""Output hub (ported from cycloidal_drive tests/test_output_hub.py).

  1. Dimensional checks - clearances, fit relationships, hole spacing (config only)
  2. The built solid - topology, bounding box, feature probes, volume
"""
import math

import pytest

import parts
from lib import reference as R
from lib.cycloidal import hub_flange, hub_height, stack_positions
from tests.cycloidal.helpers import CFG
from tests.helpers import is_inside

cycloidal_output_hub = parts.load("cycloidal_output_hub")


class TestOutputHubDimensions:

    def test_hub_od_near_bearing_bore(self):
        """The hub OD is the 70 mm 6814 bore + 0.3 mm interference: within 0.5 mm."""
        assert abs(CFG.output_hub.od - CFG.bearings.out_bore) <= 0.5

    def test_hub_od_is_press_fit(self):
        assert abs(CFG.bearings.out_bore - CFG.output_hub.od) < 0.5

    def test_hub_height_matches_bearing_stack(self):
        """The turning shell's hub carries one 6814 (the other is on the motor plate's sleeve), past its flange."""
        assert CFG.stack_up.output_bearing_total == CFG.bearings.out_width
        assert hub_height(CFG) == hub_flange(CFG) + CFG.bearings.out_width + CFG.shell.end_lip + CFG.output_hub.proud_above_housing == 23.0

    def test_proud_extension_reaches_the_yoke(self):
        """The turning shell's hub runs on past the shell (z 61) onto the yoke's hub-side leg (z 62); the port's sat
        5 mm proud (spec 5.3)."""
        hub = CFG.output_hub
        assert hub.proud_above_housing == CFG.shell.end_plate_gap == 1.0
        assert stack_positions(CFG)["z_hub"] + hub_height(CFG) == pytest.approx(62.0)

    def test_shaft_bore_clears_spine(self):
        clearance = CFG.output_hub.shaft_clearance_bore - CFG.shaft.spine_od
        assert clearance >= 0.2

    def test_625_pocket_clears_shaft_bore(self):
        assert CFG.bearings.inp_od + CFG.tolerances.bearing_seat_bore_add > CFG.output_hub.shaft_clearance_bore

    def test_625_pocket_inside_hub(self):
        pocket_r = (CFG.bearings.inp_od + CFG.tolerances.bearing_seat_bore_add) / 2.0
        assert CFG.output_hub.od / 2.0 - pocket_r >= 5.0

    def test_output_pin_holes_inside_hub(self):
        d = CFG.disc
        assert d.output_pin_circle_dia / 2.0 + d.output_pin_dia / 2.0 < CFG.output_hub.od / 2.0

    def test_output_pin_holes_clear_625_pocket(self):
        d = CFG.disc
        pin_inner = d.output_pin_circle_dia / 2.0 - d.output_pin_dia / 2.0
        pocket_r = (CFG.bearings.inp_od + CFG.tolerances.bearing_seat_bore_add) / 2.0
        assert pin_inner - pocket_r >= 2.0

    def test_output_pin_holes_clear_bearing_bore(self):
        d = CFG.disc
        assert d.output_pin_circle_dia / 2.0 + d.output_pin_dia / 2.0 < CFG.bearings.out_bore / 2.0

    def test_output_pin_hole_is_clearance(self):
        """The hub pin holes are clearance holes (ring-pin convention, 0.10..0.30 mm in PETG)."""
        d, tol = CFG.disc, CFG.tolerances
        hole_dia = d.output_pin_dia - tol.ring_pin_press_sub
        assert hole_dia > d.output_pin_dia
        assert 0.10 <= hole_dia - d.output_pin_dia <= 0.30


@pytest.fixture(scope="module")
def hub_solid():
    return cycloidal_output_hub.build()


@pytest.mark.slow
class TestSolid:

    def test_solid_is_valid(self, hub_solid):
        assert len(hub_solid.solids()) == 1
        assert hub_solid.is_valid

    def test_outer_diameter(self, hub_solid):
        """The flange (ShellParams.plate_dia, the motor plate's) sets the box; the grip is the hub's od."""
        size = hub_solid.bounding_box().size
        assert abs(size.X - CFG.shell.plate_dia) < 0.2
        assert abs(size.Y - CFG.shell.plate_dia) < 0.2

    def test_the_flange(self, hub_solid):
        """Solid out to the flange over local z 0..9, its face on the 6814 cut back over the outer race (the inner race
        bears inside the relief); only the od over the 6814's grip and on."""
        sh, f = CFG.shell, hub_flange(CFG)
        r_relief = (sh.plate_relief_dia + sh.plate_dia) / 4.0
        assert is_inside(hub_solid, 0.0, r_relief, f / 2.0, 1e-3), "no flange"
        assert not is_inside(hub_solid, 0.0, r_relief, f - sh.plate_relief_depth / 2.0, 1e-3), "no relief over the outer race"
        assert is_inside(hub_solid, 0.0, sh.plate_relief_dia / 2.0 - 0.5, f - 0.2, 1e-3), "no face for the inner race"
        r = (CFG.output_hub.od + sh.plate_relief_dia) / 4.0
        assert not is_inside(hub_solid, 0.0, r, f + 1.0, 1e-3), "the flange runs on under the 6814"

    def test_height(self, hub_solid):
        """Z = bearing grip + output wall + proud extension (42.27)."""
        assert abs(hub_solid.bounding_box().size.Z - hub_height(CFG)) < 0.1

    def test_output_face_proud_of_housing(self, hub_solid):
        """Placed at z_hub, the top face lands at total_housing_depth + proud (62)."""
        stack, hub = CFG.stack_up, CFG.output_hub
        global_top = stack_positions(CFG)["z_hub"] + hub_solid.bounding_box().max.Z
        assert abs(global_top - (stack.total_housing_depth + hub.proud_above_housing)) < 0.1

    def test_arm_mount_holes_through(self, hub_solid):
        """The 4 arm-mount holes pass fully through (empty just inside both faces)."""
        hub = CFG.output_hub
        bb = hub_solid.bounding_box()
        arm_r, off = hub.arm_mount_bolt_circle_dia / 2.0, math.radians(hub.arm_mount_angle_offset_deg)
        for i in range(hub.arm_mount_bolt_count):
            a = off + 2 * math.pi * i / hub.arm_mount_bolt_count
            x, y = arm_r * math.cos(a), arm_r * math.sin(a)
            for z in (bb.min.Z + 0.5, bb.max.Z - 0.5):
                assert not is_inside(hub_solid, x, y, z, 1e-3), f"Arm-mount hole at {math.degrees(a):.0f} deg, z={z:.1f} not open"

    def test_arm_mount_nut_pockets_present(self, hub_solid):
        """Captive hex pockets on the inner face only: a point between the bolt-hole radius and
        the hex apothem is empty near the inner face and solid up in the proud section."""
        hub, h, tol = CFG.output_hub, CFG.housing, CFG.tolerances
        bb = hub_solid.bounding_box()
        arm_r, off = hub.arm_mount_bolt_circle_dia / 2.0, math.radians(hub.arm_mount_angle_offset_deg)
        apothem = h.bolt_nut_pocket_af / 2.0
        hole_r = (h.bolt_dia + tol.bolt_clearance_add) / 2.0
        probe_r = arm_r + (apothem + hole_r) / 2.0
        x, y = probe_r * math.cos(off), probe_r * math.sin(off)
        assert not is_inside(hub_solid, x, y, CFG.shell.hub_nut_depth - 0.3, 1e-3), "No hex nut pocket near the inner face"
        assert is_inside(hub_solid, x, y, bb.max.Z - 1.0, 1e-3), "Nut pocket extends too far - the proud section should be solid"

    def test_pin_holes_blind_from_output_face(self, hub_solid):
        """A 1 mm ceiling stays solid above the pin holes (probe mid-ceiling above pin #1)."""
        probe_z = CFG.stack_up.z_bearing_top - stack_positions(CFG)["z_hub"] - CFG.output_hub.output_hub_pin_ceiling / 2.0
        assert is_inside(hub_solid, CFG.disc.output_pin_circle_dia / 2.0, 0.0, probe_z, 1e-3), "blind ceiling above the output pins is missing"

    def test_central_lightening_pocket(self, hub_solid):
        """Empty at the centre near the top, solid floor above the grip zone, solid wall out at
        the arm-bolt circle."""
        hub, stack = CFG.output_hub, CFG.stack_up
        if hub.arm_mount_pocket_dia <= 0:
            pytest.skip("lightening pocket disabled")
        bb = hub_solid.bounding_box()
        assert not is_inside(hub_solid, 0, 0, bb.max.Z - 0.5, 1e-4), "central lightening pocket missing"
        z_floor = stack.z_bearing_top - stack_positions(CFG)["z_hub"] + hub.arm_mount_pocket_floor
        assert is_inside(hub_solid, 0, 0, z_floor - 0.3, 1e-4), "pocket floor missing"
        assert is_inside(hub_solid, hub.arm_mount_bolt_circle_dia / 2.0, 0, bb.max.Z - 0.5, 1e-4), "pocket should not reach the arm-bolt circle"

    def test_volume_sanity(self, hub_solid):
        """Between the cylinder minus every feature (generously) and the cylinder minus the shaft bore."""
        hub, d, b, tol, stack, h = CFG.output_hub, CFG.disc, CFG.bearings, CFG.tolerances, CFG.stack_up, CFG.housing
        hub_r, height, flange = hub.od / 2.0, hub_height(CFG), hub_flange(CFG)
        grip = stack.z_bearing_top - stack_positions(CFG)["z_hub"]       # the pins' holes' reach + the ceiling
        shaft_r = hub.shaft_clearance_bore / 2.0
        upper = math.pi * (hub_r ** 2 * height + ((CFG.shell.plate_dia / 2.0) ** 2 - hub_r ** 2) * flange - shaft_r ** 2 * flange)
        pocket_vol = math.pi * ((b.inp_od + tol.bearing_seat_bore_add) / 2.0) ** 2 * b.inp_width
        pin_vol = d.output_pin_count * math.pi * ((d.output_pin_dia - tol.ring_pin_press_sub) / 2.0) ** 2 * (grip - hub.output_hub_pin_ceiling)
        arm_hole_vol = hub.arm_mount_bolt_count * math.pi * ((h.bolt_dia + tol.bolt_clearance_add) / 2.0) ** 2 * height
        nut_pocket_vol = hub.arm_mount_bolt_count * math.pi * (h.bolt_nut_pocket_af / 2.0) ** 2 * h.bolt_nut_depth
        lightening_vol = math.pi * (hub.arm_mount_pocket_dia / 2.0) ** 2 * (height - grip - hub.arm_mount_pocket_floor) if hub.arm_mount_pocket_dia > 0 else 0.0
        lower = (upper - pocket_vol - pin_vol - arm_hole_vol - nut_pocket_vol - lightening_vol) * 0.9
        vol = R.solid_volume(hub_solid)
        assert lower < vol < upper, f"Volume {vol:.0f}mm^3 outside ({lower:.0f}, {upper:.0f})"
