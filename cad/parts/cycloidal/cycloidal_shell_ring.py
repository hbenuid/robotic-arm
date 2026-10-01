"""cycloidal_shell_ring - the turning shell's motor end (built at its stack position: z -12..9), PETG.

Designed here (docs/cycloidal_drive.md §5.5), no reference: tests/cycloidal/test_shell_ring.py holds its numbers.
The turning shell (lib/cycloidal/params.py ShellParams) takes the port's motor plate apart: the plate that holds the
motor stays with the carrier (parts/cycloidal/cycloidal_motor_plate), and this ring - turning with the ring gear body
it is bolted to - keeps the plate's housing half: the ring from the motor-plate face (z=0) to the body (z=9), the
housing's od with its pillars and windows, a ring_bore_dia bore round the held plate, the 21 ring-pin through-holes
(the pins go in one at a time from z=0, the body's blind holes hold their far ends), the housing bolts' holes with their
heads' counterbores in z=0; and behind it a skirt round the motor end of the carrier's sleeve: skirt_od, the 6814 seat
(HousingParams.output_bearing_seat_dia) out_width deep - the motor-end 6814's outer race, pressed in from z=0 - and a
skirt_lip at its end (HousingParams.lip_bore_dia), the race's stop. The z=0 face's outer silhouette and the pillars'
corners chamfered like the port's plate; the face on the body (z=9) sharp.
"""
import pathlib

from cadgen import step

from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, housing_bolt_points, ring_pin_hole_dia, ring_pin_points
from lib.cycloidal.housing import chamfer_outer_silhouette, reveal_window_cutter
from lib.geom import cylinder, single_solid, through
from lib.params import NUDGE

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    h, sh, b, tol, stack = cfg.housing, cfg.shell, cfg.bearings, cfg.tolerances, cfg.stack_up
    t = stack.motor_plate_wall + stack.motor_plate_inner_wall      # 9: the port plate's place
    ring = cylinder(h.od / 2.0, t) - through(sh.ring_bore_dia / 2.0, t)
    for xy in ring_pin_points(cfg):
        ring = ring - through(ring_pin_hole_dia(cfg) / 2.0, t, xy)
    for xy in housing_bolt_points(cfg):
        ring = ring - through((h.bolt_dia + tol.bolt_clearance_add) / 2.0, t, xy)
        ring = ring - cylinder(h.bolt_counterbore_dia / 2.0, h.bolt_counterbore_depth + NUDGE, xy, z0=-NUDGE)
    ring = ring - reveal_window_cutter(cfg, t)
    z0 = -(b.out_width + sh.skirt_lip)                              # -12: the skirt's end
    skirt = cylinder(sh.skirt_od / 2.0, -z0 + NUDGE, z0=z0)
    skirt = skirt - cylinder(h.output_bearing_seat_dia / 2.0, b.out_width + NUDGE, z0=-b.out_width)
    skirt = skirt - cylinder(h.lip_bore_dia / 2.0, sh.skirt_lip + 2 * NUDGE, z0=z0 - NUDGE)
    return chamfer_outer_silhouette(single_solid(ring + skirt), cfg, external_z=0.0)


@step
def cycloidal_shell_ring():
    """Return the ring at its LOCAL origin (= its stack position: the motor-plate face at z=0, the skirt in -Z)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    cycloidal_shell_ring()   # build: writes the sibling cycloidal_shell_ring.step
