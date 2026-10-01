"""cycloidal_shell_ring - the turning shell's motor end (built at its stack position: z -13..9), PETG.

Designed here (docs/cycloidal_drive.md §5.5), no reference: tests/cycloidal/test_shell_ring.py holds its numbers.
The turning shell (lib/cycloidal/params.py ShellParams) takes the port's motor plate apart: the plate that holds the
motor stays with the carrier (parts/cycloidal/cycloidal_motor_plate), and this ring - turning with the body round the
discs it is bolted to (lib/cycloidal/housing.py build_shell_body, printed with j1_link) - is the shell's motor end, the
hub end's mirror: the housing's od with its pillars and windows from end to end, and inside, from the outside in, an
end_lip (HousingParams.lip_bore_dia) - the motor-end 6814's outer race's stop -, the 6814's seat
(HousingParams.output_bearing_seat_dia, out_width deep: the race pressed in from z=0) and a ring_bore_dia pin ring round
the held motor plate (z 0..9) with the 21 ring pins' blind holes from its face on the body (z=9: the pins stand in
the body's pin ring and this ring closes over their other ends). The housing bolts' holes run through it, their heads'
counterbores in the end face (z=-13), which is chamfered like the port's plate; the face on the body sharp.
"""
import pathlib

from cadgen import step

from lib.cycloidal import (
    DEFAULT_CONFIG,
    DriveConfig,
    housing_bolt_points,
    ring_pin_hole_dia,
    ring_pin_points,
    shell_ends,
    stack_positions,
)
from lib.cycloidal.housing import PIN_END_CLEAR, chamfer_outer_silhouette, reveal_window_cutter
from lib.geom import cylinder, single_solid
from lib.params import NUDGE

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    h, sh, b, tol = cfg.housing, cfg.shell, cfg.bearings, cfg.tolerances
    t = cfg.stack_up.z_motor_plate_inner                            # 9: the motor plate's inner face, the body's
    z0 = shell_ends(cfg)[0]                                         # -13
    ring = cylinder(h.od / 2.0, t - z0, z0=z0)
    ring = ring - cylinder(h.lip_bore_dia / 2.0, sh.end_lip + 2 * NUDGE, z0=z0 - NUDGE)                 # the lip
    ring = ring - cylinder(h.output_bearing_seat_dia / 2.0, b.out_width + NUDGE, z0=-b.out_width)       # the 6814's seat
    ring = ring - cylinder(sh.ring_bore_dia / 2.0, t + NUDGE, z0=0.0)                                  # the pin ring
    pins_z = stack_positions(cfg)["z_ring_pins"] - PIN_END_CLEAR
    for xy in ring_pin_points(cfg):
        ring = ring - cylinder(ring_pin_hole_dia(cfg) / 2.0, t - pins_z + NUDGE, xy, z0=pins_z)
    for xy in housing_bolt_points(cfg):
        ring = ring - cylinder((h.bolt_dia + tol.bolt_clearance_add) / 2.0, t - z0 + 2 * NUDGE, xy, z0=z0 - NUDGE)
        ring = ring - cylinder(h.bolt_counterbore_dia / 2.0, h.bolt_counterbore_depth + NUDGE, xy, z0=z0 - NUDGE)
    ring = ring - reveal_window_cutter(cfg, t - z0, z_offset=z0)
    return chamfer_outer_silhouette(single_solid(ring), cfg, external_z=z0)


@step
def cycloidal_shell_ring():
    """Return the ring at its LOCAL origin (= its stack position: the motor plate's outer face at z=0, the lip in -Z)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    cycloidal_shell_ring()   # build: writes the sibling cycloidal_shell_ring.step
