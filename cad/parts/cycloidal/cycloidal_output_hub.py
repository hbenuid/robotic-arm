"""cycloidal_output_hub - the carrier: through the 6814 inner races, carries the output pins and the arm-mount face
(local z 0..28 at stack z 37; the turning shell's 0..23 at 39).

Ported from cycloidal_drive@2f1f67d src/output_hub.py (build_output_hub()); reference/
cycloidal_output_hub.step is that builder's export (kind "designed"). PETG.

Inner face z=0: 625 bearing pocket, 6 mm shaft clearance bore over the bearing-grip zone, 4 blind
4.20 mm output-pin holes (60 mm circle, 1 mm ceiling), 4 captive M4 nut pockets. Through it: 4x
M4 arm-mount clearance holes on the 50 mm circle at 45 deg from the pins. Output face z=28: the
36 mm lightening recess. Print output-face-down; drop the 4 nuts in before pressing the hub
through the 6814s.
The turning shell (ShellParams) holds the hub - the carrier the output pins hold the discs from turning with, so the
housing turns - and makes it the motor plate's mirror: a plate_dia flange (lib/cycloidal/layout.py hub_flange, 9) in
the shell's pin ring, the pin holes and the 625 in its inner face, its outer face cut back over the hub-end 6814's
turning outer race, whose inner race the grip carries and the flange stops (the shell is held both ways: this race,
the shell's lip on its outer race, and the motor-end 6814 mirrored). The grip runs past the shell's end
(proud_above_housing) onto the j1_coupler yoke's leg; the same 4 holes bolt it there, their nuts in hub_nut_depth
pockets from the flange's inner face.

Diverged from the port: build(LEGACY_CONFIG) reproduces the export (REFERENCE_BUILD - tests/cycloidal/test_port.py,
tests/test_reference_match.py); the model builds DEFAULT_CONFIG (the turning shell's hub).
"""
import pathlib

from cadgen import step

from lib.cycloidal import (
    DEFAULT_CONFIG,
    LEGACY_CONFIG,
    DriveConfig,
    arm_mount_angles,
    arm_mount_points,
    hub_flange,
    hub_height,
    output_pin_points,
    stack_positions,
)
from lib.cycloidal.housing import hex_pocket
from lib.datum import IDENTITY
from lib.geom import cylinder, single_solid, through
from lib.params import NUDGE

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    hub, d, b, tol, stack, h, sh = cfg.output_hub, cfg.disc, cfg.bearings, cfg.tolerances, cfg.stack_up, cfg.housing, cfg.shell
    flange = hub_flange(cfg)                # 0 (port), 9: the motor plate's mirror
    height = hub_height(cfg)                # 28, 23
    z_hub = stack_positions(cfg)["z_hub"]

    result = cylinder(hub.od / 2.0, height)
    if sh is not None:   # the flange in the shell's pin ring, its face on the 6814 cut back over the outer race
        result = result + cylinder(sh.plate_dia / 2.0, flange)
        relief = cylinder(sh.plate_dia / 2.0 + 1.0, sh.plate_relief_depth + NUDGE, z0=flange - sh.plate_relief_depth)
        result = result - (relief - cylinder(sh.plate_relief_dia / 2.0, sh.plate_relief_depth + 3 * NUDGE,
                                             z0=flange - sh.plate_relief_depth - NUDGE))
    clear = stack.output_bearing_total if sh is None else flange   # the port's grip zone (inside its 2x 6814), the flange
    result = result - cylinder(hub.shaft_clearance_bore / 2.0, clear + NUDGE, z0=-NUDGE)                # shaft clearance
    result = result - cylinder((b.inp_od + tol.bearing_seat_bore_add) / 2.0, b.inp_width + NUDGE, z0=-NUDGE)   # 625 pocket
    pin_r = (d.output_pin_dia - tol.ring_pin_press_sub) / 2.0                                          # 4.20 / 2
    pin_depth = stack.z_bearing_top - hub.output_hub_pin_ceiling - z_hub                               # 19 (port), 18
    for xy in output_pin_points(cfg):
        result = result - cylinder(pin_r, pin_depth + NUDGE, xy, z0=-NUDGE)
    arm_r = (h.bolt_dia + tol.bolt_clearance_add) / 2.0
    nut_depth = h.bolt_nut_depth if sh is None else sh.hub_nut_depth
    for angle, xy in zip(arm_mount_angles(cfg), arm_mount_points(cfg), strict=True):
        result = result - through(arm_r, height, xy)
        result = result - hex_pocket(cfg, xy, angle, nut_depth + NUDGE, z0=-NUDGE)
    if hub.arm_mount_pocket_dia > 0.0:
        z0 = pin_depth + hub.output_hub_pin_ceiling + hub.arm_mount_pocket_floor                        # 21, 20
        result = result - cylinder(hub.arm_mount_pocket_dia / 2.0, height - z0 + NUDGE, z0=z0)
    return single_solid(result)


def REFERENCE_BUILD():
    """The CadQuery port (LEGACY_CONFIG) - what reference/cycloidal/cycloidal_output_hub.step holds (tests/cycloidal/test_port.py)."""
    return build(LEGACY_CONFIG)


@step
def cycloidal_output_hub():
    """Return the hub at its LOCAL origin (inner face at z=0; the assembly lifts it to stack_positions z_hub)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    cycloidal_output_hub()   # build: writes the sibling cycloidal_output_hub.step
