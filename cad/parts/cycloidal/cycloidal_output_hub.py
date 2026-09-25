"""cycloidal_output_hub - passes through the two 6814 inner races, carries the output pins and
the arm-mount face (local z 0..28; sits at stack z 37..65, 5 mm proud of the housing).

Ported from cycloidal_drive@2f1f67d src/output_hub.py (build_output_hub()); reference/
cycloidal_output_hub.step is that builder's export (kind "designed"). PETG.

Inner face z=0: 625 bearing pocket, 6 mm shaft clearance bore over the bearing-grip zone, 4 blind
4.20 mm output-pin holes (60 mm circle, 1 mm ceiling), 4 captive M4 nut pockets. Through it: 4x
M4 arm-mount clearance holes on the 50 mm circle at 45 deg from the pins. Output face z=28: the
36 mm lightening recess. Print output-face-down; drop the 4 nuts in before pressing the hub
through the 6814s.
"""
import pathlib

from cadgen import step

from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, arm_mount_angles, arm_mount_points, hub_height, output_pin_points
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
    hub, d, b, tol, stack, h = cfg.output_hub, cfg.disc, cfg.bearings, cfg.tolerances, cfg.stack_up, cfg.housing
    grip = stack.output_bearing_total       # 20: the section inside the 2x 6814 inner races
    height = hub_height(cfg)                # 28

    result = cylinder(hub.od / 2.0, height)
    result = result - cylinder(hub.shaft_clearance_bore / 2.0, grip + NUDGE, z0=-NUDGE)                 # shaft clearance, grip zone only
    result = result - cylinder((b.inp_od + tol.bearing_seat_bore_add) / 2.0, b.inp_width + NUDGE, z0=-NUDGE)   # 625 pocket
    pin_r = (d.output_pin_dia - tol.ring_pin_press_sub) / 2.0                                          # 4.20 / 2
    pin_depth = grip - hub.output_hub_pin_ceiling                                                       # 19
    for xy in output_pin_points(cfg):
        result = result - cylinder(pin_r, pin_depth + NUDGE, xy, z0=-NUDGE)
    arm_r = (h.bolt_dia + tol.bolt_clearance_add) / 2.0
    for angle, xy in zip(arm_mount_angles(cfg), arm_mount_points(cfg), strict=True):
        result = result - through(arm_r, height, xy)
        result = result - hex_pocket(cfg, xy, angle, h.bolt_nut_depth + NUDGE, z0=-NUDGE)
    if hub.arm_mount_pocket_dia > 0.0:
        z0 = grip + hub.arm_mount_pocket_floor                                                          # 21
        result = result - cylinder(hub.arm_mount_pocket_dia / 2.0, height - z0 + NUDGE, z0=z0)
    return single_solid(result)


@step
def cycloidal_output_hub():
    """Return the hub at its LOCAL origin (inner face at z=0; the assembly lifts it to z=37)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    cycloidal_output_hub()   # build: writes the sibling cycloidal_output_hub.step
