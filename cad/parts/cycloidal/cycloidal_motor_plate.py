"""cycloidal_motor_plate - the plate the NEMA 17 bolts to (z 0..9): the port's housing half, or the turning shell's
held carrier end with its sleeve back over the motor (to z -22).

Ported from cycloidal_drive@2f1f67d src/motor_plate.py (build_motor_plate()); reference/
cycloidal_motor_plate.step is that builder's export (kind "designed"). PETG.

Both: outer face z=0 (the motor's mounting face): pilot recess, 4x M3 clearance holes (heads pocketed from the inner
face), 15 mm shaft pass-through; the inner face z=9 faces the discs.
The port (LEGACY_CONFIG, `shell` None): a housing half - 21 ring-pin through-holes on the ring-pin circle (pins are
inserted one at a time from this face); the M4 through-holes (bolt_count) with counterbores on the bolt circle; the
shared pillar / reveal-window silhouette; outer silhouette chamfered. The inner face seats on the ring gear body.
The turning shell (DEFAULT_CONFIG, ShellParams): held, not part of the housing - a plate_dia plate inside the shell
ring's pin ring (parts/cycloidal/cycloidal_shell_ring, which takes the ring pins and the housing bolts), its outer face
cut back plate_relief_depth from plate_relief_dia out, over the motor-end 6814's turning outer race (its inner race
bears on the face inside), and a sleeve back over the motor: sleeve_bore_dia (the motor's and the board's corners),
sleeve_od, and the hub's grip (OutputHubParams.od) over the last out_width - the 6814 slides on from the sleeve's end
and presses on there. The sleeve runs through the j1_coupler yoke's motor-side leg, which clamps it, to the leg's outer
face (lib/cycloidal/layout.py sleeve_end: -22).

Diverged from the port: build(LEGACY_CONFIG) reproduces the export (REFERENCE_BUILD - tests/cycloidal/test_port.py,
tests/test_reference_match.py); the model builds DEFAULT_CONFIG (the turning shell's carrier plate).
"""
import pathlib

from cadgen import step

from lib.cycloidal import (
    DEFAULT_CONFIG,
    LEGACY_CONFIG,
    DriveConfig,
    housing_bolt_points,
    motor_bolt_counterbore_depth,
    motor_bolt_points,
    ring_pin_hole_dia,
    ring_pin_points,
    sleeve_end,
)
from lib.cycloidal.housing import chamfer_outer_silhouette, reveal_window_cutter
from lib.datum import IDENTITY
from lib.geom import cylinder, single_solid, through
from lib.params import NUDGE

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def _motor_face(plate, cfg: DriveConfig, t: float):
    """The motor's interface through a plate `t` thick: pilot recess, shaft pass-through, the M3 holes and head pockets."""
    h, m, tol = cfg.housing, cfg.motor, cfg.tolerances
    plate = plate - cylinder((m.pilot_dia + 2 * tol.mating_surface_add) / 2.0, m.pilot_height + NUDGE, z0=-NUDGE)
    plate = plate - through(h.motor_plate_shaft_bore / 2.0, t)
    # 4x M3 clearance + head pockets from the inner face (heads flush with z=9)
    m3_r = (m.bolt_dia + tol.bolt_clearance_add) / 2.0
    cb_r = (m.motor_bolt_head_dia + tol.bolt_clearance_add) / 2.0
    cb_d = motor_bolt_counterbore_depth(cfg)
    for xy in motor_bolt_points(cfg):
        plate = plate - through(m3_r, t, xy)
        plate = plate - cylinder(cb_r, cb_d + NUDGE, xy, z0=t - cb_d)
    return plate


def _carrier(cfg: DriveConfig):
    """The turning shell's held plate and its sleeve (ShellParams)."""
    sh, b, stack = cfg.shell, cfg.bearings, cfg.stack_up
    t = stack.motor_plate_wall + stack.motor_plate_inner_wall      # 9
    plate = _motor_face(cylinder(sh.plate_dia / 2.0, t), cfg, t)
    relief = cylinder(sh.plate_dia / 2.0 + 1.0, sh.plate_relief_depth + NUDGE, z0=-NUDGE)
    plate = plate - (relief - cylinder(sh.plate_relief_dia / 2.0, sh.plate_relief_depth + 3 * NUDGE, z0=-2 * NUDGE))
    back = sleeve_end(cfg)
    sleeve = cylinder(cfg.output_hub.od / 2.0, b.out_width + NUDGE, z0=-b.out_width)                    # the 6814's seat
    sleeve = sleeve + cylinder(sh.sleeve_od / 2.0, -back - b.out_width + NUDGE, z0=back)                # the rest
    sleeve = sleeve - cylinder(sh.sleeve_bore_dia / 2.0, -back + 2 * NUDGE, z0=back - NUDGE)
    return single_solid(plate + sleeve)


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    if cfg.shell is not None:
        return _carrier(cfg)
    h, stack = cfg.housing, cfg.stack_up
    t = stack.motor_plate_wall + stack.motor_plate_inner_wall      # 9
    result = _motor_face(cylinder(h.od / 2.0, t), cfg, t)
    # 21 ring-pin through-holes (clearance fit; the ring gear body retains the pins)
    pin_r = ring_pin_hole_dia(cfg) / 2.0
    for xy in ring_pin_points(cfg):
        result = result - through(pin_r, t, xy)
    # the M4 housing bolts' through-holes + counterbores on the outer face
    m4_r = (h.bolt_dia + cfg.tolerances.bolt_clearance_add) / 2.0
    for xy in housing_bolt_points(cfg):
        result = result - through(m4_r, t, xy)
        result = result - cylinder(h.bolt_counterbore_dia / 2.0, h.bolt_counterbore_depth + NUDGE, xy, z0=-NUDGE)
    # shared outer profile, then the bevel (outer face z=0 is external, inner face z=9 mates)
    result = result - reveal_window_cutter(cfg, t)
    return chamfer_outer_silhouette(result, cfg, external_z=0.0)


def REFERENCE_BUILD():
    """The CadQuery port (LEGACY_CONFIG) - what reference/cycloidal/cycloidal_motor_plate.step holds (tests/cycloidal/test_port.py)."""
    return build(LEGACY_CONFIG)


@step
def cycloidal_motor_plate():
    """Return the plate at its LOCAL origin (= its stack position: outer face at z=0)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    cycloidal_motor_plate()   # build: writes the sibling cycloidal_motor_plate.step
