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
and presses on there. The sleeve runs through the j1_coupler yoke's motor-side leg (parts/base/j1_motor_leg, a ring round it) to
the leg's outer face (lib/cycloidal/layout.py sleeve_end: -22), bolted to it: the leg's radial M4s go into nuts pressed
into lugs on the bore at the motor's flats (sleeve_bolt_deg; none down, where the leg's post stands) - sleeve_lug_len
at sleeve_lug_r from the sleeve's end, then 45 deg out to the bore, so the plate prints on its inner face without
support -, each nut's hex pocket from the lug's face (flats across the axis), the screw's hole on out through the wall.

Diverged from the port: build(LEGACY_CONFIG) reproduces the export (REFERENCE_BUILD - tests/cycloidal/test_port.py,
tests/test_reference_match.py); the model builds DEFAULT_CONFIG (the turning shell's carrier plate).
"""
import pathlib

from cadgen import build123d as bd
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
    sleeve_bolt_z,
    sleeve_end,
)
from lib.cycloidal.housing import chamfer_outer_silhouette, reveal_window_cutter
from lib.datum import IDENTITY
from lib.geom import align_min, cylinder, hex_prism, single_solid, through
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


def _radial(r: float, r0: float, length: float, z: float):
    """A cylinder of radius r along +Y from r0, `length` long, its axis at height z."""
    return bd.Pos(0.0, r0, z) * bd.Rot(-90.0, 0.0, 0.0) * bd.Cylinder(r, length, align=align_min())


def _sleeve_lugs(sleeve, cfg: DriveConfig):
    """The lugs on the sleeve's bore for the yoke leg's radial M4s (ShellParams.sleeve_bolt_deg), each built at +Y and
    turned there: sleeve_lug_len at sleeve_lug_r from the sleeve's end, then 45 deg out into the wall; the press-fit
    nut's hex pocket from the lug's face out to the nut's outer face (flats across the axis); the screw's hole from the
    pocket's floor out through the wall."""
    sh, h = cfg.shell, cfg.housing
    back, z = sleeve_end(cfg), sleeve_bolt_z(cfg)
    wall = sh.sleeve_bore_dia / 2.0 + 0.3                                # a hair into the wall, so the two fuse
    rise = wall - sh.sleeve_lug_r
    outline = [(sh.sleeve_lug_r, back), (wall, back), (wall, back + sh.sleeve_lug_len + rise), (sh.sleeve_lug_r, back + sh.sleeve_lug_len)]
    lug = bd.extrude(bd.Plane.YZ.offset(-sh.sleeve_lug_w / 2.0) * bd.Polygon(*outline, align=None),
                     amount=sh.sleeve_lug_w, dir=(1.0, 0.0, 0.0))
    floor = sh.sleeve_nut_r + h.bolt_nut_thickness
    # a hex prism along +Y, a corner along X (hex_prism's first vertex on +X): its flats across the drive's axis
    pocket = bd.Pos(0.0, sh.sleeve_lug_r - NUDGE, z) * bd.Rot(-90.0, 0.0, 0.0) * hex_prism(sh.sleeve_nut_af, 0.0, floor - sh.sleeve_lug_r + NUDGE)
    hole = _radial((h.bolt_dia + cfg.tolerances.bolt_clearance_add) / 2.0, floor - NUDGE, sh.sleeve_od / 2.0 - floor + 1.0, z)
    for deg in sh.sleeve_bolt_deg:
        turn = bd.Rot(0.0, 0.0, deg - 90.0)
        sleeve = sleeve + turn * lug
        sleeve = sleeve - turn * pocket - turn * hole
    return sleeve


def _carrier(cfg: DriveConfig):
    """The turning shell's held plate and its sleeve (ShellParams), the sleeve's lugs for the yoke leg's screws."""
    sh, b, stack = cfg.shell, cfg.bearings, cfg.stack_up
    t = stack.motor_plate_wall + stack.motor_plate_inner_wall      # 9
    plate = _motor_face(cylinder(sh.plate_dia / 2.0, t), cfg, t)
    relief = cylinder(sh.plate_dia / 2.0 + 1.0, sh.plate_relief_depth + NUDGE, z0=-NUDGE)
    plate = plate - (relief - cylinder(sh.plate_relief_dia / 2.0, sh.plate_relief_depth + 3 * NUDGE, z0=-2 * NUDGE))
    back = sleeve_end(cfg)
    sleeve = cylinder(cfg.output_hub.od / 2.0, b.out_width + NUDGE, z0=-b.out_width)                    # the 6814's seat
    sleeve = sleeve + cylinder(sh.sleeve_od / 2.0, -back - b.out_width + NUDGE, z0=back)                # the rest
    sleeve = sleeve - cylinder(sh.sleeve_bore_dia / 2.0, -back + 2 * NUDGE, z0=back - NUDGE)
    return single_solid(plate + _sleeve_lugs(sleeve, cfg))


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
