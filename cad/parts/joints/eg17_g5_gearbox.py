"""eg17_g5_gearbox - purchased (COTS) part: the elbow motor's 5:1 planetary gearbox (StepperOnline EG17-G5).

Bought for the elbow_pitch motor (nema17_40mm#2), so the elbow belt no longer carries the joint's whole reduction
(lib/params.py ELBOW_RATIO: the gearbox's ratio times the belt's). In the arm (lib/mounts.py) its output face sits on
the web across j1_link's motor hole where the motor's face would - its boss in the web's counterbore, 4x M4 up
through the web into its face (parts/joints/elbow_gearbox_screws) -, the motor on its input end (its pilot in the
bore, its shaft in the clamp, held by parts/joints/elbow_motor_screws through the input plate), the elbow's 20T on
its output shaft (parts/joints/gt2_pulley_20t_8mm).

No catalog model, no export and NO reference file: a purchased part with no reference (lib/reference.py
NO_REFERENCE), its envelope the geometry - built from the datasheet drawing (lib/motors.py GearboxParams) and locked by
tests/upper_arm/test_eg17_g5_gearbox.py. Output end first: a round output flange and a round band, both as wide as the
square (the drawing's front view shows the input end's corner holes past them); the square input end, its corners
clipped and cut back to the input plate (the channels the motor's screws go in by, onto the plate's holes on the NEMA
17 square), the Ø22 bore in it round the shaft clamp's collar, the clamp's two access holes; on the output face the
4x M4 holes and the boss, the shaft's shoulder, the Ø8 shaft with its keyway (the key left out: the 20T's set screw
in it) and the M3 hole in its end. Simplified: the corner channels as square notches (the drawing's end in a ramp);
left out: the 4 hex screws at each end, the access holes' plugs (0.6 proud).

Frame: the motor's (lib/mounts.py) - the output face z=0, the body -Z, the output shaft +Z; the M4s on the X and Y
axes, the keyway on +X, the access holes +Y in the +X face and -Y in the -X face.
"""
import pathlib

import build123d as bd
from cadgen import step

from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.geom import box, cylinder, single_solid, through
from lib.params import EG17_G5, EG17_G5_MASS_G, NUDGE

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = EG17_G5_MASS_G   # [DATASHEET] the listing's weight
PURCHASE_SPEC = f"NEMA 17 planetary gearbox {EG17_G5.ratio:g}:1, Ø8 keyed output shaft (StepperOnline EG17-G5)"
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = ("on the elbow motor; it ships with the 4 M3 screws that hold the motor on it (parts/joints/elbow_motor_screws) "
                 "and its key, which stays out (the 20T's set screw in the keyway)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY
G = EG17_G5


def _envelope():
    """The gearbox from its datasheet drawing (GearboxParams), one solid."""
    g, half = G, G.square / 2.0
    z_square = -(g.flange_t + g.band_t)                       # the square input end, from here to -length
    body = cylinder(half, g.flange_t + g.band_t, z0=z_square)
    square = box((-half, half), (-half, half), (-g.length, z_square))
    body = body + bd.chamfer(square.edges().filter_by(bd.Axis.Z), g.corner_ch)
    plate = -g.length + g.plate_t
    for sx in (-1, 1):                                         # the corner channels, the band's end down to the plate
        for sy in (-1, 1):
            xs = (g.channel, half + 1.0) if sx > 0 else (-half - 1.0, -g.channel)
            ys = (g.channel, half + 1.0) if sy > 0 else (-half - 1.0, -g.channel)
            body = body - box(xs, ys, (plate, z_square))
    body = body - cylinder(g.bore_dia / 2.0, g.bore_depth + NUDGE, z0=-g.length - NUDGE)
    collar = cylinder(g.collar_dia / 2.0, g.bore_depth - g.collar_from, z0=-g.length + g.collar_from)
    body = body + collar - through(g.input_dia / 2.0, g.bore_depth - g.collar_from, z0=-g.length + g.collar_from)
    for sx in (-1, 1):
        for sy in (-1, 1):
            body = body - through(g.plate_hole / 2.0, g.plate_t, (sx * g.plate_sp / 2.0, sy * g.plate_sp / 2.0), -g.length)
    r = g.bolt_pcd / 2.0
    for xy in ((r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)):
        body = body - cylinder(g.bolt_hole_dia / 2.0, g.bolt_depth + NUDGE, xy, -g.bolt_depth)
    z_access = -g.length + g.access_from_input
    for sx in (-1, 1):                                         # the access holes, in from each X face to the bore
        hole = cylinder(g.access_dia / 2.0, half - g.bore_dia / 2.0 + 2.0, z0=g.bore_dia / 2.0 - 1.0)
        hole = hole + cylinder(g.access_cb_dia / 2.0, g.access_cb_depth + 1.0, z0=half - g.access_cb_depth)
        hole = hole.moved(bd.Location((0.0, 0.0, 0.0), (0.0, 90.0 * sx, 0.0)))   # its +Z onto +/-X, out from the axis
        body = body - hole.moved(bd.Location((0.0, sx * g.access_off, z_access)))
    shaft = (cylinder(g.boss_dia / 2.0, g.boss_t) + cylinder(g.shoulder_dia / 2.0, g.shoulder_t, z0=g.boss_t)
             + cylinder(g.shaft_dia / 2.0, g.shaft_end - g.boss_t, z0=g.boss_t))
    # the keyway on +X: a round-ended slot key_w wide, key_l long from key_from_end short of the end, keyway_depth() deep
    w, z1 = g.key_w / 2.0, g.shaft_end - g.key_from_end
    x0 = g.shaft_dia / 2.0 - g.keyway_depth()
    slot = box((x0, g.shaft_dia), (-w, w), (z1 - g.key_l + w, z1 - w))
    for z in (z1 - g.key_l + w, z1 - w):
        slot = slot + cylinder(w, g.shaft_dia, z0=0.0).moved(bd.Location((x0, 0.0, z), (0.0, 90.0, 0.0)))
    shaft = shaft - slot - cylinder(g.end_tap_dia / 2.0, g.end_tap_depth + NUDGE, z0=g.shaft_end - g.end_tap_depth)
    return single_solid(body + shaft)


@step(out="eg17_g5_gearbox.step")
def eg17_g5_gearbox():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    eg17_g5_gearbox()   # build: writes the sibling eg17_g5_gearbox.step
