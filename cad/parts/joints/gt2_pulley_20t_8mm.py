"""gt2_pulley_20t_8mm - purchased (COTS) part: the elbow's 20T, bored for the gearbox's Ø8 output shaft.

A stock GT2 20T for the 6 mm belt with an 8 mm bore, on the elbow gearbox's output shaft (parts/joints/eg17_g5_gearbox)
where the motor's own 20T sat before the gearbox: J1_MOTOR_20T_HUB_Z out from the gearbox's face, its tooth band level
with the elbow 90T's (lib/mounts.py). Its set screws hold it on the shaft, one in the shaft's keyway - the key that
ships with the gearbox stays out.

No vendor model of its own and NO reference file: a purchased part with no reference (lib/reference.py NO_REFERENCE).
Its envelope is the 5 mm pulley's vendor geometry (parts/wrist/gt2_pulley_20t: vendor/gt2_pulley_20t.step, the same
20T) bored out to the gearbox's shaft (EG17_G5.shaft_dia), the set screws' tips cut back to the bore;
tests/upper_arm/test_gt2_pulley_20t_8mm.py locks its numbers. Frame: the 5 mm pulley's - the bore along +X, the hub's
face on x = 0, the tooth band toward +X, the set screws in the hub along +Y and +Z.
"""
import pathlib

import build123d as bd
from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.params import EG17_G5, NUDGE

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 7.0     # [ESTIMATE] the 5 mm pulley's estimate (8 g) less the aluminium the wider bore takes
PURCHASE_SPEC = "GT2 20T timing pulley, 8 mm bore, 6 mm belt"
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = ("on the elbow gearbox's Ø8 keyed shaft: leave the gearbox's key out and tighten one set screw into the "
                 "keyway; the other 20Ts stay 5 mm bore (motor shafts)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY
BORE = EG17_G5.shaft_dia
FIVE_MM = pathlib.Path(__file__).resolve().parents[2] / "vendor" / "gt2_pulley_20t.step"   # the 5 mm pulley's vendor file


def _envelope():
    """The 5 mm pulley's vendor solids (the body, its two set screws) less a Ø8 bore along +X."""
    bore = bd.Cylinder(BORE / 2.0, 30.0 + 2 * NUDGE, rotation=(0, 90, 0)).moved(bd.Location((15.0, 0.0, 0.0)))
    return pattern(s - bore for s in step.read(FIVE_MM).solids())


@step(out="gt2_pulley_20t_8mm.step")
def gt2_pulley_20t_8mm():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    gt2_pulley_20t_8mm()   # build: writes the sibling gt2_pulley_20t_8mm.step
