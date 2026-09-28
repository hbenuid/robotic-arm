"""base_motor_mount_nuts - purchased (COTS) part: the base motor mount's 4 M4 hex nuts (ISO 4032), in the base.

The mount's screws (base_motor_mount_screws) run into them, in the hex pockets of the base's two joint ribs (base).

In the arm (lib/mounts.py, hosted on base_motor_mount_screws#1): each nut home in its pocket, its outer face flush
with the rib's back face (lib/base/layout.py joint_stations()), a corner up (+/-Y) as the pocket holds it. The nuts
go in from the base's cavity, pressed (JointParams.nut_pocket_af: a press, so they stay when the mount is off).

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like elbow_pulley_nuts - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and its reference is that envelope
(reference/native/base_motor_mount_nuts.step, tools/reference/import_native.py). Frame: axis on Z, the nuts' bearing
faces on z=0, standing in +Z; the four at lib/base/layout.py joint_bolt_points() as (x, y) = the base's (z, y), each a
corner along +/-Y.
"""
import math
import pathlib

from cadgen import step

from lib.base import joint_bolt_points
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import BASE_MOUNT_NUTS_MASS_G, M4_NUT

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BASE_MOUNT_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(joint_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "pressed into the hex pockets of the base's joint ribs from inside the base, before the motor mount goes on"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, corners along +/-Y (the base's Y) as the pockets hold them: the geometry until a vendor
    model exists."""
    return pattern(hex_nut(M4_NUT, math.pi / 2.0, xy) for xy in joint_bolt_points())


@step
def base_motor_mount_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    base_motor_mount_nuts()   # build: writes the sibling base_motor_mount_nuts.step
