"""forearm_roll_mount_nuts - purchased (COTS) part: the roll motor mount's 4 M3 hex nuts (ISO 4032), in the elbow block.

The mount's countersunk screws (forearm_roll_mount_screws) run into them, in the block's two nut channels
(forearm_roll_block).

In the arm (assemblies/forearm_roll_drive.py, a row of the module): each nut up against its channel's ceiling
(lib/forearm/layout.py stack_positions()["y_mount_nut"], lib/forearm/params.py RollDriveParams.mount_nut_roof under
the step's floor), a flat on either wall of its channel. They slide in along the channels from the block's rear face,
two per channel - the front one first, pushed to the channel's end, which stops it under its screw
(docs/forearm_roll.md §4).

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like elbow_pulley_nuts - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and its reference is that envelope
(reference/native/forearm_roll_mount_nuts.step, tools/reference/import_native.py). Frame: axis on Z, the nuts' bearing
faces on z=0, standing in +Z; the four at lib/forearm/layout.py mount_bolt_points() as (x, y) = the module's (x, z),
each a flat toward +/-X - the module turns +Z down its -Y.
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.forearm import mount_bolt_points
from lib.params import M3_NUT, ROLL_MOUNT_NUTS_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_MOUNT_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M3_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(mount_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "slid into the elbow block's two channels from its rear face, the front nut of each first"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, corners along +/-Y (the channel's length) so a flat faces each wall: the geometry until a
    vendor model exists."""
    return pattern(hex_nut(M3_NUT, math.pi / 2.0, xy) for xy in mount_bolt_points())


@step
def forearm_roll_mount_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    forearm_roll_mount_nuts()   # build: writes the sibling forearm_roll_mount_nuts.step
