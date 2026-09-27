"""forearm_roll_mount_nuts - purchased (COTS) part: the roll motor mount's 4 M3 hex nuts (ISO 4032), in the elbow block.

The mount's countersunk screws (forearm_roll_mount_screws) run into them, in the block's hex pockets under its top
wall (forearm_roll_block).

In the arm (assemblies/forearm_roll_drive.py, a row of the module): each nut up against its pocket's ceiling
(lib/forearm/layout.py stack_positions()["y_mount_nut"]: the screw's tip RollDriveParams.mount_tip_past past it), a
flat toward +/-X as the pocket holds it. Each pocket opens into the core bore: the nut goes in from inside the bore,
pressed up to its seat (mount_nut_pocket_af: a press, so it stays when its screw is out), before the roll shaft goes
in (docs/forearm_roll.md §4).

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
PURCHASE_NOTE = "pressed into the elbow block's hex pockets from inside its core bore, before the roll shaft"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, corners along +/-Y (the module's Z) so a flat faces +/-X as the pockets hold them: the
    geometry until a vendor model exists."""
    return pattern(hex_nut(M3_NUT, math.pi / 2.0, xy) for xy in mount_bolt_points())


@step
def forearm_roll_mount_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    forearm_roll_mount_nuts()   # build: writes the sibling forearm_roll_mount_nuts.step
