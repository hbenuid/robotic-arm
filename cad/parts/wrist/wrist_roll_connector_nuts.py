"""wrist_roll_connector_nuts - purchased (COTS) part: the 4 M3 hex nuts (ISO 4032) in the wrist roll pulley's face, for
gripper_j3_connector's screws (wrist_roll_connector_screws).

Each sits in its pocket in the face (lib/wrist/layout.py face_nut_pocket: a slot from the face's rim inward past the nut's
inner corner, the nut's across flats + 2 nut_fit wide - its flats hold it from turning), its bearing face face_nut_depth
under the face. Push them in from the face's rim before the connector goes on.

In the arm: x1, in wrist_roll_drive (its row in assemblies/wrist_roll_drive.py, the rotor - wrist_roll_link).

No catalog model, no SolidWorks export and NO reference file: a pattern part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py hex_nut()) and tests/wrist/test_roll_screws.py locks its
numbers. Frame: axis on Z, each nut's far face on z=0 (its bearing face at z = h), a corner along its radius; the four on
the connector's bolt circle as the module has them (lib/wrist/layout.py connector_bolt_points_module(): the row is a
translation).
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import WRIST_ROLL_CONNECTOR_NUTS_MASS_G
from lib.wrist import DEFAULT as WRIST
from lib.wrist.layout import connector_bolt_points_module

NUT = WRIST.roll.nut
POINTS = tuple(connector_bolt_points_module(WRIST))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_CONNECTOR_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "in the wrist roll pulley's face, for the connector's screws: pushed into the pockets from the face's rim"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, a corner along each one's radius: the geometry until a vendor model exists."""
    return pattern(hex_nut(NUT, math.atan2(y, x), (x, y)) for x, y in POINTS)


@step(out="wrist_roll_connector_nuts.step")
def wrist_roll_connector_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_connector_nuts()   # build: writes the sibling wrist_roll_connector_nuts.step
