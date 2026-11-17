"""wrist_roll_connector_screws - purchased (COTS) part: the 4 M3 socket head cap screws (ISO 4762) that hold
gripper_j3_connector on the wrist roll pulley's face.

Through the connector's flange (ConnectorParams: its 4 holes on bolt_circle_dia at bolt_deg, its notches over them) into
the nuts in the pulley face's pockets (wrist_roll_connector_nuts): the heads on the flange's plate side, between it and
the connector's plate - a key reaches each one along Y past the plate's flats -, the tips at the end of their holes in
the pulley (z_ring_end). Bolt the connector on after the rotor clamp is tight, before the gripper's cage.

In the arm: x1, in wrist_roll_drive (its row in assemblies/wrist_roll_drive.py, the rotor - wrist_roll_link, with the
gripper). Length (under the head): ConnectorParams.bolt_len.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/wrist/test_roll_screws.py locks its numbers. Frame: axis on Z, the
heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the connector's bolt circle as the module has
them (lib/wrist/layout.py connector_bolt_points_module() mirrored in y: the module's row turns the pattern over about X,
the heads up R').
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import WRIST_ROLL_CONNECTOR_SCREWS_MASS_G
from lib.wrist import DEFAULT as WRIST
from lib.wrist.layout import connector_bolt_points_module

SCREW = WRIST.connector.screw
LENGTH = WRIST.connector.bolt_len
POINTS = tuple((x, -y) for x, y in connector_bolt_points_module(WRIST))   # turned over by the row: module (x, y) = (x, -y)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_CONNECTOR_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("gripper_j3_connector onto the wrist roll pulley's face: heads between the connector's flange and its "
                 "plate (a key past the plate's flats), into the nuts in the face; before the gripper's cage")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step(out="wrist_roll_connector_screws.step")
def wrist_roll_connector_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_connector_screws()   # build: writes the sibling wrist_roll_connector_screws.step
