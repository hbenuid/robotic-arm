"""gripper_cage - the gripper's bracket, designed here (no reference): a back plate on the MG996R's back that bolts onto
gripper_j3_connector's plate (4x M3 x 8 from the connector's flange side, into nuts slid into slots from the plate's
edges), two side walls along the servo's case, and an ear on each pair of the servo's tabs over the base's posts - the
tab screws (M3) clamp ear, tab and post together into the posts' nut traps, put in through the back plate's access
holes; the servo's cable runs in a slot in the back plate to the axis, its plug through the hole there and on through
the connector's bore. Parametric build123d (lib/gripper/body.py build_cage(), every number lib/gripper/params.py
CageParams), in the gripper base's frame (lib/gripper/params.py BaseParams: the plate's bottom on z 0, the servo toward
+z).

Assembly: the servo into the cage (its back on the back plate), the base on its tabs, the tab screws through the
access holes; then the cage onto the connector's plate (already on the wrist roll pulley), its screws from the flange
side. docs/open_issues.md: a design not yet printed.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.gripper import DEFAULT
from lib.gripper.body import build_cage

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # designed here: no reference (lib/reference.py NO_REFERENCE)
CONVERTED = True
LOCAL_FROM_REF = IDENTITY


@step(out="gripper_cage.step")
def gripper_cage():
    """The cage in the gripper base's frame; the assembly owns placement."""
    part = build_cage(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    gripper_cage()   # build: writes the sibling gripper_cage.step
