"""forearm_roll_block - the forearm roll drive's STATOR - the roll frame (the elbow block), which is also the elbow's output
flange, ONE body round the roll motor: seen along the elbow axis round about it (r 45, concentric with j1_link's round end)
and tangent up to the TOWER round the roll axis (ELBOW_ROLL_OFFSET above the elbow axis), from its underside (3.0 over
j1_link's flat top, repeating the SolidWorks j3_coupler's lip, Ø62 boss, Ø40 journal and Ø30 stub about the elbow axis down
into j1_link's recess and bore - the elbow 90T pulley bolts up through the stub into 4x M4 nuts in hex channels that open
up into the motor's pocket; j3_coupler#1 is retired) to its open +N face. The 40 mm kit motor sits on the elbow axis in
the POCKET open on that face, its front wall the PLATE (the tension slots and the pilot slot), whose face is the frame's one
front face; the tower carries the 6806 pair back to back on a lip - bearing 1's seat open into the shaft's BAY behind it
(open on +N, the stop post on the tower's rear face), bearing 2's seat under the CUP, a round boss on the front face the
pulley's ring turns sunk in.

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/forearm/roll.py build_block(cfg), every number
lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py
places it at 0); its reference is its accepted build, reference/native/forearm_roll_block.step (tools/reference/import_native.py).
PETG. In the arm: x1, inside forearm_roll_drive#1 - elbow_link (the stator rides with the elbow pulley; it IS the elbow coupler).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_block

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/native/<NAME>.step - the accepted build
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: ForearmConfig = DEFAULT):
    return build_block(cfg)


@step
def forearm_roll_block():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_block()   # build: writes the sibling forearm_roll_block.step
