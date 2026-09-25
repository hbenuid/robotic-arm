"""forearm_roll_retainer - the forearm roll drive's END CAP on the block's front face: bearing 2's Ø52.15 seat and the lip its
outer race stops on, the block's rounded outline (66 x 72), 4x M3 at its corners into the block's front face, the hard-stop post on
its outer face at -X that the shaft's lug meets at +/- FOREARM_ROLL_LIMIT_DEG.

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/forearm/roll.py build_retainer(cfg), every number
lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py
places it at 0); its reference is its accepted build, reference/native/forearm_roll_retainer.step (tools/reference/import_native.py).
PETG. In the arm: x1, inside forearm_roll_drive#1 - elbow_link (bolted to the block's front face).
"""
import pathlib

from cadgen import step
from lib.datum import IDENTITY
from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_retainer

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/native/<NAME>.step - the accepted build
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: ForearmConfig = DEFAULT):
    return build_retainer(cfg)


@step
def forearm_roll_retainer():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_retainer()   # build: writes the sibling forearm_roll_retainer.step
