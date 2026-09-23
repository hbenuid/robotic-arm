"""forearm_roll_shaft - the forearm roll drive's ROTOR - the hollow roll shaft: two Ø40.3 journals (bearing 1 goes on from the elbow end up to the middle shoulder, bearing 2 from the wrist end), the Ø44 core through the retainer, the integral 90T GT2 ring with two flanges, the hard-stop pin boss, the Ø60 flange with 4x M3 (nuts captive from its elbow face) and the 2 mm spigot into the forearm wall's recess, the Ø28 cable bore end to end.

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/forearm/roll.py build_shaft(cfg), every number
lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py
places it at 0); its reference is its accepted build, reference/native/forearm_roll_shaft.step (tools/reference/import_native.py).
PETG. In the arm: x1, inside forearm_roll_drive#1 - forearm_link (it IS the forearm's elbow end).
"""
import pathlib

from cadgen import step
from lib.datum import IDENTITY
from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_shaft

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/native/<NAME>.step - the accepted build
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: ForearmConfig = DEFAULT):
    return build_shaft(cfg)


@step
def forearm_roll_shaft():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_shaft()   # build: writes the sibling forearm_roll_shaft.step (preview: ./cadtool show parts/joints/forearm_roll_shaft.py)
