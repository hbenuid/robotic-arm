"""forearm_roll_shaft - the forearm roll drive's ROTOR - the hollow roll shaft that crosses the elbow axis: a Ø40.3 journal at each
end of its Ø44 core (bearing 1 in the block's rear seat, bearing 2 in the cap over the Ø38 neck), the integral flanged 90T GT2 ring
on the core 18..25 mm past the elbow axis, the hard-stop lug on the neck, the Ø39.7 end spigot the forearm's wall bolts onto (4x M3
self-tapping in its end wall, on Ø32), the Ø24 cable bore end to end (the cables leave through the block's rear end wall).

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
    forearm_roll_shaft()   # build: writes the sibling forearm_roll_shaft.step
