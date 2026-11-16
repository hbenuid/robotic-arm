"""j1_coupler_plate - the base_yaw coupler's bottom part, PETG: the plate under j1_coupler's cut, with the stub through the
upper base bearing, the recess over the thrust bearing and its rim, and the 120T's nuts.

Designed here, no reference: tests/yaw_coupler/test_j1_coupler.py holds its numbers. DEFAULT's coupler prints in two
parts (lib/yaw_coupler/params.py SplitParams): whole it would stand on its stub with the disc in the air (support
under all of it, the recess a 30 mm roof), so it is cut level at the floor of the pocket over the hub. This is the part
below the cut - the faceted disc's lowest band (the rim, the recess's ceiling the seat on the thrust bearing's upper
washer), the stub (the bearings' bore, its end where the 120T's hub end meets it), the 120T's 4 holes and their nuts'
hex pockets in its top face (the pocket's floor, where the shoulder home tray sits), the lower ends of the motor leg's
foot nut slots - and the 4 holes of the screws that hold j1_coupler on it (parts/base/coupler_plate_screws), their nuts
(parts/base/coupler_plate_nuts) pressed up into hex pockets in its underside, inside the thrust washers' bore. It
prints upside down, on the cut: the stub and the recess face up. In j1_coupler's part frame (lib/yaw_coupler/body.py
build_coupler_plate); lib/mounts.py places it at identity on j1_coupler#1.
"""
import pathlib

from cad import step

from lib.yaw_coupler import DEFAULT
from lib.yaw_coupler.body import build_coupler_plate

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: a small separate print, the plate j1_coupler bolts to
REFERENCE = None
CONVERTED = True


@step(out="j1_coupler_plate.step")
def j1_coupler_plate():
    """The plate in j1_coupler's part frame (on the base_yaw axis at the seat-ring top); the assembly owns placement."""
    part = build_coupler_plate(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j1_coupler_plate()   # build: writes the sibling j1_coupler_plate.step
