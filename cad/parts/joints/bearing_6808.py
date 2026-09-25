"""bearing_6808 - purchased (COTS) part: 6808-2RS (61808) thin-section deep-groove ball bearing, 40 x 52 x 7 - the
two ring bearings of the forearm roll drive (assemblies/forearm_roll_drive.py, one seat in the elbow block).

No vendor model (step.parts has nothing above a 17 mm bore, vendor/README.md) and no SolidWorks export: a NATIVE
COTS part (lib/reference.py NATIVE_COTS) - its envelope IS the geometry, and its reference is that envelope
(reference/native/bearing_6808.step, tools/reference/import_native.py). Frame: axis on Z, standing on z=0.
Dimensions: lib/forearm/params.py RollDriveParams (bearing_bore / od / width). In the arm: x2.
"""
import pathlib

from cadgen import read_step, step

from lib.datum import IDENTITY, to_location
from lib.forearm import DEFAULT, ForearmConfig
from lib.geom import cylinder
from lib.params import BEARING_6808_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BEARING_6808_MASS_G   # [DATASHEET] see lib/params.py
PURCHASE_SPEC = "6808-2RS (61808) thin-section deep-groove ball bearing, {0.bearing_bore:g} x {0.bearing_od:g} x {0.bearing_width:g}".format(DEFAULT.drive)
PURCHASE_QTY = 1    # pieces per occurrence (the drive places it twice)
PURCHASE_NOTE = "any brand; sealed (2RS) - the belt side is open to the room"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope(cfg: ForearmConfig = DEFAULT):
    """The annulus (outer race OD, bore, width): the geometry until a vendor model exists."""
    d = cfg.drive
    return cylinder(d.bearing_od / 2.0, d.bearing_width) - cylinder(d.bearing_bore / 2.0, d.bearing_width)


@step
def bearing_6808():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = read_step(VENDOR_STEP).moved(to_location(VENDOR_TO_REF)) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


if __name__ == "__main__":
    bearing_6808()   # build: writes the sibling bearing_6808.step
