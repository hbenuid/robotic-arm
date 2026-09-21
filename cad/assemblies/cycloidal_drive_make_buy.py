"""The cycloidal drive sorted by where each part comes from - a WORKING VIEW, NOT the module:
assemblies/cycloidal_drive.py stays the model the arm places and EXPECTED / BODIES describe.

    cycloidal_drive_make_buy
    |- printed   the 6 printed parts                                    (MAKE_BUY_TINTS["printed"])
    |- bought    bearings, motor, dowel pins, bolts, nuts - COTS = True (MAKE_BUY_TINTS["bought"])

The rows are cycloidal_drive.OCCURRENCES (never retyped), in the module frame (Z = the motor axis).
Purchased items that are NOT modelled - the arm-mount bolts and nuts, grease - are only on the buy
list: ./cadtool python tools/bom.py --module cycloidal_drive.

Run:  ./cadtool gen assemblies/cycloidal_drive_make_buy.py    -> assemblies/cycloidal_drive_make_buy.step (git-ignored)
      ./cadtool show assemblies/cycloidal_drive_make_buy.py   -> preview in the OCP CAD Viewer (no build)
"""

from cadgen import build123d as bd
from cadgen import step

from assemblies import cycloidal_drive as full
from assemblies._occurrences import make_buy_children
from lib.assembly import assembly


@step
def cycloidal_drive_make_buy():
    """cycloidal_drive()'s 18 rows under the 'printed' / 'bought' nodes, tinted, in the module frame."""
    rows = [(part, role, bd.Location(tuple(pos))) for part, role, pos in full.OCCURRENCES]
    return assembly("cycloidal_drive_make_buy", make_buy_children(rows))


if __name__ == "__main__":
    cycloidal_drive_make_buy()   # build: writes the sibling cycloidal_drive_make_buy.step (preview: ./cadtool show assemblies/cycloidal_drive_make_buy.py)
