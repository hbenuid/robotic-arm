"""yaw_home_nuts - purchased (COTS) part: the base_yaw home bracket's 2 M3 hex nuts (ISO 4032), in the motor plate's top.

In the arm: one pattern (yaw_home_nuts#1, a mount on base#1 in the bracket's frame - lib/mounts.py YAW_HOME_MOUNTS):
each dropped into its hex pocket in the base's motor plate's top (lib/base/body.py build_base, YawHomeParams.nut /
nut_clear: nut.h deep, flush), its bearing face on the pocket's floor, on the bracket's M3s (yaw_home_screws) up through
the plate; the pocket holds it from turning, so the screws tighten from under the base. The pockets are reached from
under the base through its open -X side.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py hex_nut(): no thread, the bore at the
nominal diameter) and tests/base/test_yaw_home.py locks its numbers. Frame: the bracket's (lib/base/layout.py
home_bracket_frame()), the nuts standing on the pockets' floors (z = bracket_t + the plate's thickness - nut.h), a
corner on +X (the pockets').
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT, home_screw_points
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import YAW_HOME_NUTS_MASS_G

HOME = DEFAULT.home
POINTS = home_screw_points(DEFAULT)
FLOOR = HOME.bracket_t + DEFAULT.plate.y[1] - DEFAULT.plate.y[0] - HOME.nut.h   # the pockets' floors in the bracket's frame

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_HOME_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.nut.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = ("dropped into the 2 hex pockets in the base's motor plate's top, from under the base through its open "
                 "side, before the home bracket's M3s go up")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nuts: the geometry until a vendor model exists."""
    return pattern([hex_nut(HOME.nut, 0.0, (x, -z), FLOOR) for x, z in POINTS])


@step(out="yaw_home_nuts.step")
def yaw_home_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_home_nuts()   # build: writes the sibling yaw_home_nuts.step
