"""yaw_home_inserts - purchased (COTS) part: the base_yaw home bracket's 2 M3 heat-set inserts (knurled brass).

In the arm: one pattern (yaw_home_inserts#1, a mount on base#1 in the bracket's frame - lib/mounts.py YAW_HOME_MOUNTS):
pressed hot into the blind holes in the motor plate's underside (lib/base/body.py build_base, YawHomeParams.insert:
its length deep, 1.0 of the plate over it), their open ends flush with the underside, where the bracket's M3s
(yaw_home_screws) come up. The plate is reached from under the base.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py heat_set_insert(): a plain sleeve)
and tests/base/test_yaw_home.py locks its numbers. Frame: the bracket's (lib/base/layout.py home_bracket_frame()),
the inserts from z = bracket_t (the plate's underside) up.
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT, home_screw_points
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import heat_set_insert
from lib.params import YAW_HOME_INSERTS_MASS_G

HOME = DEFAULT.home
POINTS = home_screw_points(DEFAULT)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_HOME_INSERTS_MASS_G   # [ESTIMATE] the modelled brass (lib/params.py)
PURCHASE_SPEC = (f"M{HOME.insert.d:g} heat-set insert, {HOME.insert.length:g} long, Ø{HOME.insert.od:g} knurl "
                 f"(for a Ø{HOME.insert.hole_dia:g} hole)")
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "pressed hot into the base's motor plate from under the base, flush with its underside: the home bracket's M3s"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain sleeves: the geometry until a vendor model exists."""
    return pattern([heat_set_insert(HOME.insert, (x, -z), HOME.bracket_t) for x, z in POINTS])


@step(out="yaw_home_inserts.step")
def yaw_home_inserts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_home_inserts()   # build: writes the sibling yaw_home_inserts.step
