"""ky003_hall_sensor - purchased (COTS) part: KY-003 hall-effect sensor module (A3144 unipolar hall switch on a PCB).

The joints' endstop / home sensor: the A3144 switches when a magnet's SOUTH pole comes up to its branded face; the
board carries its pull-up, a status LED and the 3-pin header (-, +, S) - S reads low while the magnet is there. The
chip is rated 4.5..24 V: run it at 5 V. NOT PLACED yet (UNPLACED below, parts/AGENTS.md "Modelled, not placed yet"):
the forearm roll's home sensor, the chip on the frame's tower beside the stop post, in the shaft's bay behind bearing 1
(forearm_roll_block), and a magnet in the shaft's stop lug (forearm_roll_shaft) - neither is designed yet; the bay is
open on the frame's +N face, so the board can sit there or the chip go on its leads (docs/open_issues.md).

No catalog model (step.parts has no KY-003 and no A3144, only bare TO-92S packages - vendor/README.md) and no
SolidWorks export: a NATIVE COTS part (lib/reference.py NATIVE_COTS) - its envelope IS the geometry and its reference
is that envelope (reference/native/ky003_hall_sensor.step, tools/reference/import_native.py). Three solids: the board
(its two mounting holes, the holes the leads and the pins pass through), the chip (body + its three leads), the
header (housing + its three pins); the SMD LED and resistors (< 1 mm) are left out. Every dimension: lib/sensors.py.

Frame: the board's underside on z = 0, centred in Y, its sensor end at x = 0 - the chip past it toward -X, lying flat
on its leads, branded face up (+Z); the header at the far end, its pins out toward +X. The hall element:
lib/sensors.py ky003_hall_point().
"""
import pathlib

from cadgen import build123d as bd
from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.geom import single_solid, through
from lib.params import (
    A3144_BODY_H,
    A3144_BODY_T,
    A3144_BODY_W,
    A3144_LEAD_PITCH,
    A3144_LEAD_T,
    A3144_LEAD_W,
    KY003_BOARD_L,
    KY003_BOARD_T,
    KY003_BOARD_W,
    KY003_HEADER_H,
    KY003_HOLE_DIA,
    KY003_HOLE_INSET,
    KY003_LEAD_BEND_X,
    KY003_LEAD_OUT,
    KY003_LEAD_TAIL,
    KY003_MASS_G,
    KY003_PIN_OUT,
    KY003_PIN_PITCH,
    KY003_PIN_TAIL,
    KY003_PIN_W,
    KY003_PINS,
)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = KY003_MASS_G   # [ESTIMATE] see lib/sensors.py
PURCHASE_SPEC = "KY-003 hall-effect sensor module (A3144 / 3144E unipolar hall switch, 3-pin 2.54 header), 5 V"
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = "sold in packs of 10; the chip is rated 4.5-24 V (not the listings' 3.3 V) - power it at 5 V"
UNPLACED = ("the forearm roll's home sensor: its seat beside the frame's stop post and the magnet in the shaft's stop lug are not "
            "designed yet (docs/open_issues.md) - tools/bom.py EXTRAS carries the order line")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY

LEAD_EMBED = 0.5   # the chip's leads start this far inside its body, so body + leads fuse into one solid


def _box(x, y, z):
    """A box over the x / y / z ranges."""
    return bd.Pos(x[0], y[0], z[0]) * bd.Box(x[1] - x[0], y[1] - y[0], z[1] - z[0],
                                             align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))


def _row(pitch: float) -> list[float]:
    """The Y of each of the three leads / pins, pitch apart, centred on the board."""
    return [(i - (KY003_PINS - 1) / 2.0) * pitch for i in range(KY003_PINS)]


def _lead_tails():
    """Where the chip's leads turn down through the board to their cut ends under it."""
    w, t = A3144_LEAD_W / 2.0, A3144_LEAD_T
    return [_box((KY003_LEAD_BEND_X, KY003_LEAD_BEND_X + t), (y - w, y + w), (-KY003_LEAD_TAIL, KY003_BOARD_T + t))
            for y in _row(A3144_LEAD_PITCH)]


def _pin_tails():
    """The header pins' tails, down through the board one pitch behind the housing."""
    x_tail, w = KY003_BOARD_L - KY003_HEADER_H / 2.0 - KY003_PIN_PITCH, KY003_PIN_W / 2.0
    z_pin = KY003_BOARD_T + KY003_HEADER_H / 2.0
    return [_box((x_tail - w, x_tail + w), (y - w, y + w), (-KY003_PIN_TAIL, z_pin + w)) for y in _row(KY003_PIN_PITCH)]


def _board():
    body = _box((0.0, KY003_BOARD_L), (-KY003_BOARD_W / 2.0, KY003_BOARD_W / 2.0), (0.0, KY003_BOARD_T))
    for sy in (1.0, -1.0):
        body = body - through(KY003_HOLE_DIA / 2.0, KY003_BOARD_T, (KY003_HOLE_INSET, sy * (KY003_BOARD_W / 2.0 - KY003_HOLE_INSET)))
    for tail in _lead_tails() + _pin_tails():
        body = body - tail
    return single_solid(body)


def _chip():
    """The A3144 flat past the board's sensor end, its leads along the board's top to where they turn down."""
    x1 = -KY003_LEAD_OUT
    top = KY003_BOARD_T + A3144_BODY_T
    body = _box((x1 - A3144_BODY_H, x1), (-A3144_BODY_W / 2.0, A3144_BODY_W / 2.0), (KY003_BOARD_T, top))
    w, t = A3144_LEAD_W / 2.0, A3144_LEAD_T
    for y in _row(A3144_LEAD_PITCH):
        body = body + _box((x1 - LEAD_EMBED, KY003_LEAD_BEND_X + t), (y - w, y + w), (KY003_BOARD_T, KY003_BOARD_T + t))
    for tail in _lead_tails():
        body = body + tail
    return single_solid(body)


def _header():
    """The right-angle header: the housing on the board's far end, the pins through it and out past the end."""
    half = KY003_PINS * KY003_PIN_PITCH / 2.0
    body = _box((KY003_BOARD_L - KY003_HEADER_H, KY003_BOARD_L), (-half, half), (KY003_BOARD_T, KY003_BOARD_T + KY003_HEADER_H))
    x_tail, w = KY003_BOARD_L - KY003_HEADER_H / 2.0 - KY003_PIN_PITCH, KY003_PIN_W / 2.0
    z_pin = KY003_BOARD_T + KY003_HEADER_H / 2.0
    for y in _row(KY003_PIN_PITCH):
        body = body + _box((x_tail - w, KY003_BOARD_L + KY003_PIN_OUT), (y - w, y + w), (z_pin - w, z_pin + w))
    for tail in _pin_tails():
        body = body + tail
    return single_solid(body)


def _envelope():
    """The board, the chip and the header - three solids: the geometry until a vendor model exists."""
    return pattern([_board(), _chip(), _header()])


@step
def ky003_hall_sensor():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    ky003_hall_sensor()   # build: writes the sibling ky003_hall_sensor.step
