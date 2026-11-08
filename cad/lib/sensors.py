"""The arm's sensors - a leaf (lib/params.py re-exports it; nothing below it imports lib.params).

KY-003 hall-effect sensor module (parts/joints/ky003_hall_sensor): an A3144 / 3144E unipolar hall switch in the
"UA" 3-lead SIP (TO-92S class) on a small PCB with a pull-up, a status LED and a right-angle 3-pin 2.54 header. The
chip lies flat past the board's sensor end on its three leads, its branded face up: it switches when a magnet's SOUTH
pole comes up to that face (the field along the face's normal). The endstop / home sensors of the joints - bought as
a 10-pack; no catalog model (step.parts: no KY-003, no A3144, vendor/README.md).

Part frame (the module's own): the board's underside on z = 0, centred in Y, its sensor end at x = 0 (the chip
toward -X), the header's pins out past its far end toward +X, the branded face up (+Z). The board's size comes from
the listings (18.5 x 15; the 29 x 15 x 7 some give is the whole module: the chip + the board + the pins along X, the
pins' tails + the board + the header's housing along Z - tests/test_params_invariants.py holds the model to it); every
[ESTIMATE] is for the calipers on a module in hand (docs/open_issues.md).

The home magnets (a disc in the turning part, over the chip at the joint's home angle) and their field at the chip:
disc_magnet_bz() / switch_half_width() - how wide the switch's zone is for each operate point the A3144 may have
(A3144_OPERATE_MT), which sets where a home magnet must sit (tests/base/test_yaw_home.py).

Tags as in lib/params.py: [MEASURE] [DATASHEET] [DESIGN] [REFERENCE] [ESTIMATE]. Units mm / g.
"""
import math

# --- A3144 hall switch, Allegro "UA" package ---------------------------------------------------------
A3144_BODY_W = 4.09             # [DATASHEET] across the leads (Y)
A3144_BODY_H = 3.02             # [DATASHEET] along the leads (X)
A3144_BODY_T = 1.52             # [DATASHEET] branded face .. back (Z)
A3144_LEAD_PITCH = 1.27         # [DATASHEET] 3 leads
A3144_LEAD_W = 0.43             # [ESTIMATE] across the lead (Y)
A3144_LEAD_T = 0.38             # [ESTIMATE] the lead's thickness
A3144_HALL_DEPTH = 0.50         # [ESTIMATE] the hall element under the branded face, at the body's centre

# --- KY-003 module --------------------------------------------------------------------------------
KY003_BOARD_L = 18.5            # [ESTIMATE] the sensor end .. the header end (X), the listings' board size
KY003_BOARD_W = 15.0            # [ESTIMATE] (Y)
KY003_BOARD_T = 1.6             # [ESTIMATE] FR4
KY003_HOLE_DIA = 3.0            # [ESTIMATE] 2 mounting holes at the sensor end's corners (the listing photo) ...
KY003_HOLE_INSET = 2.5          # [ESTIMATE] ... their centres this far in from the end and the long edges
KY003_LEAD_OUT = 1.5            # [ESTIMATE] the chip's leads bridge this far from its body to the board's end
KY003_LEAD_BEND_X = 4.0         # [ESTIMATE] where they turn down through the board (between the mounting holes)
KY003_LEAD_TAIL = 1.0           # [ESTIMATE] their cut ends under the board
KY003_PINS = 3                  # [DATASHEET] header: - (GND), + (VCC), S (signal)
KY003_PIN_PITCH = 2.54          # [DATASHEET] 0.1 in header
KY003_PIN_W = 0.64              # [DATASHEET] square pin
KY003_HEADER_H = 2.54           # [DATASHEET] the right-angle header's housing: this high and this deep (X), at the board's far end
KY003_PIN_OUT = 6.0             # [ESTIMATE] the pins past the board's end
KY003_PIN_TAIL = 3.0            # [ESTIMATE] their tails under the board, one pitch behind the housing
KY003_MASS_G = 2.0              # [ESTIMATE] PCB + chip + header; weigh one

# --- The home magnets: NdFeB discs magnetised through their thickness, SOUTH pole toward the chip ---------------
A3144_OPERATE_MT = (7.0, 35.0)  # [DATASHEET] the A3144's operate point - the field at its branded face that switches it
#                                 on - across parts at 25 C (70 .. 350 G); it releases a little lower
NDFEB_N52_BR_T = 1.43           # [DATASHEET] N52 remanence (1.42 .. 1.47 T)
HOME_MAGNET_DIA = 6.0           # [DESIGN] the base_yaw home magnet: a Ø6 x 3 N52 disc glued into a pocket in the 120T's
HOME_MAGNET_T = 3.0             # [DESIGN] outer face (lib/pulley/params.py YAW; parts/base/yaw_home_magnet)
MAGNET_POCKET_CLEAR = 0.1       # [DESIGN] a magnet's pocket: this much over its diameter each side and past its thickness
#                                 (the glue), its face flush with the face the pocket opens in


def disc_magnet_bz(dia: float, t: float, rho: float, gap: float, br: float = NDFEB_N52_BR_T, n: int = 48) -> float:
    """The field along the axis (mT) of a disc magnet (dia x t, magnetised through t) at `gap` beyond one pole face and
    `rho` off its axis: the two pole faces as charged discs, Br / 4 pi times the integral of +/- dz / d^3 over each,
    midpoint rule on an n x 2n polar grid (half of it: the field is symmetric about the plane through the axis and the
    point) - within 0.2 % of the exact on-axis value at the gaps used here."""
    r_disc = dia / 2.0
    dr, dth = r_disc / n, math.pi / n
    total = 0.0
    for face_z, sign in ((gap, 1.0), (gap + t, -1.0)):   # the near face, the far face (opposite pole)
        for i in range(n):
            r = (i + 0.5) * dr
            for j in range(n):
                th = (j + 0.5) * dth
                dx, dy = rho - r * math.cos(th), r * math.sin(th)
                d2 = dx * dx + dy * dy + face_z * face_z
                total += sign * face_z / (d2 * math.sqrt(d2)) * r
    return br / (4.0 * math.pi) * 2.0 * total * dr * dth * 1e3


def switch_half_width(dia: float, t: float, gap: float, operate_mt: float, br: float = NDFEB_N52_BR_T) -> float:
    """How far off the magnet's axis (mm, sideways at `gap` under its pole face) the field stays at `operate_mt` or
    more - the switch's zone is twice it wide; 0.0 when even the axis is short of it."""
    if disc_magnet_bz(dia, t, 0.0, gap, br) < operate_mt:
        return 0.0
    lo, hi = 0.0, dia + 4.0 * gap
    for _ in range(30):
        mid = (lo + hi) / 2.0
        lo, hi = (mid, hi) if disc_magnet_bz(dia, t, mid, gap, br) >= operate_mt else (lo, mid)
    return lo


def ky003_hall_point() -> tuple[float, float, float]:
    """The A3144's hall element in the KY-003's part frame: the chip body's centre, A3144_HALL_DEPTH under its
    branded face - where a magnet's axis must point to switch it (placing the part: the magnet's path crosses this)."""
    x = -(KY003_LEAD_OUT + A3144_BODY_H / 2.0)
    return (x, 0.0, KY003_BOARD_T + A3144_BODY_T - A3144_HALL_DEPTH)
