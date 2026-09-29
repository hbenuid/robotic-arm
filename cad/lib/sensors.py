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

Tags as in lib/params.py: [MEASURE] [DATASHEET] [DESIGN] [REFERENCE] [ESTIMATE]. Units mm / g.
"""
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


def ky003_hall_point() -> tuple[float, float, float]:
    """The A3144's hall element in the KY-003's part frame: the chip body's centre, A3144_HALL_DEPTH under its
    branded face - where a magnet's axis must point to switch it (placing the part: the magnet's path crosses this)."""
    x = -(KY003_LEAD_OUT + A3144_BODY_H / 2.0)
    return (x, 0.0, KY003_BOARD_T + A3144_BODY_T - A3144_HALL_DEPTH)
