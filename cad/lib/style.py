"""The arm's design language - what every part and assembly shares for how the arm looks (docs/design_language.md).

The colours, one per kind of part, the same in the arm and in every module standing alone
(assemblies/_occurrences.py colour()): a printed part is BODY (the structure) or ACCENT (what drives, turns or covers -
the part module declares ACCENT = True, parts.accent()), each one of FILAMENTS (what it can be printed in); a purchased
part (parts.bought()) is BOUGHT, a belt (parts/*/<joint>_belt.py) BELT.

The shapes: a faceted end (lib/geom.py faceted_end()) is a regular polygon of SIDES with a flat on each of the part's
own axes; a printed part's main faces have their outline chamfered EDGE_CHAMFER at 45 deg (lib/geom.py chamfer_outline());
a back panel carries a grill of GRILL_SLOT slots at GRILL_PITCH.

A leaf (lib/AGENTS.md): it imports nothing, so every lib/ package may take its values from it.
"""

SIDES = 8              # [DESIGN] the faceted ends: an octagon, a flat on each of the part's own axes (forearm_roll_block's)
EDGE_CHAMFER = 1.5     # [DESIGN] the 45 deg chamfer on the outline of a printed part's main faces: it reads as an edge
#                        at arm's length and prints with no support even on the edge on the bed
COVER_WALL = 2.5       # [DESIGN] a belt cover's wall and lid ...
COVER_CLEAR = 2.0      # [DESIGN] ... standing this far clear of every pulley, belt and screw head inside it - a link a cover
#                        follows is as wide as what turns inside the cover needs (lib/forearm/params.py WebParams.half_w)
GRILL_SLOT = 2.0       # [DESIGN] a back panel's grill: straight slots this wide, cut through, running up the panel ...
GRILL_PITCH = 4.0      # [DESIGN] ... at this pitch, one on the panel's centre line (the ribs between them 2.0: 5 perimeters
#                        at 0.4), clipped to the panel's field over its shaft's bay (docs/design_language.md)

# [MEASURE] the PETG colours a printed part can be bought in, each its swatch's hex in the shop's colour picker (screen
# colours sampled off the swatches, not a measured filament; the shop names only its white, the other names are read
# off the swatches). A printed part's colour is one of them, stored as its viewer_hex() so the viewer draws it as the
# filament
FILAMENTS = {
    "white": "#EFF0F1",
    "peach": "#F3DFD5",
    "olive": "#5A6D3D",
    "ivory": "#EEE7D4",
    "black": "#000000",
    "red": "#CD001A",
    "teal": "#00ADAD",
    "yellow": "#FEDD00",
    "grey": "#75787B",
    "brown": "#362111",
    "blue": "#1729AB",
    "green": "#1DAA27",
    "gold": "#F1C400",
    "magenta": "#D9027D",
    "sky_blue": "#00A3E0",
    "salmon": "#FF9A8D",
    "khaki": "#C2B29A",
    "purple": "#685BC7",
    "pink": "#F9C5C2",
    "orange": "#FF6A14",
}


def viewer_hex(filament: str) -> str:
    """The hex the viewer and `./cadtool snapshot` (cad 0.11) draw as `filament`. They draw a STEP colour darker - the
    swatch's own orange comes out a red-orange -, so each channel is taken as linear and encoded to sRGB (white and
    black stay as they are)."""
    out = []
    for i in (1, 3, 5):
        c = int(filament[i:i + 2], 16) / 255
        out.append(round(255 * (12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055)))
    return "#" + "".join(f"{v:02X}" for v in out)


BODY = viewer_hex(FILAMENTS["ivory"])      # [DESIGN] the printed structure
ACCENT = viewer_hex(FILAMENTS["orange"])   # [DESIGN] the printed parts that drive, turn or cover
BOUGHT = "#9AA0A6"     # [DESIGN] grey: every purchased part
BELT = "#717276"       # [DESIGN] the belts, darker than the other purchased parts
