"""The arm's design language - what every part and assembly shares for how the arm looks (docs/design_language.md).

The colours, one per kind of part, the same in the arm and in every module standing alone
(assemblies/_occurrences.py colour()): a printed part is BODY (the structure) or ACCENT (what drives, turns or covers -
the part module declares ACCENT = True, parts.accent()); a purchased part (parts.bought()) is BOUGHT, a belt
(parts/*/<joint>_belt.py) BELT.

A leaf (lib/AGENTS.md): it imports nothing, so every lib/ package may take its values from it.
"""

BODY = "#E6E3DC"       # [DESIGN] off-white: the printed structure
# [DESIGN] orange: the printed parts that drive, turn or cover. The filament is #E2581C; the viewer and `./cadtool
# snapshot` (cad 0.10) draw a STEP colour that saturated much darker (#E2581C itself comes out a deep red), and this
# value - #E2581C's channels taken as linear and encoded to sRGB - draws close to it
ACCENT = "#F29F5D"
BOUGHT = "#9AA0A6"     # [DESIGN] grey: every purchased part
BELT = "#717276"       # [DESIGN] the belts, darker than the other purchased parts
