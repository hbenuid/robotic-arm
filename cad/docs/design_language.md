# The arm's design language

**Purpose:** the one set of rules for how the arm looks, so every printed part reads as part of the same machine.
**Audience:** agent and human. The values live in `lib/style.py`; a change to them is a design choice (`cad/AGENTS.md`
"Design choices": the preview first).

The arm's parts came from three places (the SolidWorks capture, the CadQuery port of the cycloidal drive, the parts
designed here), each with its own look; these rules make them one.

## The rules
- **Structure is faceted.** Every link end, leg top and frame is a regular polygon of `lib/style.py SIDES` (an octagon)
  with a flat on each of the part's own axes - `lib/geom.py faceted_end()`, `forearm_roll_block` the pattern -, and the
  base and its motor mount are an octagonal frustum on an octagonal foot. Where a slab tapers onto its end, the bend is
  sharp like the facets.
- **What turns stays round.** A part that turns against a neighbour keeps its round: the cycloidal drive, a bearing boss
  standing proud of a face (the roll frame's, the wrist frame's), the pulleys and the base_yaw turntable (the fork's
  disc, the base's cap) - a facet there would sweep a corner past the part beside it. A link's end with a bearing seat
  inside it is structure: faceted round the seat.
- **Edges are chamfered.** One 45° chamfer (`lib/style.py EDGE_CHAMFER`) on the outline of every printed part's main
  faces and on every cover - `lib/geom.py chamfer_outline()`, cut on the main body before its holes and pockets, so
  holes, bores and seats stay sharp. A 45° chamfer prints with no support, even on an edge facing the bed.
- **Two tones.** The printed structure is `BODY`; what drives, turns or covers is `ACCENT` (its part module declares
  `ACCENT = True`, `parts.accent()`); a purchased part is `BOUGHT` and a belt `BELT`. `assemblies/_occurrences.py
  colour()` gives each part its colour, the same in the arm and in each module standing alone. Why `ACCENT` is not the
  filament's own hex: its comment in `lib/style.py`.
- **Belts are covered.** Each belt run (the upper arm's, the forearm's, the base's) gets a cover and the roll frame's open
  face a lid. A cover follows its link's outline, so the covered link reads as one piece, stands clear of the pulleys,
  belt and screw heads inside, and is held by disc magnets like the roll frame's back panel (`forearm_roll_back_panel`):
  it comes off by hand to reach its belt's tensioner. Its wall is `lib/style.py COVER_WALL`, `COVER_CLEAR` off what it
  covers. A link a cover follows is as wide as what turns inside the cover needs: the forearm's web
  (`lib/forearm/params.py WebParams.half_w`) round the wrist's 90T.

## Where it stands
In the model: the colours; the upper arm (`j1_link`: its faceted elbow end, its slab's outline chamfered); the roll
frame (`forearm_roll_block`: its outline chamfered); the forearm (`j2_link`: its web as wide as its cover's end, its
wrist end and boss faceted and chamfered, its flange faceted); the wrist (`wrist_link`: its roll frame and its plate's
end on the pitch axis faceted and chamfered); the fork (`j1_coupler`, `j1_motor_leg`: their legs' tops faceted and
chamfered - on the motor leg not on the flats the sleeve's screws seat on, which span its whole thickness); the base and
its motor mount (`ShellParams.sides`: the round half, the mount's end and the foot faceted, the slits laid out flat by
flat, `lib/base/layout.py slits`). Not yet - `docs/open_issues.md` "Not modelled yet" -: the covers.
