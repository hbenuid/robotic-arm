# The arm's design language

**Purpose:** the one set of rules for how the arm looks, so every printed part reads as part of the same machine.
**Audience:** agent and human. The values live in `lib/style.py`; a change to them is a design choice (`cad/AGENTS.md`
"Design choices": the preview first).

The arm's parts came from three places (the SolidWorks capture, the CadQuery port of the cycloidal drive, the parts
designed here), each with its own look; these rules make them one.

## The rules
- **Structure is faceted.** Every link end, leg top and frame is a regular polygon of `lib/style.py SIDES` (an octagon)
  with a flat on each of the part's own axes - `lib/geom.py faceted_end()`, `forearm_roll_block` the pattern -, and the
  base and its motor mount are an octagonal frustum on an octagonal foot - carried on up through the fork's disc, which
  turns on it: the disc is the same octagon on the same cone, and the arm's zero puts its flats on the base's
  (`lib/datum.py BASE_TURN_DEG`). Where a slab tapers onto its end, the bend is sharp like the facets; where a plate
  grows out of a faceted body (the fork's legs out of its disc, the forearm's web out of its flange), it does so in
  planar facets, the body's flat facing it running on up its face (`lib/geom.py hull_solid`); where the plate is the
  wider, the body's facets run on along it into its edge chamfer.
- **What turns stays round.** A part that turns against a neighbour keeps its round: the cycloidal drive, a bearing boss
  standing proud of a face (the roll frame's, the wrist frame's), the pulleys and the base's cap - a facet there would
  sweep a corner past the part beside it. The one exception is the fork's disc, the base's octagon carried up: its
  corners sweep over the base's top face, nothing beside them. A link's end with a bearing seat inside it is structure:
  faceted round the seat.
- **Joint pieces stay plain; leftovers go.** A small functional piece tucked between a frame and the link it turns on -
  the elbow's and the wrist's flange posts (`lib/forearm/params.py RollDriveParams` `post_*`, `lib/wrist/params.py
  FlangeParams`) - is the plainest form that does the job: a round post, its foot chamfered, no skirt, taper or cover;
  offering a styled one, offer it beside the plain one, which is not assumed to lose. Geometry that carries nothing (a
  SolidWorks remnant past the body that replaced it) is removed outright, what stays cut flush with that body (the
  wrist flange's post with the roll frame) - never kept as a smaller version of itself.
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
- **Back panels are grilled.** A roll drive's removable back panel carries a grill over its shaft's bay: straight slots
  `lib/style.py GRILL_SLOT` wide at `GRILL_PITCH`, cut through, one on the roll axis, the same on both panels, so the
  turning shaft shows behind it. The slots are clipped to a field inside the panel's locating rim and clear of its
  magnets and notches, its outline faceted - the frame's octagon carried in, 45° facets across its corners - and a
  plain border of the panel round it. The slots' edges stay sharp like the panel's other holes. It is a look, not a
  vent: neither panel covers a motor or a driver board.

## Where it stands
In the model: the colours; the upper arm (`j1_link`: its faceted elbow end, its slab's outline chamfered); the roll
frame (`forearm_roll_block`: its tower the forearm flange's octagon carried back through it, its front face's long edges
faceted, the bearing boss rising out of them, its outline chamfered, its back panel grilled); the forearm (`j2_link`: its web as wide as its
cover's end, its wrist end and boss faceted and chamfered, its flange faceted and the web growing out of it, the
flange's lower facets carried on along the web into its chamfer); the wrist (`wrist_link`: its roll frame
laid out as the roll frame's - an octagon about the pitch axis hulled to its tower, the far face closed, a grilled back
panel over the bay, its front edges faceted, the bearing boss rising out of them -, no plate past it: the wrist flange's
post stands on the frame, cut flat on its front face and rear); the fork (`j1_coupler`, `j1_motor_leg`: their legs' tops faceted and chamfered - on the motor leg not on the
flats the sleeve's screws seat on, which span its whole thickness -, the legs growing out of the disc in planar facets,
the disc and the ring on it octagons); the base and
its motor mount (`ShellParams.sides`: the round half, the mount's end and the foot faceted, the slits laid out flat by
flat, `lib/base/layout.py slits`). Not yet - `docs/open_issues.md` "Not modelled yet" -: the covers.
