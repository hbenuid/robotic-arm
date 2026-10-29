# assemblies/ — occurrences, placements, modules

Loads when you work in `assemblies/`. `arm.py` (the whole arm), `gripper.py` (the gripper module, placed from
`placements.json`), `cycloidal_drive.py` / `forearm_roll_drive.py` / `wrist_roll_drive.py` (code-driven modules), `_occurrences.py` (the
placing / tinting helpers, also used by `robot/`, `tools/` and `tests/`). Every output STEP is git-ignored.

## Assembly & placements
- `reference/placements.json` (from `tools/reference/extract_placements.py`; schema: `reference/README.md`) holds
  every occurrence: `rel` (to its parent node) and `world`, as `Location(position, rotation_xyz_deg)`; keys
  `"<part>#<n>"`, module `"gripper#1"`. Treat it as an immutable input. An occurrence the DESIGN has replaced is
  **retired** in `lib/placements.py RETIRED` (`j3_coupler#1`: the roll drive's block took over its
  features, `docs/forearm_roll.md`; `gt2_pulley_90t#1` / `#2`: re-seated `PULLEY_SEAT_SHIFT` out as mounts, below;
  `nema17_pancake#1` / `gt2_pulley_20t#1`: the wrist roll drive carries them, `docs/wrist_roll.md`): its record stays,
  `P.keys()` leaves it out (`retired=True` lists the file), no table / link / total
  claims it (`test_placements.py`, `test_assembly.py`, `test_robot.py` follow `keys()`). A PART the design drops
  entirely (the link caps `j1_cap` / `j2_cap_1` / `j2_cap_2`, `gripper_clamp_bracket`) cannot be retired - its part module,
  `CUSTOM` row and manifest entry go, and a record must name a known part -: its SolidWorks product goes into
  `lib/reference.py SKIPPED_PRODUCTS` and `tools/reference/mount_placements.py` (the merge mode, no monolith) moves the
  record to `skipped` as the entry an extraction writes (pose, totals, reason) - `test_skipped_nodes_are_the_dropped_products`.
- **Shifted occurrences** (`lib/placements.py SHIFTS`): a link whose length the design changes (`j1_link`
  `slab.elbow_x`, `j2_link` `web.wrist_x`) moves every capture record beyond it along its own +X by the parameter's
  DEFAULT − LEGACY - along its +Y by `across_y` where the design moved its far end across it (`j1_link`'s elbow end,
  slid with the arm until its underside is flush with the drive shell's hub end: `lib/upper_arm/layout.py arm_slide`;
  the mounts hosted on `j1_link#1` itself add it to their frames, `lib/params.py J1_ARM_SLIDE`), and along its +Z by
  `normal_z` where it moved it off the link's plane (the wrist roll drive, below); the yoke
  (`j1_coupler`) moves the drive, `j1_link` and all beyond along its +X (N) by `ForkParams.face_x` −
  `CAPTURE_FACE_X`, the middle of the discs onto the base_yaw axis; the forearm roll drive (`forearm_roll_drive`, anchor
  `j2_link#1`) moves `j2_link#1` and all beyond along j2_link's +Y by `ForearmConfig.elbow_offset` (its roll axis across
  the elbow axis, the elbow's own records - the retired `j3_coupler#1` / `gt2_pulley_90t#1` - staying); the wrist roll drive
  (`wrist_roll_drive`, anchor `wrist_link#1`) moves the gripper onto its roll axis - along wrist_link to the roll pulley's
  face, across it by the drive's centre distance, off its plate (`normal_z`) to the axis' height — the file stays as
  extracted, `location()` applies the shift
  (`to_location()` / `to_record()` stay raw: the writers and the round-trip test use them). A `LinkShift` lists the
  TOP-LEVEL records beyond its link, the retired hosts included; a module's children follow its world pose (their
  `rel` is to it); mounted records are never listed — `mount_placements.py` resolves their hosts through
  `record_location()`, so the shift is baked into their records. `robot/frames.py` moves the joint origins beyond a
  link with `P.shifted()`, `tests/totals.py` a record's bbox with `record_shift()`. `test_placements.py` checks the
  lists against `robot/frames.py LINKS`, each anchor axis against its record, and where the links put what lies
  beyond them.
- **Turned occurrences** (`lib/placements.py TURNS`, after the SHIFTS): a link the design turned about an axis of its
  own turns every record it lists - the forearm: `j2_link#1` and all beyond it half a turn about its roll axis
  (`ForearmConfig.roll_turn_deg`; the wrist-pitch motor on the upper arm's side, the wrist and the gripper rolled with
  it). `location()` applies it (`turned=False`: the frame before it); `robot/frames.py` turns its points with
  `P.shifted()` and its directions with `P.turned()`; a turned record's bbox in `tests/totals.py` is its placed build's.
  The mounted records hosted on turned ones follow (`mount_placements.py`); a `ModuleMount` resolves its host
  UNTURNED - the roll drive drives the joint the turn is about, so it keeps its pose (its frame, its tests and
  `tests/forearm/helpers.py in_drive_frame()` are in `j2_link`'s frame before the turn) - unless it is `turned`: the
  wrist roll drive, beyond the turned link, turns with `wrist_link#1`. `test_placements.py
  test_the_forearm_turns_about_its_roll_axis` locks it.
- **Mounted occurrences** (`lib/mounts.py MOUNTS`): what the SolidWorks capture never had - the belt joints' motors +
  boards and their drive (20Ts, belts, tensioners), the bearings, the base_yaw thrust stack and 120T, the base's motor
  mount, the yoke's motor leg, the fasteners that hold them - and what it had where the lower 6806 has no room: the
  elbow's and the wrist's 90T (`gt2_pulley_90t#3` / `#4`), hosted on the retired records they correct,
  `PULLEY_SEAT_SHIFT` out along the pulley's own axis. They are declared as frames-as-data in the host occurrence's
  frame (`Mount(key, part, host, link, joint, frame)`; a board's host is its motor, a nut set's its screw set) and
  `tools/reference/mount_placements.py` materialises them into `placements.json` as ordinary part records
  (parent `None`, `rel == world = host world * frame`, solids / volume / bbox from `parts.build`, a `mount` block; keys
  under top-level `mounted`, `P.keys(mounted=True)`), checking each part's `lib/mounts.py AXES` axis against its joint
  axis (Recipe B). The extractor appends them on every run (`extract_placements.py --no-pancake` - the flag keeps this
  machine's bytes out of `vendor/nema17_pancake.step`); `mount_placements.py` alone is the merge mode that needs no
  monolith (change a spin or a frame in `lib/mounts.py`, or a dimension a frame reads, e.g. `J2_MOTOR_X`, which
  follows `WRIST_MOTOR_CENTRES` → run it → re-derive the inertials). `assemblies/arm.py OCCURRENCES` / `GROUPS` and
  `robot/frames.py LINKS` list the keys like any other (roles mostly the joint names: below). `tests/test_mounts.py`
  re-checks the geometry: axis on the joint, mounting face on the host's pad, each bearing stack's contacts
  (`test_bearing_stacks`), the pulley bolts' seats (`test_pulley_bolts_clamp_their_joints`) and reach
  (`test_the_pulley_bolts_reach_their_nuts`), zero interference with the neighbours - the one budget is the nuts'
  designed press in their pockets (`_press`: the pulley bolts' `nut_af`, the base motor mount's
  `JointParams.nut_pocket_af`); the base motor mount's joint is `tests/base/test_motor_mount.py`. The drive's own board
  is a `cycloidal_drive.py` row (`stack_positions["z_mks_board"]`).
- `assemblies/arm.py` / `gripper.py`: `OCCURRENCES = [(part, role|None, key), …]` in SolidWorks
  document order; `assemblies/_occurrences.py` places each occurrence as
  `lib.models.geometry(parts.model(part)).moved(rel * LOCAL_FROM_REF⁻¹)` — inside a build the child
  model's result (`model().shape`: the child runs as the build's child, replayed when unchanged, writes its
  own STEP and hands back a new placement the parent owns), otherwise a fresh in-process copy — and
  `.moved()`s the modules the same way (`MODULES` maps a module name to its **model**; never `.locate()` in
  place). The arm re-tints the leaves of those copies (below).
- **The arm is emitted Z up.** `placements.json` is Y up, but cadgen's viewer and snapshot renderer
  hardcode +Z as up and have no up-axis option (no `@step` argument, URL parameter or flag), so a
  capture-frame `arm.step` renders lying on its side. `arm.py arm_from_w()` =
  `lib/datum.py base_frame()⁻¹` (capture frame → `base_link` frame: Z up, X forward, the base's
  mounting face on z = 0 — the frame `arm.urdf` uses, so both open in the same pose) goes into
  `grouped_children(…, root=)`, which composes `root * rel` into **every occurrence's
  placement**. Never `.moved()` the built root Compound instead: the placements carry the frame and
  the root stays at the identity. The
  modules keep their own module frames (each already has its axis on +Z), and
  `robot/` never reads the arm compound (it goes `placements.json world` → `world_rows`).
  `test_assembly.py` compares the SolidWorks world bbox through `arm_from_w()`.
- Roles label a part with the joint it serves (mostly the joint name: `elbow_pitch_1`, `forearm_roll`), which also keeps
  duplicate labels unique (`test_placements.py test_duplicate_parts_have_unique_labels`): `1` / `2` on the gripper's
  pairs and the cycloidal and forearm roll drives' bearings, `j2` / `j3` on the elbow's / wrist's 90T and
  `j3_coupler#2`.
- A view of the arm with some occurrences hidden needs no model: `./cadtool snapshot assemblies/arm.step out.png
  --display '{"hidden": ["arm/wrist/gripper"]}'` (occurrence-path refs, `./cadtool inspect` lists them; `isolated`
  draws only the ones named). A second model would be the only way to get a STEP (a model takes no parameters).
- **Printed vs. bought is a colour in every assembly, never a second model.** `gripper.py`, `cycloidal_drive.py`,
  `forearm_roll_drive.py` and `wrist_roll_drive.py` declare a `TINT` (their printed parts' colour, reused by `arm.py MODULE_TINTS`) and pass it to
  `occurrence_children(…, tint=)` / `located_children(…, tint=)`; `_tint_parts` gives every purchased part
  (`parts.bought()`) the one `_occurrences.BOUGHT_TINT` grey instead. Tints go on the LEAVES only (`_tint`), never
  on a group or module node: cadgen writes a multi-solid part's colour on its solids, and a colour on an enclosing
  node outranks those in the renderers (verified by snapshot: the bought MKS boards and the MG996R drew in their
  link's colour while the groups carried one). Separate make/buy models were tried and removed as
  duplicates — one STEP per assembly. A purchased item that is **not modelled** lives only in
  `tools/bom.py EXTRAS` — it is on the buy list and absent from the model, the totals and the inertials; model it as a
  COTS pattern part (`cycloidal_housing_bolts` is the pattern, `elbow_pulley_screws` a native mounted one,
  `yaw_pulley_screws` a mounted one with no reference file, the belts (`parts/*/<joint>_belt.py`, a smooth band round
  their loop) the same for a non-pattern part - the state
  for a new one, `parts/AGENTS.md` Part states → *no reference*) to change that.
- `arm.py GROUPS` buckets the occurrences into the component tree
  `arm → base_link/shoulder_link/upper_arm_link/elbow_link/forearm_link/wrist_pitch_link/wrist` — the
  `robot/frames.py LINKS` partition with the modules (`arm.py MODULES`) kept whole (`wrist` = wrist_roll_link + jaw
  links; the cycloidal drive under `shoulder_link` although `LINKS` puts its rotor body in
  `upper_arm_link`; the forearm roll drive under `elbow_link` although its rotor is in `forearm_link`; the wrist roll
  drive under `wrist_pitch_link` although its rotor is in `wrist_roll_link`) — via
  `_occurrences.grouped_children()`, which tints each subtree's **printed** parts with its group's color
  (`MODULE_TINTS` overrides for the modules), every **purchased** part — inside the modules too — with `BOUGHT_TINT`
  (above; no group / module tint may reuse it, `test_grey_means_bought`), and raises unless the groups cover the keys
  exactly once — so each component toggles as one node in the viewers. `test_assembly.py` locks the group labels +
  the LINKS mirror. The tints are per-leaf (`_tint` colours every leaf, no group node: above).
- `assemblies/cycloidal_drive.py` is **code-driven**: rows are `(part, role, placement)` (data) from
  `lib/cycloidal stack_positions` (`located_children`) — the placement a position `(x, y, z)` when the part's
  frame is the module's shifted, or a frame `((x, y, z), (rx, ry, rz))` when it needs a rotation
  (`_occurrences.row_location`, used by `located_children` and `world_rows` alike); its placement key
  `cycloidal_drive#1` is a `designed` module record in `placements.json` (pose from the SolidWorks node, no
  leaf records, `solidworks` cross-check block; `tools/reference/extract_placements.py` never descends into
  `DESIGNED_MODULES`). A designed module the capture never placed declares its pose in `lib/mounts.py`
  `MODULE_MOUNTS` (`ModuleMount(key, module, host, joint, frame)`: module +Z ON the joint's axis, checked) and
  `mount_placements.py` writes its record (`kind: module, designed: true`, a `mount` block, no totals — listed
  under `designed_modules` AND `mounted`) — `assemblies/forearm_roll_drive.py` is that case (rows from
  `lib/forearm stack_positions`, key `forearm_roll_drive#1`), and `assemblies/wrist_roll_drive.py` (rows from
  `lib/wrist stack_positions`, key `wrist_roll_drive#1`; its frame is its host, `wrist_link`, not a row). `world_rows(key)` expands a designed-module key into
  world-placed parts for links and inertials; `BODIES` names the drive's rigid bodies (`stator` / `rotor`) and a
  `:<body>` key suffix (`"cycloidal_drive#1:rotor"`, `_occurrences.split_key`) selects one. Keep `EXPECTED`
  (whole module + `bodies`) in step with the geometry (`totals()` / `totals(body)`).
- **Assemblies are native build123d**: a model body returns `lib.assembly.assembly(name, children)` =
  `Compound(label=…, children=[…])`, and the `_occurrences` helpers (`occurrence_children`,
  `grouped_children`, `located_children`) RETURN the placed children, each labelled with `lib.assembly`'s
  `label_shape` (`gt2_pulley_90t:j3`: `label_text`'s compact form).
- When adding source-level joints, they are build123d joints in the model body (a `bd.RevoluteJoint` on the
  parent's part, a `bd.RigidJoint` of the same label on the child's) and `cadgen.ap242` mechanism states (viewer
  sliders, posed snapshots; the plugin's `skills/cad/references/kinematics.md`) — not helper frames; keep
  placements parameter-driven, and validate with `cadgen.geometry.closest_points` / `overlap_volume` on the
  bodies' shapes (`parts.build`, `lib.models.raw`).
- A new assembly model goes in `tests/test_lazy_kernel.py`'s module list.
- `.moved()`/`.located()` deep-copy the shape **and its parent chain**; never call them on a child
  of a big imported assembly (copy its `solids()` instead). A shape added to two Compounds is
  silently re-parented — `_occurrences` builds a fresh body per occurrence outside a build (inside
  one, each call to a child model hands back a new placement of its result).

**Recipe B — add a mounted occurrence** (something SolidWorks never placed, or a SolidWorks pose the design corrects -
retire the record (`lib/placements.py RETIRED`, above), host the mount on it): `lib/mounts.py` `Mount(key, part, host,
link, joint, frame)` in the tuple of its kind in `MOUNTS` — the frame as data in the HOST's frame, the part's `AXES`
axis parallel to the joint axis, ON it where `AXES` says so (a part with no `AXES` row gets no axis check) →
`./cadtool python tools/reference/mount_placements.py` (writes the `placements.json` record, checks the axis) → the key
into `assemblies/arm.py OCCURRENCES` + `GROUPS` and `robot/frames.py LINKS` → a case in `tests/test_mounts.py` (face on
the host, interference budget) → Recipe C.
