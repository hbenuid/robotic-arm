# assemblies/ — occurrences, placements, modules

Loads when you work in `assemblies/`. `arm.py` (the whole arm), `gripper.py` (the gripper module, placed from
`placements.json`), `cycloidal_drive.py` / `forearm_roll_drive.py` (code-driven modules), `_occurrences.py` (the
placing / tinting helpers, also used by `robot/` and `tests/totals.py`). Every output STEP is git-ignored.

## Assembly & placements
- `reference/placements.json` (from `tools/reference/extract_placements.py`; schema: `reference/README.md`) holds
  every occurrence: `rel` (to its parent node) and `world`, as `Location(position, rotation_xyz_deg)`; keys
  `"<part>#<n>"`, module `"gripper#1"`. Treat it as an immutable input. An occurrence the DESIGN has replaced is
  **retired** in `lib/placements.py RETIRED` (`j3_coupler#1`: the roll drive's block took over its
  features, `docs/forearm_roll.md`; `gt2_pulley_90t#1` / `#2`: re-seated 3 mm out as mounts, below): its record stays, `P.keys()` leaves it out (`retired=True` lists the file), no table / link / total
  claims it (`test_placements.py`, `test_assembly.py`, `test_robot.py` follow `keys()`). A PART the design drops
  entirely (the link caps `j1_cap` / `j2_cap_1` / `j2_cap_2`) cannot be retired - its part module,
  `CUSTOM` row and manifest entry go, and a record must name a known part -: its SolidWorks product goes into
  `lib/reference.py SKIPPED_PRODUCTS` and `tools/reference/mount_placements.py` (the merge mode, no monolith) moves the
  record to `skipped` as the entry an extraction writes (pose, totals, reason) - `test_skipped_nodes_are_the_dropped_products`.
- **Mounted occurrences** (`lib/mounts.py`): the belt joints' motors - `nema17_48mm#1` (the 48 mm motor, under the base;
  motor + board hang `BASE_MOTOR_STACK_PROUD` below the base's bottom face) and `nema17_40mm#2..3`, + `mks_servo42d#1..3`,
  on the NEMA 17 pads `base` / `j1_link` / `j2_link` carry - and each belt joint's 6806-2RS pair (`bearing_6806#1..6`, on
  the lip of its housing's bore) and the base_yaw thrust bearing (`washer_as6590#1`, `bearing_axk6590#1` in the base's groove,
  `washer_as6590#2` under `j1_coupler`'s seat) never existed in the SolidWorks capture; the elbow's and the wrist's 90T
  (`gt2_pulley_90t#3` / `#4`) did, but where the lower bearing had no room - their mounts are hosted on the retired
  records they correct, `PULLEY_SEAT_SHIFT` out along the pulley's own axis; each 90T's 4x M4 screws + nuts
  (`elbow_pulley_screws#1` / `elbow_pulley_nuts#1`, `wrist_pulley_*#1` - COTS pattern parts) are hosted on the pulley and
  on the screws, centred on the joint axis. They are declared as
  frames-as-data in the host occurrence's frame (`Mount(key, part, host, link, joint, frame)`; a board's host is its
  motor, a nut set's its screw set) and `tools/reference/mount_placements.py` materialises them into `placements.json` as ordinary part records
  (parent `None`, `rel == world = host world * frame`, solids / volume / bbox from `parts.build`, a `mount` block; keys
  under top-level `mounted`, `P.keys(mounted=True)`), checking each motor's +Z against its joint axis (a bearing's +Z, a pulley's +Y, a pulley-bolt pattern's +Z ON it - `lib/mounts.py AXES`). The extractor
  appends them on every run (`extract_placements.py --no-pancake` - the flag keeps this machine's bytes out of
  `vendor/nema17_pancake.step`); `mount_placements.py` alone is the merge mode that needs no monolith (change a spin or
  `J2_MOTOR_SLIDE_X` in `lib/params.py` → run it → re-derive the inertials). `assemblies/arm.py OCCURRENCES` /
  `GROUPS` and `robot/frames.py LINKS` list the keys like any other (roles = the joint names). `tests/test_mounts.py`
  re-checks the geometry: axis on the joint, mounting face on the host's pad, each bearing stack's contacts
  (`test_bearing_stacks`), the pulley bolts' seats and reach (`test_pulley_bolts_clamp_their_joints`), zero interference
  with the neighbours - the one budget is the nuts' designed press in their `nut_af` pockets (`_press`). The drive's own board is a `cycloidal_drive.py` row
  (`stack_positions["z_mks_board"]`).
- `assemblies/arm.py` / `gripper.py`: `OCCURRENCES = [(part, role|None, key), …]` in SolidWorks
  document order; `assemblies/_occurrences.py` places each occurrence as
  `lib.models.geometry(parts.model(part)).moved(rel * LOCAL_FROM_REF⁻¹)` — inside a build the linked
  child (built in parallel; the gripper's STEP links the part's tree), otherwise a fresh in-process
  copy — and `.moved()`s the modules the same way (`MODULES` maps a module name to its **model**;
  `.locate()` in place is gone: it would force and mutate a linked child). The tinted arm uses
  `geometry(model, inline=True)`: the child is still called (pinned, rebuilt, its STEP rewritten) but
  the arm owns a recoloured copy — cadgen's packager keeps a linked child's own colours, so tints
  through links are lost (verified by snapshot).
- **The arm is emitted Z up.** `placements.json` is Y up, but cadgen's viewer and snapshot renderer
  hardcode +Z as up and have no up-axis option (no `@step` kwarg, sidecar field, URL parameter or
  flag), so a capture-frame `arm.step` renders lying on its side. `arm.py arm_from_w()` =
  `lib/datum.py base_frame()⁻¹` (capture frame → `base_link` frame: Z up, X forward, the base's
  mounting face on z = 0 — the frame `arm.urdf` uses, so both open in the same pose) goes into
  `grouped_children(…, root=)`, which composes `root * rel` into **every occurrence's
  placement**. Never `.moved()` the built root Compound instead: cadgen's STEP packager reads only the
  children's locations, so the in-process shape would rotate and the written STEP would not. The
  gripper and the drive keep their own module frames (both already have their axis on +Z), and
  `robot/` never reads the arm compound (it goes `placements.json world` → `world_rows`).
  `test_assembly.py` compares the SolidWorks world bbox through `arm_from_w()`.
- Roles (`j2`/`j3` = the elbow_pitch / wrist_pitch pulley + coupler pairs, `1`/`2`) only disambiguate
  duplicates; renaming them after the joints is a follow-up.
- A view of the arm with some occurrences hidden needs no model: `./cadtool snapshot assemblies/arm.step out.png
  --hide '#<label>'` (label refs; STEP input only, not with `--render` / `--focus`; the viewer has no `?hide=`
  parameter). A second model would be the only way to get a STEP (a model takes no parameters, the freshness gate
  sees no environment variable).
- **Printed vs. bought is a colour in every assembly, never a second model.** `gripper.py`, `cycloidal_drive.py` and
  `forearm_roll_drive.py` declare a `TINT` (their printed parts' colour, reused by `arm.py MODULE_TINTS`) and pass it to
  `occurrence_children(…, tint=)` / `located_children(…, tint=)`; `_tint_parts` gives every purchased part
  (`parts.bought()`) the one `_occurrences.BOUGHT_TINT` grey instead. Verified by snapshot: a tint set on a
  module's DIRECT linked children does reach its STEP, so the modules stay linked (the arm's inline copies predate
  that finding and were left alone). Separate make/buy models were tried and removed as duplicates — one STEP per
  assembly. A purchased item that is **not modelled** (the
  drive's arm-mount bolts + captive nuts, the belts, the roll drive's self-tapping and motor M3 screws) lives only in
  `tools/bom.py EXTRAS` — it is on the buy list and absent from the model, the totals and the inertials; model it as a
  COTS pattern part (`cycloidal_housing_bolts` is the pattern, `elbow_pulley_screws` a native mounted one,
  `forearm_roll_mount_screws` a native module row) to change that.
- `arm.py GROUPS` buckets the occurrences into the component tree
  `arm → base_link/shoulder_link/upper_arm_link/elbow_link/forearm_link/wrist_pitch_link/wrist` — the
  `robot/frames.py LINKS` partition with the three modules kept whole (`wrist` = wrist_roll_link + jaw
  links; the cycloidal drive under `shoulder_link` although `LINKS` puts its rotor body in
  `upper_arm_link`; the forearm roll drive under `elbow_link` although its rotor is in `forearm_link`) — via
  `_occurrences.grouped_children()`, which tints each subtree's **printed** parts with its group's color
  (`MODULE_TINTS` overrides for the modules), every **purchased** part — inside the modules too — with the one
  `_occurrences.BOUGHT_TINT` grey (`_tint_parts`: grey always means bought, so no group / module tint may reuse it;
  `test_grey_means_bought`), and raises unless the groups cover the keys
  exactly once — so each component toggles as one node in the viewers. `test_assembly.py` locks the group labels +
  the LINKS mirror. The tints are per-leaf (a compound-level color did not cascade in the OCP CAD Viewer) — hence the
  inline copies above.
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
  `lib/forearm stack_positions`, key `forearm_roll_drive#1`). `world_rows(key)` expands a designed-module key into
  world-placed parts for links and inertials; `BODIES` names the drive's rigid bodies (`stator` / `rotor`) and a
  `:<body>` key suffix (`"cycloidal_drive#1:rotor"`, `_occurrences.split_key`) selects one. Keep `EXPECTED`
  (whole module + `bodies`) in step with the geometry (`totals()` / `totals(body)`).
- **Assemblies are native build123d**: a model body returns `lib.assembly.assembly(name, children)` =
  `Compound(label=…, children=[…])`, and the `_occurrences` helpers (`occurrence_children`,
  `grouped_children`, `located_children`) RETURN the placed children, each labelled with cadgen's
  `label_shape` (`j3_coupler:j2`; `label_shape` / `label_text` are not deprecated). cadgen 0.6.5 deprecated
  its `AssemblyHelper` wrapper (a `FutureWarning` per build) — never reintroduce it.
- When adding source-level joints, declare them as data — `@step(kinematics={"mates": [cadgen.revolute(name,
  parent="#label", child="#label", …)]})` (viewer sliders, posed snapshots; the plugin's
  `skills/cad/references/kinematics.md`) — not as helper frames; keep placements
  parameter-driven, and validate with `cadgen.geometry.closest_points` / `overlap_volume` on
  `read_scene(…).resolve(ref).shape()`.
- A new assembly model goes in `tests/test_lazy_kernel.py`'s module list.
- `.moved()`/`.located()` deep-copy the shape **and its parent chain**; never call them on a child
  of a big imported assembly (copy its `solids()` instead). A shape added to two Compounds is
  silently re-parented — `_occurrences` builds a fresh body per occurrence outside a build (inside
  one, each call to a child model is its own linked occurrence).

**Recipe B — add a mounted occurrence** (something SolidWorks never placed, or a SolidWorks pose the design corrects -
retire the record, host the mount on it): `lib/mounts.py` `Mount(key, part, host,
link, joint, frame)` — the frame as data in the HOST's frame, +Z on the joint axis → `./cadtool python
tools/reference/mount_placements.py` (writes the `placements.json` record, checks the axis) → the key into
`assemblies/arm.py OCCURRENCES` + `GROUPS` and `robot/frames.py LINKS` → a case in `tests/test_mounts.py` (face on the
host, interference budget) → Recipe C.
