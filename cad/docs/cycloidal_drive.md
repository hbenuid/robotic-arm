# Cycloidal drive — 20:1 shoulder-pitch actuator

**Purpose:** the specification of the drive (carried over from the `cycloidal_drive` repo, corrected
where the code disagreed with it), where it lives in `cad/`, what changed in the build123d port, and
how it is attached to the arm.

## Provenance
- Designed in CadQuery in [`hbenuid/cycloidal_drive`](https://github.com/hbenuid/cycloidal_drive);
  imported at `2f1f67d` (117 commits) by a subtree merge into `cad/cycloidal_import/` (`6739509`),
  ported to build123d (`f670d20` printed parts, `7c4b9aa` purchased parts, `8b89e20` assembly,
  robot description in the following commit) and the import directory removed once the port was
  complete — `git log` keeps every original commit. The old repo is untouched.
- **Reference geometry** = the CadQuery builders' own STEP exports at that revision, produced by
  `tools/cycloidal/export_cadquery.py` (run inside the old repo's venv) and copied by
  `tools/cycloidal/import_cadquery.py` into `reference/cycloidal/<name>.step` (manifest kind `designed` for
  the 6 printed parts, `cots` for the 10 purchased ones, `origin: cycloidal_drive@2f1f67d`).
  `tests/cycloidal/test_port.py` proves the port reproduces them: identical face sets and
  tessellations, and the exact analytic volume for every part but the two spline discs (§11).

## Overview

| Parameter | Value |
|---|---|
| Application | The arm's `shoulder_pitch` joint (~400 mm reach) — between `j1_coupler` and `j1_link` (§12) |
| Type | Two-disc cycloidal drive, discs 180° apart |
| Gear ratio | 20:1 (20 lobes, 21 ring pins) — `lib/params.py CYCLOIDAL_RATIO` |
| Motor | NEMA 17, 48 mm body, 22 mm × Ø5 D-shaft (`parts/cycloidal/nema17_48mm.py`; the shown geometry is `vendor/nema17_48mm.step`, the real body from the user's kit export carrying this motor's own pilot + shaft, `lib/cycloidal/motor.py`) + its MKS SERVO42D board kit on the rear face (`parts/joints/mks_servo42d.py`, `z_mks_board` = −48) |
| Eccentricity | 1.5 mm |
| Housing OD | 140 mm (`CYCLOIDAL_HOUSING_OD`) |
| Print material | PETG, 100 % infill on the discs |

The drive's own dimensions live in **`lib/cycloidal/params.py`** (`DriveConfig`, ten frozen groups:
`gear, disc, shaft, bearings, housing, motor, output_hub, tolerances, profile, stack_up`); every
number below names its field.

## 1. Drive geometry

### 1.1 Ring gear (`cfg.gear`)

| Parameter | Value | Notes |
|---|---|---|
| Ring pins | 21 (`num_ring_pins`) | N + 1, N = 20 lobes |
| Ring pin Ø | 4.00 (`ring_pin_dia`) | h6 ground steel dowel |
| Ring pin circle Ø | 108.00 (`ring_pin_circle_dia`) | centred in the housing bore |
| Ring pin length | 35 (`ring_pin_length`) | 3.5 in the motor plate + 28 open bore zone + 3.5 into the bearing-zone wall; the plate's 9 mm through-holes leave 5.5 mm empty at the outer face |
| Housing bore Ø | 116.00 (`housing.bore_dia`) | |

### 1.2 Cycloidal disc (`cfg.disc`, `parts/cycloidal/cycloidal_disc_1.py`, `_2.py`)

| Parameter | Value | Notes |
|---|---|---|
| Lobes | 20 (`gear.num_lobes`) | sets the ratio |
| Disc count | 2 | orbit centres 180° apart cancel vibration. Disc 2's epitrochoid is phase-rotated by `gear.disc2_phase_deg = -180°/N_lobes = -9°`; the output-pin holes are identical (disc-local 0/90/180/270 on the 60 mm circle). **The discs are distinct printed parts** — a 180° assembly rotation is a no-op on a 20-lobe disc. |
| OD | ~108 (105.9 / 107.0 bbox) | epitrochoid profile (§8) |
| Centre bore Ø | 35.10 (`center_bore_dia`) | 35 mm 6003 OD + 0.10 clearance |
| Thickness | 10.00 (`thickness`) | = 6003 width |
| Lobe chamfer | 1.00 × 45° both faces (`lobe_chamfer`) | assembly lead-in, hides elephant foot; symmetric (no wrong side); costs ~20 % lobe contact length |
| Inter-disc spacer | 2.00 (`inter_disc_spacer`) | |
| Output pin holes | 4 × Ø7.4 on Ø60 | §1.3 |

### 1.3 Output stage (`cfg.disc`, `cfg.output_hub`)

| Parameter | Value | Notes |
|---|---|---|
| Output pins | 4 at 90° (`output_pin_count`) | 4 × 45 mm h6 dowels (`output_pin_dia`, `output_pin_length`) on Ø60 (`output_pin_circle_dia`) |
| Disc pin hole Ø | 7.40 (`output_pin_hole_dia`) | 4 + 2 × 1.5 ecc + 0.4 clearance |
| Hub pin holes | Ø4.20 × 19 blind (`ring_pin_hole_dia`, grip − `output_hub_pin_ceiling`) | greased sliding fit through the discs; captured between the closed hub ceiling and the motor plate |

Clearances: pin-hole inner edge 26.30 vs bore radius 17.55 → **8.75 mm wall**; pin-hole outer edge
33.70 vs lobe valley ~49 → **~15 mm wall**.

**Backlash:** the ring-pin mesh is a zero-clearance theoretical epitrochoid, so the output-pin holes are
the dominant designed source: slack = hole_r − pin_r − e = 3.70 − 2.00 − 1.50 = **0.20 mm radial**, i.e.
**≈ ±0.38° at the 30 mm pin circle** (was ±0.95° at Ø8.0, ±0.57° at Ø7.6). **7.4 mm is the practical
floor:** below ~7.2 the four rigid pins over-constrain against printed-hole position error across two
prints and the disc binds after shrink. Going lower means attacking the secondary sources (4.20 hub
fit, 6003 radial clearance, the 35.10 bore) or bushings/rollers on the pins — a redesign. Guarded by
`test_output_hole_backlash_budget` (`tests/cycloidal/test_disc.py`).

### 1.4 Eccentric shaft (`cfg.shaft`, `parts/cycloidal/cycloidal_eccentric_shaft.py`)

| Parameter | Value | Notes |
|---|---|---|
| Bearing-seat OD | 17.10 (`bearing_seat_od`) | light press in the 6003 bore |
| Eccentricity | 1.50 | lobe 1 at (+e, 0), lobe 2 at (−e, 0) |
| Spine OD | 5.00 (`spine_od`) | |
| Input collar OD | 10.00 (`input_collar_od`) | wall around the D-bore |
| D-bore Ø | 5.13 (`d_bore_dia` + 2 × `tolerances.d_bore_clearance_add`) | receives the motor shaft directly, flat on +Y (`d_bore_flat` 4.5) |
| D-bore depth | 14.00 (`d_bore_depth`) | 13 mm engagement (22 shaft − 9 plate) + 1 mm clearance |
| Support-pin hole | Ø5.15 × 11 blind (`support_pin_dia` + 2 × `dowel_bore_clearance_add`, `support_pin_hole_depth`) | keeps a 1 mm wall to the D-bore |
| Support pin | 5 × 20 h6 dowel | 11 in the shaft + 2 gap + 5 in the 625 + 2 proud |
| Bridge / retention flange | Ø23.10 (`bridge_flange_od` = seat OD + `bridge_flange_add` 6) | ruled loft (+e → −e) across the spacer zone; keeps the 6003s apart |
| Material | PETG, 100 % infill, 0.16 mm layers | built at its stack position z 9..35 |

## 2. Bearings (`cfg.bearings`)

| Bearing | Qty | d × D × B | Part | Role |
|---|---|---|---|---|
| 6003-2RS | 2 | 17 × 35 × 10 | `parts/cycloidal/bearing_6003.py` | one per disc, on the shaft lobes (~5.4 kN dyn.) |
| 6814-2RS | 2 | 70 × 90 × 10 | `parts/cycloidal/bearing_6814.py` | end-to-end output support in the ring gear body seat, inner races on the hub (~7.0 kN dyn.) |
| 625-2RS | 1 | 5 × 16 × 5 | `parts/cycloidal/bearing_625.py` | in the hub's inner-face pocket, on the shaft support dowel (~1.0 kN dyn.) |

The output bearings (z 37..57) and the discs (z 13..35) are axially separated; the hub pin holes
(r 27.9..32.1) sit well inside the 70 mm inner-race bore.

## 3. Non-bearing purchased parts

| Part | Spec | Qty | Module |
|---|---|---|---|
| Motor | NEMA 17, 42.3² × 48 body, Ø5 shaft 22 long (4 round + 18 D-cut, flat 4.5), Ø22 × 2 pilot, 4 × M3 on 31 mm square tapped 4.5, ~0.45 Nm | 1 | `nema17_48mm` |
| Ring pins | 4 × 35 h6 hardened dowel (buy 25) | 21 | `cycloidal_ring_pins` |
| Output pins | 4 × 45 h6 hardened dowel; light press in the hub's 4.20 blind holes (printed ~4.00–4.10), hub ceiling / motor plate as backup capture, free through the discs' 7.4 holes | 4 | `cycloidal_output_pins` |
| Shaft support pin | 5 × 20 h6 dowel | 1 | `cycloidal_shaft_support_pin` |
| Housing bolts | M4 × 55 SHCS (ISO 4762), Ø7 × 4 head; counterbored in the motor plate, captive nut in the ring gear body | 8 | `cycloidal_housing_bolts` |
| Housing nuts | M4 hex, 7.0 AF × 3.2 (pocket 7.2 AF) | 8 | `cycloidal_housing_nuts` |
| Motor bolts | M3 × 10 SHCS, Ø5.3 × 3 head (13 total): 6 through the plate + 4 engagement, head flush in the 3 mm inner-face pocket | 4 | `cycloidal_motor_bolts` |
| Arm-mount bolts + nuts | 4 × M4 (≈40–50 long: link + 28 hub) into captive M4 nuts on the hub inner face — install the nuts before pressing the hub through the 6814s | 4 + 4 | not modelled — on the buy list through `tools/bom.py EXTRAS` |

The purchased parts are modelled as the drive repo's simplified solids (annuli, cylinders, hex
prisms): each module's `_envelope()`, which is also its reference STEP. `vendor/bearing_625.step`
is a step.parts catalog model (`bearing_625_2rs_sealed_simple`); see `vendor/README.md` for why the
6003, 6814 and NEMA 17 catalog models were not adopted.

## 4. Axial stack-up (`cfg.stack_up`; z = 0 at the motor-plate outer face, +Z inward)

| Layer | Thickness | Running total |
|---|---|---|
| Motor plate outer wall (`motor_plate_wall`) | 5 | 5 |
| Motor plate inner wall (`motor_plate_inner_wall`) | 4 | 9 = `z_motor_plate_inner` |
| Input clearance / D-shaft collar (`input_clearance`) | 4 | 13 = `z_disc1` |
| Disc 1 + 6003 (`disc_thickness`) | 10 | 23 |
| Inter-disc spacer (`inter_disc_spacer`) | 2 | 25 = `z_disc2` |
| Disc 2 + 6003 | 10 | 35 |
| Clearance to the output bearings (`output_clearance`) | 2 | 37 = `z_output_bearings` |
| 2 × 6814 (`output_bearing_total`) | 20 | 57 = `z_bearing_top` |
| Output wall: retention lip + nut pockets (`output_wall`) | 3 | 60 = `total_housing_depth` |

Derived: `disc_zone` 26, `bore_zone` 28 (the 116 mm bore length), `ring_gear_body_height` 51.
**Hub protrusion:** the hub is 28 tall (grip 20 + wall 3 + `proud_above_housing` 5) at z 37, so its
arm-mount face sits at **z = 65**, 5 mm proud of the housing (`CYCLOIDAL_OUTPUT_FACE_Z`). **Housing
depth follows the bolt:** M4 × 55 in the 4.5 counterbore ends at 59.5 — inside the 60 mm depth with full
nut engagement (`test_bolt_does_not_protrude`); M4 × 50 would not reach the nut, so 60 is the floor.
**Total envelope with the motor:** 60 + 48 = 108 (the pilot recesses 2 mm into the plate); the hub adds 5, the MKS board kit behind the motor 14.1 (module z −62.1 … 65 = 127.1).

The module layout (`lib/cycloidal/layout.py stack_positions`, the drive repo's `assembly.py`
numbers) places: discs at (±1.5, 0, 13 / 25) with their 6003s, 6814s at 37 / 47, hub and 625 at 37,
ring pins at 5.5, output pins at 11, support pin at 24, motor bolts at −5, housing bolts at 0.5,
nuts at 56; shaft, motor and motor plate at 0 (built in place).

## 5. Housing design notes (`cfg.housing`)

### 5.1 Envelope — OD 140, depth 60, bore Ø116, 6814 seat Ø90.15 × 20, bolt circle Ø125 (8 × M4).

### 5.2 Housing split — two printed parts (the former output cap is folded into the ring gear body)

1. **Motor plate** (`parts/cycloidal/cycloidal_motor_plate.py`, z 0..9): NEMA 17 pattern with a Ø22.30 × 2 pilot
   recess, Ø15 shaft pass-through (`motor_plate_shaft_bore`), 21 ring-pin through-holes Ø4.20, 8 × M4
   holes with Ø7.4 × 4.5 counterbores on the outer face, M3 heads flush in 3 mm inner-face pockets.
2. **Ring gear body** (`parts/cycloidal/cycloidal_ring_gear_body.py`, local z 0..51 at stack 9..60): 21 blind
   ring-pin holes Ø4.20 × 31.5 with 1 mm entry funnels at the bore/bearing transition (Ø5.2 → Ø4.2,
   `ring_pin_entry_chamfer_*`), stepped bore Ø116 (0..28) / Ø90.15 seat (28..48) / **Ø86.15 integral
   retention lip** (48..51, `lip_radial` 2 over the 90 mm outer races — bearings insert from the input
   side and seat against it), 8 × M4 through-holes, **8 captive hex nut pockets** (7.2 AF × 4) on the
   output face, each backed by a full-height bolt pillar.

**Shared outer profile** — both parts carry the same 8-pillar / 8-window silhouette around the bolt
circle (pillars `pillar_inner_w` 18 at the bore, `pillar_outer_w` 10 at the OD, one per bolt); the
plate seats on the body's pillar faces, no continuous rim. One shared cutter
(`lib/cycloidal/housing.py reveal_window_cutter`) is subtracted from every housing base solid.

**Outer-edge chamfer** (`edge_chamfer` 1.5, `chamfer_outer_silhouette`): the 8 pillar outer vertical
corners always, plus the *entire* outer-wire perimeter of each part's external end face (the plate's
motor face z = 0, the body's output face z = 51); the mating faces (plate z = 9, body z = 0) stay
sharp so the stack beds flush. Internal holes are never beveled. `edge_chamfer = 0` disables; keep
≤ 2 to preserve the nut-pocket-to-OD wall.

### 5.3 Output hub (`parts/cycloidal/cycloidal_output_hub.py`, `cfg.output_hub`)

PETG (aluminium viable): Ø70.3 × 28 (`od` = 70 mm 6814 bore + 0.3 interference grip); 4 blind
Ø4.20 × 19 pin holes on Ø60 closed by a 1 mm ceiling; Ø6 shaft-clearance bore over the 20 mm grip zone
only; Ø16.2 × 5 625 pocket on the inner face; **arm-link mount** 4 × Ø4.4 through-holes on Ø50 at 45°
from the pins, each into a captive M4 nut pocket (7.2 AF × 4) on the inner face; a Ø36 × ~7 lightening
recess in the proud face (`arm_mount_pocket_dia`, 0 = sealed) — the arm link bears on the r 25..35
annulus. The proud section passes the lip bore with ~7.9 mm radial clearance. Print output-face-down.

## 6. PETG print tolerances (`cfg.tolerances`)

PETG holes print 0.10–0.20 undersized (shrink, elephant foot, arc overshoot); the positive offsets
are print compensation, so **everything ends up a press fit on a real print** — the offset sets how
tight. Never design a hole smaller than its steel part.

| Fit | Adjustment | Field | Application |
|---|---|---|---|
| Bearing outer race → housing | +0.15 / +0.20 on the bore | `output_bearing_seat_dia` 90.15, `bearing_seat_bore_add` 0.2 | 6814 seat (firm press), 625 pocket 16.2 (slip-to-light press) |
| Ring & output pin holes | +0.20 → Ø4.20 | `ring_pin_press_sub` −0.20 | slip-press as printed, through (plate) and blind (body, hub) |
| Disc centre bore | +0.10 → 35.10 | `center_bore_dia` | 6003 OD |
| Disc output-pin holes | nominal 7.4, no offset | `output_pin_hole_dia` | shrink *reduces* backlash (§1.3) |
| Motor-shaft D-bore | +0.065 on the radius → Ø5.13 | `d_bore_clearance_add` | slip-to-light press |
| Support-pin bore | +0.075 on the radius → Ø5.15 | `dowel_bore_clearance_add` | firm press |
| Mating surfaces | +0.15 | `mating_surface_add` | pilot recess 22.30 |
| Bolt / head clearance | +0.4 on the Ø | `bolt_clearance_add` | M3 3.4 / 5.7, M4 4.4 |

Print the discs flat at 100 % infill; ≤ 0.16 mm layers for bearing seats; test-print a bearing fit
gauge first; PETG shrinks 0.3–0.5 % over 140 mm (the housing may need ~140.5 — not compensated).

## 7. Performance estimates

Motor 0.45 Nm × 20 = 9.0 Nm theoretical; 55–65 % efficiency (printed, no pin bearings) →
**5.0–5.9 Nm practical**, ~1.3–1.5 kg at 400 mm including the arm; 200–500 rpm in → 10–25 rpm out;
not backdrivable; backlash ≈ ±0.38° (§1.3). **Torque budget warning:** marginal for a 3-DOF arm at
400 mm reach — a lightweight demonstrator; NEMA 23 or a higher ratio for heavier payloads.

## 8. Disc profile (`lib/cycloidal/profiles.py`)

```
x(θ) =  R·cos θ − r·cos(θ + ψ) − e·cos(N·θ)
y(θ) = −R·sin θ + r·sin(θ + ψ) + e·sin(N·θ)
ψ    = atan2(sin((1 − N)·θ), R/(e·N) − cos((1 − N)·θ))
R = 54 (ring-pin circle radius), r = 2 (pin radius), N = 21 (pins), e = 1.5;  lobes = N − 1 = 20
```
2000 points per revolution (`profile.num_points`, `endpoint=False`) through a **periodic
interpolating** B-spline (`Edge.make_spline(..., periodic=True)` — never an approximating spline).

## 9. Shopping list

Generated, never retyped: `./cadtool python tools/bom.py --module cycloidal_drive` prints what to order
(each purchased module's `PURCHASE_SPEC` × pieces, from `assemblies/cycloidal_drive.py OCCURRENCES`) and,
under "not modelled", the items that have no geometry (`tools/bom.py EXTRAS`: the arm-mount bolts and nuts,
grease). `cycloidal_drive.step` shows the same split: purchased parts grey, printed parts the drive's `TINT`.

Budget (2f1f67d estimate): NEMA 17 48 mm $10–15 (as the MKS SERVO42D closed-loop kit with the board: ~$30–40) · 6003-2RS ×2 $4–8 · 6814-2RS ×2 $16–40 · 625-2RS $1–2 ·
ring-pin dowels (pack of 25) $8–12 · output-pin dowels $2–4 · motor bolts $1–2 · housing bolts $2–4 · M4 nuts
(housing + arm mount) $2 · support dowel $0.5–1 · arm-mount bolts $1–2 — **~$47–87**.

## 10. Where things live in `cad/`

| What | Where |
|---|---|
| Dimensions | `lib/cycloidal/params.py` (`DriveConfig`, `DEFAULT_CONFIG`); interface values re-exported as `lib/params.py CYCLOIDAL_*` (+ `STEEL_DENSITY`, bearing / motor / fastener masses) with locks in `tests/test_params_invariants.py` |
| Derived numbers | `lib/cycloidal/layout.py` — hole patterns, `hex_circumdiameter`, `ring_pin_engagement`, `motor_bolt_counterbore_depth`, `hub_height`, `stack_positions` |
| Profile maths | `lib/cycloidal/profiles.py` (numpy) |
| Shared builders | `lib/cycloidal/housing.py` (reveal-window cutter, outer-silhouette chamfer, hex prisms), `lib/cycloidal/disc.py` (`build_disc`), `lib/cycloidal/geom.py` (cylinders with `NUDGE` overshoot, `single_solid`) |
| Printed parts (designed, `CONVERTED = True`) | `parts/cycloidal/`: `cycloidal_disc_1.py`, `cycloidal_disc_2.py`, `cycloidal_eccentric_shaft.py`, `cycloidal_motor_plate.py`, `cycloidal_ring_gear_body.py`, `cycloidal_output_hub.py` — each exposes `build(cfg)` for tests and its `@step` model |
| Purchased parts (COTS) | `parts/cycloidal/`: `bearing_6003.py`, `bearing_6814.py`, `bearing_625.py`, `nema17_48mm.py` (+ the board kit `parts/joints/mks_servo42d.py`, shared with the belt joints' motors), `cycloidal_ring_pins.py` (21), `cycloidal_output_pins.py` (4), `cycloidal_shaft_support_pin.py`, `cycloidal_motor_bolts.py` (4), `cycloidal_housing_bolts.py` (8), `cycloidal_housing_nuts.py` (8) — shared body `parts/cycloidal/_cots.py`; the multi-body ones are registered in `MULTI_BODY`; each says what to order (`PURCHASE_SPEC` / `PURCHASE_QTY`, built from `DEFAULT_CONFIG`) |
| Assembly | `assemblies/cycloidal_drive.py` — one row `(part, role, position)` per piece from `stack_positions` (the MKS board at `z_mks_board`); `EXPECTED` locks the leaves / solids / volume, whole and per body (`EXPECTED["bodies"]`); `./cadtool python -c "from assemblies.cycloidal_drive import totals; print(totals())"` |
| Printed vs. bought | `./cadtool python tools/bom.py --module cycloidal_drive` (print list, buy list, the purchased items not modelled — `EXTRAS`); in `cycloidal_drive.step` (and in the arm) purchased parts are `_occurrences.BOUGHT_TINT` grey, printed parts `cycloidal_drive.TINT`; `./cadtool python tools/export_printables.py` → `print/<name>.stl` |
| References | `reference/cycloidal/<name>.step` (CadQuery exports, Git LFS), `reference/manifest.json` entries (`file` field); `tools/cycloidal/export_cadquery.py` + `tools/cycloidal/import_cadquery.py` |
| Tests | `tests/cycloidal/test_{disc,eccentric_shaft,motor_plate,ring_gear_body,output_hub,housing,purchased,fitment,assembly,port}.py` + `tests/cycloidal/helpers.py` (one module per part, geometry marked `slow`) |
| Viewer / export | `./cadtool gen assemblies/cycloidal_drive.py`, `./cadtool viewer` (`?file=assemblies/cycloidal_drive.step`), `./cadtool export parts/cycloidal/<name>.step stl` |

## Viewing the drive

```bash
cd cad
./cadtool gen assemblies/cycloidal_drive.py          # assemblies/cycloidal_drive.step (git-ignored) if missing
./cadtool viewer                                     # then open the printed URL with ?file=assemblies/cycloidal_drive.step
#   also ?file=assemblies/arm.step (the drive in the arm), ?file=parts/cycloidal/cycloidal_ring_gear_body.step (any part),
#   ?file=robot/arm.urdf (the robot with joint sliders - shoulder_pitch turns the drive's rotor with j1_link)
./cadtool snapshot assemblies/cycloidal_drive.step snapshots/cycloidal_drive.png --size-profile assembly --view-labels
./cadtool snapshot assemblies/cycloidal_drive.step snapshots/cycloidal_drive_x.png --display '{"mode": "transparent"}' --camera "30:20"
#   (no turntable GIF: cadgen 0.6's `snapshot --video` renders a model's `@step(animation=…)` clip, and the
#    drive declares none - motion review is the CAD Viewer)
```

## 11. Port notes (CadQuery → build123d)

| CadQuery | build123d (here) |
|---|---|
| `cq.Workplane(...).circle().extrude()`, `.cut()`, `.union()` | algebra mode: `lib.cycloidal.geom.cylinder/through`, `-`, `+` |
| `.faces(">Z").edges().chamfer(d)` | `solid.chamfer(d, None, face.edges())` on the planar face picked by centre Z |
| `cq.Edge.makeSpline(pts, periodic=True)` → face → `extrudeLinear` | `Edge.make_spline(pts, periodic=True)` → `Face(Wire([edge]))` → `Solid.extrude` (same `GeomAPI_Interpolate`) |
| `.workplane().center(x, y)` (cumulative!) | explicit `Pos(x, y, z) *` |
| `.polygon(6, d).transformed(rotate=...)` | `RegularPolygon(r, 6, rotation=deg)` (vertex 0 on +X, like CadQuery) |
| 21 per-hole ruled lofts (funnels) | `Cone(r_entry, r_hole, depth)` |
| D-bore: cut a round bore, union a 0.25 mm key sliver back | round bore minus a half-space beyond the flat (identical geometry, no sliver) |
| `chamfer_outer_silhouette(result, cfg, external_face="<Z")` | `chamfer_outer_silhouette(solid, cfg, external_z=0.0)` — end face by height, edges by `GeomType.LINE` + radius |
| `.val().Volume()`, `.val().isInside()`, `.section(height=z).Area()`, `intersect().Volume()` | `lib.reference.solid_volume`, `Solid.is_inside`, thin-slab `section_area`, `interference` (`tests/cycloidal/helpers.py`) |
| `copy.deepcopy` + `object.__setattr__` on a frozen dataclass | `dataclasses.replace` |

- **Magic numbers promoted to tagged fields:** `housing.motor_plate_shaft_bore` 15, `lip_radial` 2,
  `ring_pin_entry_chamfer_depth/_add` 1/1, `pillar_inner_w/outer_w` 18/10, `bolt_nut_af` 7;
  `shaft.bridge_flange_add` 6; `motor.pilot_height` 2, `motor_bolt_thread_margin` 0.5,
  `motor_bolt_recess` 1; `tolerances.bolt_clearance_add` 0.4. Module constants: `PILLAR_OVERSHOOT`,
  `CUTTER_OVERSHOOT`, `BARREL_EDGE_MARGIN` (`lib/cycloidal/housing.py`).
- **Dropped (unused) fields:** `ProfileParams.spline_tolerance`, `PETGTolerances.bearing_inner_shaft_sub`
  / `sliding_clearance_add`, `HousingParams.wall_thickness` / `motor_plate_wall`,
  `BearingParams.ecc_qty` / `inp_qty`. `DriveConfig` itself is frozen now.
- **Geometry is identical** to the CadQuery builders (`tests/cycloidal/test_port.py`: same face sets, same
  tessellations to 1e-11, same analytic volume to 1e-13 for the analytic parts). Order matters on the
  disc: the lobe chamfer is applied while the end faces carry only the spline edge, *before* the holes.
- **OCCT volume caveat:** `BRepGProp` volume integration is ~0.3 % off on the 2000-knot spline face of
  the discs (differently on the CadQuery-exported and the native solids, both wrong; the adaptive
  integrator is worse). Face fingerprints and tessellations are the identity check; the discs' masses
  in the URDF therefore carry ~0.3 % error. `REF_VOL_TOL` stays at the 0.5 % default for the discs and
  is 1e-4 for the other designed parts.
- **Layout source:** the drive repo's `export.py` assembly had the ring pins at z 4.5 and the output
  pins at z 13; its `assembly.py` (and every test) uses 5.5 and 11. The port uses `stack_positions`
  (= `assembly.py`). The SolidWorks sub-assembly was imported from `export.py`'s STEP, so its pins are
  1–2 mm off ours (cosmetic, inside their holes); it is kept in `placements.json` as a cross-check.
- **Designed interference budget** (`test_module_interference_budget`, mm³): 6814 inner race / hub
  press fit 330.6 × 2; housing bolts through the solid nuts 321.7; motor-bolt heads in the plate 51.9
  and shanks in the vendor motor's tapped holes (modelled at the M3 minor diameter) 46.4; 6003 / lobe press fits
  26.8 × 2; the MKS kit's four M3x30 in the same holes from the rear 159.4. Everything else
  is < 1 mm³; the whole module vs the arm: base / j1_link 0, j1_coupler yoke contact ≤ 150 (the board
  behind the motor has 55 mm of free air).
- **Corrections to the drive repo's spec:** its §10 said "both discs are identical — the 180° offset is
  applied in the assembly": wrong (disc 2 carries the −9° phase, §1.2); §3.3's "7.6 mm disc holes"
  → 7.4; stale 67 / 134 / 120 mm comments (the OD is 140) and other rotted numbers in comments.
- **Two venvs:** `tools/cycloidal/export_cadquery.py` runs in the old repo's CadQuery venv
  (`cd ../cycloidal_drive && uv run python ../robotic-arm/cad/tools/cycloidal/export_cadquery.py`);
  everything else via `./cadtool`. The name maps of the exporter and `lib/reference.py` are kept equal
  by `test_exporter_name_map_matches_registry`.

## 12. Attachment to the arm

- `placements.json` record **`cycloidal_drive#1`** (`kind: module, designed: true`) = the SolidWorks
  node `New cyloidal assembly` (sic) at path 1.3: position (1.844381, 85.010435, 31.446506) mm,
  rotation XYZ (−180, −3.694455, 180)°, parent = the arm root. Its `solidworks` block keeps the node's
  leaves / solids / volume / world bbox as a cross-check; `assemblies/arm.py` locates
  `assemblies/cycloidal_drive.py` there (row after `j1_coupler#1`).
- **Frame:** module Z = motor axis, z = 0 the motor-plate outer face, motor body in −Z, hub face at 65.
  In the arm the axis is horizontal (module +Z → world −N, N = the J2/J3 pitch direction): the housing
  sits in the `j1_coupler` yoke (its pads touch the motor-plate outer face) and the hub's arm-mount
  face is coplanar with `j1_link`'s big mounting face — verified by `TestPoseInTheArm`. `j1_link` (parametric,
  `lib/upper_arm/`) puts its 4 Ø4.4 holes on `arm_mount_points` as this pose places them (`HubParams.bolt_angle_deg`,
  `tests/upper_arm/`): the SolidWorks holes sat 3.36° off, where the M4 bolts would not pass.
- **Kinematics:** the drive **is the `shoulder_pitch` joint** of `robot/frames.py` (axis `N` = the
  drive's −Z; origin `SHOULDER_ORIGIN` = `j1_link#1`'s origin, on the drive axis; limits
  `SHOULDER_PITCH_LIMIT_DEG`). `assemblies/cycloidal_drive.py BODIES` splits the rows into two rigid
  bodies and `LINKS` places them with a `:<body>` key suffix (`_occurrences.split_key` / `world_rows`):
  the **stator** (`cycloidal_drive#1:stator` — every row but the rotor's: motor plate, ring gear body, ring
  pins, housing bolts + nuts, NEMA 17 + bolts + its MKS board, and the gear train: eccentric shaft, support pin,
  discs, 6003s, 6814s) rides in `shoulder_link` with the yawing `j1_coupler`; the **rotor**
  (`cycloidal_drive#1:rotor` — output hub, output pins, 625) rides in `upper_arm_link` with `j1_link`.
  `EXPECTED["bodies"]` locks the per-body totals and `TestPoseInTheArm` checks the joint origin sits on the drive
  axis. The arm STEP's viewer tree keeps
  the module whole under `shoulder_link` (one linked child); the per-link meshes split it. Which MKS
  motor (`software/control/src/config.py` J1..J3) drives which joint is unconfirmed; `software/control/src/config.py` still carries
  `gear_ratio` 1.0 while `CYCLOIDAL_RATIO` = 20.

## 13. Change policy (carried over)

Every change to the drive must update (1) the tests — one `tests/cycloidal/test_<part>.py` per part
plus `tests/cycloidal/test_assembly.py` for the stack-up, (2) this document, and (3) run `./cadtool pytest`
green before it is done. Geometry changes also regenerate the STEPs (`./cadtool gen`), the
module totals lock (`totals()` / `totals(body)`), `robot/meshes/shoulder_link.stl` + `upper_arm_link.stl` and the URDF/SDF inertials
(`tools/robot/derive.py --urdf-draft` / `--check`).
