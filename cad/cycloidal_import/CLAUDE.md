# Cycloidal Drive Specification — Shoulder Joint

## Overview

| Parameter | Value |
|---|---|
| Application | Shoulder joint, 3-DOF robotic arm, ~400mm reach |
| Type | Two-disc cycloidal drive, 180° offset |
| Gear Ratio | 20:1 (20 lobes, 21 ring pins) |
| Motor | NEMA 17, 48mm body, 20mm shaft, 5mm ⌀ |
| Eccentricity | 1.5mm |
| Housing OD | ~140mm |
| Print Material | PETG, 100% infill on discs |

---

## 1. Drive Geometry

### 1.1 Ring Gear

| Parameter | Value | Notes |
|---|---|---|
| Number of ring pins | 21 | N + 1 where N = 20 lobes |
| Ring pin diameter | 4.00mm | Ground steel dowel, h6 tolerance |
| Ring pin circle ⌀ | 108.00mm | Centered in housing bore |
| Ring pin length | 35mm | 3.5mm motor plate engagement + 28mm bore zone (open) + 3.5mm bearing-zone wall engagement. Motor plate has 9mm through-holes; pin sits at the inner end with 5.5mm of empty hole at the outer face. |
| Housing bore ⌀ | 116.00mm | 140mm OD − 2 × 12mm wall |

### 1.2 Cycloidal Disc

| Parameter | Value | Notes |
|---|---|---|
| Number of lobes | 20 | Sets the ratio |
| Disc count | 2 | Orbit centers 180° apart to cancel vibration; disc 2's epitrochoid profile is phase-rotated by −180°/N_lobes = −9° relative to disc 1 so lobes mesh correctly with ring pins. Output-pin hole positions are identical between the two discs (disc-local 0°/90°/180°/270° on a 60 mm circle); only the profile is rotated. Disc 1 and disc 2 are **not interchangeable** printed parts — see `cfg.gear.disc2_phase_deg`. |
| Approximate OD | ~108mm | Defined by epitrochoid profile |
| Center bore ⌀ | 35.10mm | 35mm bearing OD + 0.10mm clearance (PETG) |
| Disc thickness | 10.00mm | Matched to 6003-2RS bearing width |
| Lobe chamfer | 1.00mm × 45° | Both Z faces of outer epitrochoid; assembly lead-in past ring pins, hides elephant foot. Symmetric so disc has no "wrong side". Costs ~20% lobe contact length. |
| Inter-disc spacer | 2.00mm | Separates the two discs |
| Output pin holes | 4× equally spaced | See Section 1.3 |

### 1.3 Output Stage

| Parameter | Value | Notes |
|---|---|---|
| Output pin count | 4 | Equally spaced at 90° |
| Output pin circle ⌀ | 60.00mm | — |
| Output pin ⌀ | 4.00mm | 4mm × 45mm h6 ground steel dowel pin |
| Disc pin hole ⌀ | 7.40mm | 4mm pin + 2 × 1.5mm ecc + 0.4mm clearance (0.20mm radial slack — practical floor for 4 rigid pins, see note below) |
| Pin hole approach | Oversized, no bearings | Greased sliding fit through discs (7.4mm); blind clearance holes in hub (4.20mm × 19mm deep), captured between closed hub ceiling and motor plate inner face |

**Clearance check — output pin holes:**

- Pin hole center radius: 30.00mm
- Pin hole edge (inner): 30.00 − 3.70 = 26.30mm from center
- Center bore radius: 17.55mm
- **Wall between bore and pin hole: 8.75mm** ✓
- Pin hole edge (outer): 30.00 + 3.70 = 33.70mm from center
- Disc lobe valley (approx inner radius): ~49mm from center
- **Wall between pin hole and lobe root: ~15mm** ✓

**Backlash (output lost motion):** the output-pin holes are the dominant *designed* backlash source — the ring-pin mesh (§8) is a zero-clearance theoretical epitrochoid, so it contributes ~0 lost motion on a nominal print. The hole carries **0.20mm of radial slack** beyond the kinematic orbit requirement (slack = hole_r − pin_r − e = 3.70 − 2.00 − 1.50), giving **≈ ±0.38° of output lost motion** at the 30mm pin circle (down from ±0.95° at the original 8.00mm hole, via 7.60mm/±0.57°). **7.4mm is the practical floor for hole-tightening:** below ~7.2mm the 4 rigid steel pins over-constrain against printed-hole position error (disc holes + hub holes must agree across two separate prints) and the disc binds or won't assemble after print shrink. Going lower requires reducing the *secondary* sources — hub pin fit (4.20mm), 6003 bearing radial clearance, the 35.10mm disc center bore — or fitting bushings/rollers on the output pins (a redesign). Guarded by `test_output_hole_backlash_budget` in `tests/test_disc_geometry.py`.

### 1.4 Eccentric Shaft

| Parameter | Value | Notes |
|---|---|---|
| Shaft OD at bearing seats | 17.10mm | Slight clearance for 6003 bearing bore |
| Eccentricity | 1.50mm | Offset between shaft center and bearing seat center |
| Spine OD | 5.00mm | Between lobes |
| Input collar OD | 10.00mm | Enlarged input section for D-bore wall |
| D-bore ⌀ | 5.13mm | Receives motor shaft directly (5.00mm + 2×0.065mm clearance) |
| D-bore flat | 4.50mm | Matches motor shaft D-cut |
| D-bore depth | 14.00mm | Provides 13mm motor shaft engagement + 1mm bottom clearance (motor shaft 22mm − motor plate 9mm = 13mm) |
| Support pin hole ⌀ | 5.15mm | Blind hole for 5mm steel dowel clearance fit (5.00 + 2×0.075mm) |
| Support pin hole depth | 11.00mm | Through lobe 2 + 1mm into bridge; shortened from 12mm to keep 1mm wall to deepened D-bore |
| Support pin | 5mm × 20mm ground steel dowel, h6 | Press-fit into shaft, supports 625 bearing |
| Material | PETG | 3D printed; use 100% infill, 0.16mm layer height |

The eccentric shaft has two lobes offset 180° from each other, one per disc. Each lobe is a cylindrical section with center offset 1.5mm from the shaft axis. The input end has a D-bore socket that receives the motor shaft directly (no coupler). A 10mm OD collar at the input provides wall thickness around the D-bore.

The output end has a blind hole (5.15mm × 11mm deep) centered on the shaft axis, extending from the output face of lobe 2 through into the bridge zone. A 5mm × 20mm ground steel dowel pin (h6) is pressed into this hole and extends through the 2mm clearance gap into the 625 bearing in the output hub, providing rigid radial support for the shaft's free end. Pin breakdown: 11mm insertion + 2mm gap + 5mm bearing + 2mm proud = 20mm.

The bridge between the two lobes (spanning the 2mm inter-disc spacer zone) is enlarged to 23.10mm OD (lobe OD + 6mm, 3mm radial flange per side) and acts as a bearing retention flange, preventing the two 6003 bearings from sliding toward each other. The bridge is a ruled loft that transitions the eccentric center from (+e, 0) to (−e, 0).

---

## 2. Bearings

### 2.1 Eccentric Bearings — 6003-2RS

| Parameter | Value |
|---|---|
| Quantity | 2 (one per disc) |
| Designation | 6003-2RS |
| Bore (d) | 17mm |
| OD (D) | 35mm |
| Width (B) | 10mm |
| Type | Deep groove ball bearing, sealed |
| Dynamic load rating | ~5.4 kN (typical) |
| Purpose | Supports cycloidal disc on eccentric shaft |

Each bearing press-fits into the center bore of one cycloidal disc. The eccentric shaft's lobed sections seat in the bearing bore.

### 2.2 Output Bearings — 6814-2RS (×2, end-to-end)

| Parameter | Value |
|---|---|
| Quantity | 2 |
| Designation | 6814-2RS |
| Bore (d) | 70mm |
| OD (D) | 90mm |
| Width (B) | 10mm each (20mm total) |
| Type | Deep groove ball bearing, sealed |
| Dynamic load rating | ~7.0 kN (typical) |
| Purpose | Main output support, carries radial + moment loads |

Two bearings mounted end-to-end in the housing. The inner races sit on the output hub. The outer races sit in the housing bore. Paired arrangement provides significantly better moment load resistance than a single bearing.

**Clearance check — output pins vs. output bearing:**

- Output bearings sit at z=37–57mm; discs at z=13–35mm — axially separated, no interaction with disc pin holes.
- Hub pin holes (Ø4.20mm on 60mm circle) span radius 27.9–32.1mm — well inside the 35mm bearing inner-race bore.

### 2.3 Input Shaft Support Bearing — 625-2RS

| Parameter | Value |
|---|---|
| Quantity | 1 |
| Designation | 625-2RS |
| Bore (d) | 5mm |
| OD (D) | 16mm |
| Width (B) | 5mm |
| Type | Deep groove ball bearing, sealed |
| Dynamic load rating | ~1.0 kN (typical) |
| Purpose | Supports eccentric shaft at output side |

Located in the output hub, supporting the eccentric shaft's free end via the 5mm steel dowel pin.

---

## 3. Non-Bearing Purchased Parts

### 3.1 Motor

| Parameter | Value |
|---|---|
| Type | NEMA 17 |
| Faceplate size | 42.3mm × 42.3mm |
| Body length | 48mm (excluding shaft) |
| Shaft diameter | 5mm (with D-cut flat at 4.5mm across) |
| Shaft length | 22mm from mounting face (20mm from pilot face; 18mm D-cut from tip, 4mm round base) |
| Pilot / centering boss | 22mm ⌀ × 2mm |
| Mounting hole spacing | 31mm × 31mm (center-to-center) |
| Mounting hole thread | M3, tapped 4.5mm deep |
| Holding torque | ~0.45 Nm (typical) |

### 3.2 Ring Pins

| Parameter | Value |
|---|---|
| Quantity | 21 (buy 25) |
| Diameter | 4.00mm, h6 ground |
| Length | 35mm |
| Material | Hardened steel dowel pin |

### 3.3 Output Pins

| Parameter | Value |
|---|---|
| Quantity | 4 |
| Type | Ground steel dowel pin, h6 |
| Diameter | 4.00mm |
| Length | 45mm |
| Retained by | Friction in 4.20mm blind holes in output hub (4.20mm designed → ~4.00–4.10mm printed → light press on 4mm pin); closed hub ceiling above and motor plate inner face below act as backup capture; free-floating through disc 7.6mm clearance holes |
| Material | Hardened steel dowel pin |

### 3.4 Housing Bolts

| Parameter | Value |
|---|---|
| Quantity | 8 |
| Type | M4 × 55mm socket head cap screw (ISO 4762) |
| Head ⌀ | 7.0mm |
| Head height | 4.0mm |
| Shank ⌀ | 4.0mm |
| Shank length | 55mm |
| Retained by | Counterbore in motor plate, M4 hex nut captured in ring gear body output face |

### 3.5 Housing Nuts

| Parameter | Value |
|---|---|
| Quantity | 8 |
| Type | M4 hex nut |
| Width across flats | 7.0mm (standard M4) |
| Thickness | 3.2mm |
| Captured in | Hex nut pocket in ring gear body output face (pocket sized at 7.2mm AF for clearance) |

### 3.6 Other Fasteners

| Item | Spec | Qty | Purpose |
|---|---|---|---|
| Motor mounting bolts | M3 × 10mm socket head (5.3mm head ⌀ × 3mm head height, 13mm total) | 4 | Mount motor to housing (6mm through plate + 4mm thread engagement, head flush in 3mm counterbore on inner face) |
| Arm-mount bolts | M4 socket head, length per arm link + ~28mm hub (≈M4 × 40–50mm) | 4 | Arm link → output hub; drops through the proud face and threads into a captive M4 nut on the hub inner face |
| Arm-mount nuts | M4 hex nut | 4 | Captive in the output-hub inner-face hex pockets (7.2mm AF × 4mm); install before pressing hub through 6814 bearings |

---

## 4. Axial Stack-Up

Measured from motor mounting face inward:

| Layer | Thickness | Running Total |
|---|---|---|
| Motor-side housing plate wall | 5mm | 5mm |
| Motor plate inner wall | 4mm | 9mm |
| Input clearance (D-shaft collar) | 4mm | 13mm |
| Cycloidal disc 1 + 6003 bearing | 10mm | 23mm |
| Inter-disc spacer | 2mm | 25mm |
| Cycloidal disc 2 + 6003 bearing | 10mm | 35mm |
| Clearance to output bearing | 2mm | 37mm |
| Output bearings (2 × 6814) | 20mm | 57mm |
| Output-side housing wall (retention lip + nut pockets) | 3mm | 60mm |

**Total housing depth: ~60mm** (not including motor body protrusion)

**Output hub protrusion:** the rotating output hub extends **5mm proud of the 60mm housing** (its output face is at z=65mm) so the arm link clears the stationary housing output face. This is an output-side stub on the rotating member — it does not change the housing depth or the 3mm output wall; the ring gear body and motor plate are unaffected.

**Housing depth follows the bolt length:** the output cap was removed and its two jobs (6814 retention + captive nuts) folded into the ring gear body, shrinking the housing 65mm → 60mm and dropping the bolt M4×60 → M4×55. The M4×55 bolt occupies counterbore 4.5mm + 55mm = 59.5mm from the motor face, landing inside the 60mm depth with full nut engagement and no protrusion past the output face (`tests/test_assembly_clearances.py::test_bolt_does_not_protrude`). A shorter housing would need a shorter standard bolt, but the next size down (M4×50) is too short to reach the captive nut — so 60mm is the practical floor.

**Total assembly depth including NEMA 17:** ~60mm housing + 48mm motor body = **~108mm** (motor pilot recesses 2mm into the motor plate's outer face — already inside the 60mm housing depth, so it doesn't add or subtract from the total). The 5mm hub stub adds to the output-side interface but not to the housing/motor envelope.

---

## 5. Housing Design Notes

### 5.1 Overall Envelope

| Dimension | Value |
|---|---|
| OD | 140mm |
| Depth | ~60mm (housing only) |
| Housing bore (ring pin area) | 116mm |
| Output bearing seat OD | 90.15mm (press fit for 6814 outer race) |
| Output bearing seat depth | 20mm (for 2 × 6814) |

### 5.2 Housing Split

The housing splits into 2 printed parts (the former output cap is gone — its two structural jobs are folded into the ring gear body):

1. **Motor plate** — NEMA 17 bolt pattern, 15mm shaft bore for motor shaft pass-through, 21 ring-pin through-holes (4.20mm ⌀), 8× M4 counterbore holes (7.4mm ⌀ × 4.5mm deep) on outer face. Plate is 9mm thick uniform — the previous Ø100mm × 1mm inner-face recess has been replaced by trimming the entire plate to that level. M3 motor-bolt heads sit flush with the inner face in 3mm counterbores.
2. **Ring gear body** — Main cylinder (51mm tall) with 21 ring-pin blind holes (4.20mm ⌀, 31.5mm deep, chamfered entry), output bearing seat bore, 8× M4 clearance through-holes. Lateral pin retention is provided by the motor plate (9mm through-holes) plus 3.5mm engagement at the bearing-zone end. No input shoulder ring — the 6814 bearings are retained by press-fit into the 90.15mm seat plus an **integral output-end lip** (the bore steps 90.15→86.15mm at z=57, a 2mm radial lip over the 90mm outer races; bearings insert from the input side and seat against it). The output face also carries **8× captive hex nut pockets** (7.2mm AF × 4.0mm deep) for the housing-bolt nuts — each backed by the full-height bolt pillar, so the thin 3mm output wall imposes no floor constraint.

**Shared outer profile**: both parts share the same outer silhouette — an 8-pillar / 8-window pattern around the bolt circle (pillars 18mm wide at the bore, 10mm wide at the OD, one per M4 bolt). The motor plate sits directly on the ring gear body's pillar faces; no continuous rim anywhere. The profile is generated by a single shared cutter (`src/helpers/housing_profile.py:build_reveal_window_cutter`) that every housing builder subtracts from its base solid, guaranteeing both parts stay geometrically aligned. The central solid regions (motor mount on the plate, bearing-retention annulus + lip on the ring gear body) are untouched.

**Outer-edge chamfer** (`cfg.housing.edge_chamfer`, default 1.5mm): a shared post-process helper (`src/helpers/housing_profile.py:chamfer_outer_silhouette`) breaks the external edges of both parts; **faces that mate against an adjacent part are left sharp** so the stack still seats flush. Applied per part after the windows are cut:

- **External end faces** — the motor plate's motor-mounting face and the ring gear body's output face are external (nothing seats on them). Each has its *entire* outer-wire perimeter beveled: pillar outer arcs, the inner arcs between pillars, and the pillar side edges (taken from the face's outer wire, so internal bolt/shaft/pilot/nut-pocket holes stay sharp), plus the 8 pillar outer **vertical** corners.
- **Mating faces** — the motor plate's inner face and the ring gear body's motor-side face seat against each other and are left sharp so the stack beds flush (only the shared barrel verticals are beveled there). The ring body's output-face bevel uses the face outer wire, which the OCCT kernel chamfers cleanly even with the stepped lip bore inboard — the old `StdFail_NotDone` limitation only hit end-rims that crossed the window edges.

On the assembled stack this reads as one continuously beveled outer barrel with flush, sharp seams at the part interface. Set `edge_chamfer = 0` to disable; keep ≤2mm to preserve the nut-pocket-to-OD wall.

Parts joined with 8× M4 × 55mm socket head cap screws on a 125mm bolt circle. Bolt heads sit in counterbores on the motor plate; M4 hex nuts captured in hex pockets on the ring gear body's output face.

### 5.3 Output Hub / Plate

A separate printed PETG part (3D-printed; an aluminum version is also viable) that:

- Has 4× blind clearance holes (4.20mm × 19mm deep) on the 60mm pin circle for 4mm dowel pins, closed by a 1mm ceiling on the output side. Depth references the 20mm bearing-grip zone (`cfg.stack_up.output_bearing_total`), so it stays fixed regardless of the proud extension
- Passes through the 2× 6814 bearing inner races (hub OD = 70.3mm; 70mm bearing inner race + 0.3mm interference grip)
- **Output face protrudes 5mm past the chassis.** The hub is 28mm tall (`output_bearing_total` 20mm + `output_wall` 3mm + `proud_above_housing` 5mm), so its output face sits at **z=65mm — 5mm proud of the housing output face (z=60)**. The rotating arm-link interface therefore clears the stationary housing with a 5mm air gap (no rubbing). The proud section passes through the ring body's integral-lip Ø86.15mm bore with ~7.9mm radial clearance. Tune via `cfg.output_hub.proud_above_housing`
- **Arm-link mount:** 4× M4 clearance through-holes (4.4mm ⌀) on a 50mm bolt circle, offset 45° from the output pins. Each hole runs the full height into a **captive M4 hex-nut pocket (7.2mm AF × 4mm deep) on the inner face** — the arm-link bolt drops from the proud face, through the hub, and threads into the trapped nut (no threads cut in PETG). Drop the 4 nuts into the pockets **before** pressing the hub through the 6814 bearings. Tune via `cfg.output_hub.arm_mount_*`
- The central 6mm shaft clearance bore is capped at the bearing-grip zone (runs z=37→57 only). Above that the block is solid **except for a central lightening recess** (Ø36mm, ~7mm deep) cut into the proud arm-mount face — it reclaims ~7cm³ of otherwise-dead material; the arm link bears on the r=25→35mm annulus around the bolt circle. Tune via `cfg.output_hub.arm_mount_pocket_dia` (set 0 for a fully sealed face)
- Houses a 625 bearing seat (Ø16.2mm × 5mm) on the inner-face side to support the eccentric shaft output end

---

## 6. PETG Print Tolerances

**Important context:** PETG holes print **0.10–0.20mm undersized** vs. the design value due to cooling shrinkage, elephant foot, and arc overshoot at the top of round holes. The positive offsets in the table below are **print compensation** — they shift the as-printed dimension back toward nominal. As a result:

- A "+0.20mm" pin hole (designed 4.20mm, printed ~4.00–4.10mm) lands at a **slip-to-light-press fit** on a 4mm dowel — the pin is held primarily by friction, with surrounding capture features (closed ends, adjacent parts) acting as backup retention.
- A "+0.15mm" bearing seat (designed 90.15mm for the 6814 outer race, printed ~90.00mm or slightly under) lands at a **firm press fit**. The smaller 625 outer race seat in the output hub uses a looser +0.20mm offset (designed 16.2mm, printed ~16.00–16.10mm) — a **slip-to-light-press fit** to ease assembly of the small bearing in PETG.
- Designing a hole *smaller* than the steel part (negative offset) is unsafe — print shrinkage stacks on top and the joint either won't assemble or splits the PETG.

In short: **everything ends up a press fit on a real print**; the offset just controls *how tight*. The "clearance" / "press" labels below are design-relative, not as-printed.

| Fit Type | Nominal Adjustment | Application |
|---|---|---|
| Bearing outer race → housing | +0.15 to +0.20mm on bore ⌀ | 625 outer race seat in output hub (+0.20mm), 6814 outer race seat in ring gear body (+0.15mm) |
| Bearing inner race → shaft/hub | −0.05 to −0.10mm on shaft ⌀ | Output hub through 6814 inner race |
| Ring pin holes (motor plate) | +0.20mm on hole ⌀ (4.20mm) | 4mm pins, through-holes (slip-press as printed) |
| Ring & output pin holes (gear body / hub) | +0.20mm on hole ⌀ (4.20mm) | 4mm pins, blind holes (slip-press as printed) |
| Sliding / clearance fit | +0.20 to +0.30mm on hole ⌀ | Disc center bore. **Disc output-pin holes are an exception** — modeled at nominal 7.4mm with no positive offset so PETG shrink *reduces* (not adds) output backlash; see §1.3 |
| Motor shaft D-bore | +0.065mm on bore ⌀ (5.13mm for 5mm motor shaft) | Eccentric shaft D-bore receiving NEMA 17 shaft (slip-to-light press as printed) |
| Support pin clearance bore | +0.075mm on bore ⌀ (5.15mm for 5mm pin) | Steel dowel pin hole in eccentric shaft output end (firm press as printed) |
| General mating surfaces | +0.15mm clearance | Housing halves, spacers |

**Notes:**

- Print all cycloidal discs flat (lobes in XY plane) at 100% infill
- Use 0.16mm or finer layer height for bearing seats
- Test-print a bearing fit gauge before committing to full parts — every printer/filament combination shrinks differently, so the +0.20mm compensation may need tuning
- PETG shrinks ~0.3–0.5% on large dimensions (140mm housing may need to be designed at ~140.5mm)

---

## 7. Performance Estimates

| Parameter | Value | Notes |
|---|---|---|
| Motor torque | 0.45 Nm | Typical NEMA 17 48mm |
| Gear ratio | 20:1 | |
| Theoretical output torque | 9.0 Nm | Before losses |
| Estimated efficiency | 55–65% | 3D printed, no pin bearings |
| **Practical output torque** | **5.0–5.9 Nm** | |
| Max payload at 400mm | ~1.3–1.5 kg | Including arm weight |
| Input speed (typical) | 200–500 RPM | |
| Output speed | 10–25 RPM | |
| Backdrivability | Not backdrivable | Inherent to cycloidal drives |
| Output backlash | ~±0.38° (~0.8° total) | From 0.20mm output-pin-hole slack (practical floor); ring mesh is zero-clearance (§1.3) |

**Torque budget warning:** 5–6 Nm at the shoulder with 400mm reach is marginal for a 3-DOF arm. The arm structure, two additional joint motors, and wiring may consume most of the payload budget. This design is suitable for a lightweight demonstrator arm. For heavier payloads, consider upgrading to NEMA 23 or increasing the ratio.

---

## 8. Cycloidal Disc Profile

The disc profile is an **epitrochoid** defined by:

```
x(θ) = R·cos(θ) − r·cos(θ + ψ) − e·cos(N·θ)
y(θ) = −R·sin(θ) + r·sin(θ + ψ) + e·sin(N·θ)

where:
  ψ = atan2(sin((1 − N)·θ), (R / (e·N)) − cos((1 − N)·θ))

  R = ring pin circle radius = 54.00mm
  r = ring pin radius = 2.00mm
  N = number of ring pins = 21
  e = eccentricity = 1.50mm
```

The number of lobes on the disc = N − 1 = 20, giving the 20:1 ratio.

Generate this profile at high resolution (e.g., 1000+ points per full revolution of θ from 0 to 2π) and export as DXF or SVG for CAD import.

---

## 9. Shopping List (Summary)

| # | Part | Specification | Qty | Est. Cost |
|---|---|---|---|---|
| 1 | NEMA 17 stepper | 48mm body, 5mm × 20mm shaft | 1 | $10–15 |
| 2 | Eccentric bearing | 6003-2RS (17 × 35 × 10mm) | 2 | $4–8 |
| 3 | Output bearing | 6814-2RS (70 × 90 × 10mm) | 2 | $16–40 |
| 4 | Input shaft bearing | 625-2RS (5 × 16 × 5mm) | 1 | $1–2 |
| 5 | Ring pins | 4mm × 35mm ground dowel h6 | 25 | $8–12 |
| 6 | Output pins | 4mm × 45mm ground steel dowel pin, h6 | 4 | $2–4 |
| 7 | Motor bolts | M3 × 10mm socket head (13mm total) | 4 | $1–2 |
| 8 | Housing bolts | M4 × 55mm socket head cap screw | 8 | $2–4 |
| 8a | Housing nuts | M4 hex nut | 8 | $1 |
| 9 | Shaft support pin | 5mm × 20mm ground steel dowel pin, h6 | 1 | $0.50–1 |
| 10 | Arm-mount bolts | M4 socket head, length per arm link + ~28mm hub (≈M4 × 40–50mm) | 4 | $1–2 |
| 10a | Arm-mount nuts | M4 hex nut (captive in output hub inner face) | 4 | $1 |
| | | | **Total** | **~$47–87** |

---

## 10. Project Structure & Development

### File Layout

```
src/
  params.py              # All dimensions, tolerances, counts (frozen dataclasses)
  profiles.py            # Epitrochoid math (pure numpy)
  eccentric_shaft.py     # PETG-printed shaft with two offset lobes + D-bore
  cycloidal_disc.py      # Epitrochoid profile disc (print 2 copies)
  ring_gear_body.py      # Housing cylinder + bearing seat + integral retention lip + output-face nut pockets + reveal windows
  motor_plate.py         # NEMA 17 mount + ring pin holes
  output_hub.py          # Output plate (proud arm-mount face + M4 captive-nut holes) through 6814 bearings + 625 seat
  purchased_parts.py     # Simplified bearings, motor, pins for visualization
  helpers/
    housing_profile.py   # Shared 8-pillar / 8-window cutter (motor plate, ring gear body)
assembly.py              # OCP CAD Viewer — all parts positioned per stack-up
export.py                # Export STEP + STL to export/
```

### Environment

- CadQuery 2.7.0 managed via `uv` (`pyproject.toml` + `uv.lock`)
- Disc profile uses `cq.Edge.makeSpline(periodic=True)` for high-precision closed curves
- Both discs are identical — 180° offset is applied in the assembly only

### Commands

- `uv run pytest tests/ -v` — run all tests
- `uv run python assembly.py` — interactive 3D viewer
- `uv run python export.py` — generate `export/step/` and `export/stl/`
- Each `src/*.py` has an `if __name__ == "__main__"` block for standalone viewing

### Development Workflow

**Change policy — every change MUST update tests and docs.** A change is not complete
until all three of the following are done:

1. **Tests** — Add or update tests under `tests/` so the new or changed behavior is
   covered. Each `src/` part has a matching `tests/test_*.py` (e.g. `output_hub.py` ↔
   `tests/test_output_hub.py`), plus `tests/test_assembly_clearances.py` for
   cross-part stack-up; mirror that layout.
2. **Docs** — Update this `CLAUDE.md` (and any other `.md` files) so the spec, tables,
   axial stack-up, tolerances, and notes reflect the change. Code and docs must never
   drift out of sync.
3. **Verify** — Run `uv run pytest tests/ -v` and confirm the full suite passes before
   considering the change done.

This applies to any change to the design parameters (`src/params.py`), part geometry,
or the assembly — including dimension tweaks that ripple into the stack-up and
clearance tables in this document.
