"""GT2 belt drives - a leaf module (no imports): the tooth pitch, the groove every printed pulley cuts, the pulleys the
arm uses, the stock belt lengths, the belt's section, the closed-belt arithmetic every belt joint needs (the elbow's
108T, the wrist belt's 90T, the forearm roll's 90T ring, the base_yaw 120T) and a belt's exact path round its
pulleys and a back-side idler (belt_runs() / belt_wraps() / belt_length(), idler_on_run()). lib/params.py re-exports the
constants; lib/pulley/ and lib/forearm/ import from HERE (they must not
import lib/params.py, which imports them back).

Tags as in lib/params.py. Units mm.
"""
import math

GT2_PITCH = 2.0                                     # [DATASHEET] GT2 tooth pitch
GT2_PULLEY_90T_TEETH = 90                           # [REFERENCE] printed 90T pulley (parts/joints/gt2_pulley_90t), the wrist's
GT2_PULLEY_108T_TEETH = 108                         # [DESIGN] printed 108T pulley, the elbow joint's (parts/joints/gt2_pulley_108t)
GT2_PULLEY_120T_TEETH = 120                         # [DESIGN] printed 120T pulley, the base_yaw joint's (parts/base/gt2_pulley_120t)
GT2_PULLEY_20T_TEETH = 20                           # [REFERENCE] purchased 20T pulley (parts/wrist/gt2_pulley_20t)
GT2_PULLEY_20_60T_TEETH = (60, 20)                  # [REFERENCE] printed compound pulley (parts/joints/gt2_pulley_20_60t): its lower band, its upper
GT2_PULLEY_90T_PITCH_DIA = GT2_PULLEY_90T_TEETH * GT2_PITCH / math.pi   # 57.30 mm pitch diameter
GT2_PULLEY_20T_PITCH_DIA = GT2_PULLEY_20T_TEETH * GT2_PITCH / math.pi   # 12.73 mm
GT2_RATIO = GT2_PULLEY_90T_TEETH / GT2_PULLEY_20T_TEETH                 # 4.5:1 [REFERENCE] the wrist belt's 90T on its 20T: motor turns per output turn; config.py's gear_ratio is 1 / this (robot/AGENTS.md)
GT2_PLD = 0.254                                     # [DATASHEET] pitch-line distance: pitch radius - pulley outside radius
GT2_BELT_W = 6.0                                    # [DATASHEET] the 6 mm belt every joint uses
GT2_TOOTH_DEPTH = 0.75                              # [DATASHEET] belt tooth height (= the pulley groove depth)

# The groove every printed pulley cuts (lib/pulley/teeth.py), as the SolidWorks 90T has it
# (reference/solidworks/gt2_pulley_90t.step): arcs only, each tangent to the next - a round bottom GT2_TOOTH_DEPTH under
# the land, a blend into each flank, the flank (its centre on the land circle, across the groove's centreline -
# flank_offset()), a fillet onto the land. The blend's and the fillet's centres follow from those tangencies (teeth.py
# groove_centres()).
GT2_GROOVE_R = 0.555                                # [REFERENCE] the groove's bottom (the belt tooth's tip)
GT2_FLANK_R = 1.0                                   # [REFERENCE] each flank ...
GT2_FLANK_ANGLE = 72.0                              # [REFERENCE] ... its centre on the land circle this many degrees / teeth
#                                                     across the centreline - a fifth of the pitch angle: 0.8 deg on the 90T,
#                                                     1.2 and 3.6 on the 20-60T's bands (their SolidWorks exports agree)
GT2_BLEND_R = 0.6385333225                          # [REFERENCE] flank -> bottom (the export's value: the constraint
#                                                     that set it in the SolidWorks sketch is not in the export)
GT2_TIP_R = 0.15                                    # [REFERENCE] flank -> land

# The printed 90T's hub (parts/joints/gt2_pulley_90t, in its SolidWorks part frame, its axis +Y): its end, which bolts
# flat onto the coupler's stub, and its outer face, the pulley bolts' heads sunk in counterbores in it; the 4x M4 run
# through it end to end.
GT2_PULLEY_90T_FACE_Y = (-13.2, 8.2)                # [REFERENCE] the hub's end .. the outer face, along +Y
GT2_PULLEY_90T_BOLT_R = 11.0                        # [REFERENCE] the 4x M4 on the pulley's own X / Z axes (the SolidWorks pattern)
GT2_PULLEY_90T_HEAD_SEAT = 5.0                      # [DESIGN] the M4 heads' seats (the counterbores' floors) this far under
#                                                     the outer face: the heads 1 under it, each screw 5 shorter than with
#                                                     its head on the face and every tip where it was (lib/pulley/params.py)

# The purchased toothless idler (parts/joints/gt2_idler_20t), a 20T-size smooth idler for the 6 mm belt, as the seller's
# drawing gives it (WINSINN "GT2 Idler Pulley - 20 Toothless, 5mm Bore", aluminium): two flanges on a smooth belt seat,
# a bearing inside on the bore.
GT2_IDLER_BORE = 5.0                                # [DATASHEET] the axle
GT2_IDLER_FLANGE_DIA = 18.0                         # [DATASHEET]
GT2_IDLER_SEAT_DIA = 12.1                           # [DATASHEET] the smooth belt seat (a 20T pulley's pulley_od(20): 12.22)
GT2_IDLER_WIDTH = 9.0                               # [DATASHEET] flange face to flange face
GT2_IDLER_CHANNEL_W = 7.0                           # [DATASHEET] between the flanges: 1 mm flanges, the belt + 1

# Closed-loop 2GT belts, 6 mm wide (mm = teeth x 2): the stock lengths the arm's belts are bought as - the GT2
# multipack's nine (one of each per pack) and the single-length packs (160, 200, 1000) [DESIGN]. Every belt length in
# the arm is one of them (tests/test_params_invariants.py), never an odd length ordered on its own; and every belt runs
# with a tensioner - an idler on its back (lib/belts.py idler_on_run) taking up what the stock length has over its open
# loop - so a joint's centre distance is the design's, not the belt's (cad/AGENTS.md).
STANDARD_2GT_LENGTHS = (110, 112, 122, 158, 160, 200, 280, 300, 400, 610, 1000)

# The belt itself (2GT, 6 mm): tooth tip to back; its pitch line (the cords) runs GT2_PLD outside its land (the tooth
# side's flat, on a pulley's tooth tips), so its back runs GT2_BELT_BACK outside the pitch line - what a back-side idler's
# seat carries.
GT2_BELT_THICKNESS = 1.38                           # [DATASHEET]
GT2_BELT_BACK = GT2_BELT_THICKNESS - GT2_TOOTH_DEPTH - GT2_PLD   # 0.376
GT2_BELT_MASS_G_PER_M = 7.5                         # [ESTIMATE] a 6 mm glass-cord neoprene belt


def pitch_dia(teeth: int) -> float:
    return teeth * GT2_PITCH / math.pi


def pulley_od(teeth: int) -> float:
    """The pulley's outside (tooth-tip) diameter: pitch diameter minus twice the pitch-line distance
    (90T: 56.79, the root land measured on the SolidWorks pulley)."""
    return pitch_dia(teeth) - 2.0 * GT2_PLD


def flank_offset(teeth: int) -> float:
    """How far a groove's flank centre sits across its centreline: on the land circle, GT2_FLANK_ANGLE / teeth degrees
    off it (90T: 0.396)."""
    return (pitch_dia(teeth) / 2.0 - GT2_PLD) * math.sin(math.radians(GT2_FLANK_ANGLE / teeth))


def pulley_90t_bolt_points() -> list[tuple[float, float]]:
    """The 90T's 4x M4 about its axis, on its two cross axes at GT2_PULLEY_90T_BOLT_R - (x, z) in the pulley's frame,
    (x, y) in the frame of a fastener pattern whose +Z runs along the axis (the pulley bolts, the holes they need)."""
    r = GT2_PULLEY_90T_BOLT_R
    return [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)]


def closed_belt_length(centre_distance: float, teeth_a: int, teeth_b: int) -> float:
    """Length of a closed belt over two pulleys at `centre_distance` (the usual two-pulley formula,
    pitch diameters D1 >= D2)."""
    d1, d2 = sorted((pitch_dia(teeth_a), pitch_dia(teeth_b)), reverse=True)
    return 2.0 * centre_distance + math.pi * (d1 + d2) / 2.0 + (d1 - d2) ** 2 / (4.0 * centre_distance)


def centre_distance(belt_length: float, teeth_a: int, teeth_b: int) -> float:
    """The inverse of closed_belt_length(): the centre distance a belt of `belt_length` sets."""
    d1, d2 = sorted((pitch_dia(teeth_a), pitch_dia(teeth_b)), reverse=True)
    b = belt_length - math.pi * (d1 + d2) / 2.0          # 2C + (d1-d2)^2 / 4C = b  ->  8C^2 - 4bC + (d1-d2)^2 = 0
    return (b + math.sqrt(b * b - 2.0 * (d1 - d2) ** 2)) / 4.0


# A belt's path, exactly: a loop is a list of circles (x, y, signed pitch radius) in counter-clockwise order round it -
# a pulley inside the loop +r, a back-side idler outside it -r. A line `offset` outside the pitch line (the land:
# -GT2_PLD, the back: +GT2_BELT_BACK) is the same loop with every signed radius + offset. closed_belt_length() above is
# the textbook approximation for two pulleys: at its centre distance the exact loop runs 0.04 (20T-90T) to 0.23
# (20T-120T) longer - well inside a belt's tolerance, so the motors stay where it puts them.

def pulley_circle(x: float, y: float, teeth: int) -> tuple[float, float, float]:
    """A pulley in a loop: its axis and pitch radius."""
    return (x, y, pitch_dia(teeth) / 2.0)


def idler_pitch_r() -> float:
    """A back-side idler (GT2_IDLER_*): the belt's back on its seat, so its pitch line this far from the axis (6.426)."""
    return GT2_IDLER_SEAT_DIA / 2.0 + GT2_BELT_BACK


def belt_runs(circles, offset: float = 0.0) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
    """The loop's straight runs, one leaving each circle for the next: (start, end, normal angle) - the normal the unit
    vector from a circle's centre to its tangent point, times the sign of its radius."""
    runs = []
    for (xi, yi, ri), (xj, yj, rj) in zip(circles, circles[1:] + circles[:1], strict=True):
        ri, rj = ri + offset, rj + offset
        d = math.hypot(xj - xi, yj - yi)
        phi = math.atan2(yj - yi, xj - xi) - math.acos((ri - rj) / d)
        nx, ny = math.cos(phi), math.sin(phi)
        runs.append(((xi + ri * nx, yi + ri * ny), (xj + rj * nx, yj + rj * ny), phi))
    return runs


def belt_wraps(circles, offset: float = 0.0) -> list[tuple[float, float]]:
    """Each circle's arc: (start normal angle, signed sweep) - counter-clockwise (+) round a pulley, clockwise (-) round a
    back-side idler - from the run arriving from the previous circle to the one leaving for the next."""
    runs = belt_runs(circles, offset)
    wraps = []
    for j, (_, _, r) in enumerate(circles):
        a_in, a_out = runs[j - 1][2], runs[j][2]
        sweep = (a_out - a_in) % (2.0 * math.pi) if r > 0 else -((a_in - a_out) % (2.0 * math.pi))
        wraps.append((a_in, sweep))
    return wraps


def belt_length(circles, offset: float = 0.0) -> float:
    """The loop's length along its pitch line (or the line `offset` outside it): runs + arcs."""
    runs = sum(math.dist(a, b) for a, b, _ in belt_runs(circles, offset))
    arcs = sum(abs(r + offset) * abs(sweep) for (_, _, r), (_, sweep) in zip(circles, belt_wraps(circles, offset), strict=True))
    return runs + arcs


def idler_on_run(circles, run: int, x: float, belt: float) -> tuple[float, float, float]:
    """A back-side idler on the run leaving circles[run] (a run along x), its axis at `x`, moved across it (along y)
    into the loop until the loop is `belt` long. Raises if the belt is shorter than the open loop."""
    r = idler_pitch_r()
    (x0, y0), (x1, y1), phi = belt_runs(circles)[run]
    y_run = y0 + (y1 - y0) * (x - x0) / (x1 - x0)   # the run's pitch line at x
    out = math.copysign(1.0, math.sin(phi))           # the loop's outside across the run, along y

    def length(push: float) -> float:
        return belt_length(circles[:run + 1] + [(x, y_run + out * (r - push), -r)] + circles[run + 1:])

    lo = 1e-6
    if belt < length(lo):
        raise ValueError(f"a {belt} belt is shorter than the open loop ({length(lo):.2f})")
    hi = lo
    while length(hi) < belt:        # walk in, a step at a time (the solved loop stays valid), then halve the last step
        lo, hi = hi, hi + 0.5
    for _ in range(60):
        mid = (lo + hi) / 2.0
        lo, hi = (mid, hi) if length(mid) < belt else (lo, mid)
    return (x, y_run + out * (r - lo), -r)
