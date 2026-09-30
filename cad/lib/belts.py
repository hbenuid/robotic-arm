"""GT2 belt drives - a leaf module (no imports): the tooth pitch, the groove every printed pulley cuts, the pulleys the
arm uses and the closed-belt arithmetic every belt joint needs (the elbow / wrist belts on the 90T pulleys, the forearm
roll's 90T ring). lib/params.py re-exports the constants; lib/pulley/ and lib/forearm/ import from HERE (they must not
import lib/params.py, which imports them back).

Tags as in lib/params.py. Units mm.
"""
import math

GT2_PITCH = 2.0                                     # [DATASHEET] GT2 tooth pitch
GT2_PULLEY_90T_TEETH = 90                           # [REFERENCE] printed 90T pulley (parts/joints/gt2_pulley_90t), used x2
GT2_PULLEY_120T_TEETH = 120                         # [DESIGN] printed 120T pulley, the base_yaw joint's (parts/base/gt2_pulley_120t)
GT2_PULLEY_20T_TEETH = 20                           # [REFERENCE] purchased 20T pulley (parts/wrist/gt2_pulley_20t)
GT2_PULLEY_20_60T_TEETH = (60, 20)                  # [REFERENCE] printed compound pulley (parts/joints/gt2_pulley_20_60t): its lower band, its upper
GT2_PULLEY_90T_PITCH_DIA = GT2_PULLEY_90T_TEETH * GT2_PITCH / math.pi   # 57.30 mm pitch diameter
GT2_PULLEY_20T_PITCH_DIA = GT2_PULLEY_20T_TEETH * GT2_PITCH / math.pi   # 12.73 mm
GT2_RATIO = GT2_PULLEY_90T_TEETH / GT2_PULLEY_20T_TEETH                 # 4.5:1 [REFERENCE] motor turns per output turn; config.py's gear_ratio is 1 / this (robot/AGENTS.md)
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
# flat onto the coupler's stub, and its outer face, under the pulley bolts' heads; the 4x M4 run through it end to end.
GT2_PULLEY_90T_FACE_Y = (-13.2, 8.2)                # [REFERENCE] the hub's end .. the outer face, along +Y
GT2_PULLEY_90T_BOLT_R = 11.0                        # [REFERENCE] the 4x M4 on the pulley's own X / Z axes (the SolidWorks pattern)

# The purchased toothless idler (parts/joints/gt2_idler_20t), a 20T-size smooth idler for the 6 mm belt, as the seller's
# drawing gives it (WINSINN "GT2 Idler Pulley - 20 Toothless, 5mm Bore", aluminium): two flanges on a smooth belt seat,
# a bearing inside on the bore.
GT2_IDLER_BORE = 5.0                                # [DATASHEET] the axle
GT2_IDLER_FLANGE_DIA = 18.0                         # [DATASHEET]
GT2_IDLER_SEAT_DIA = 12.1                           # [DATASHEET] the smooth belt seat (a 20T pulley's pulley_od(20): 12.22)
GT2_IDLER_WIDTH = 9.0                               # [DATASHEET] flange face to flange face
GT2_IDLER_CHANNEL_W = 7.0                           # [DATASHEET] between the flanges: 1 mm flanges, the belt + 1

# Closed-loop 2GT belts, 6 mm wide, as commonly stocked (mm = teeth x 2) [ESTIMATE] - confirm with the vendor.
STANDARD_2GT_LENGTHS = (110, 112, 122, 124, 130, 150, 158, 160, 188, 200, 202, 208, 210, 220, 224, 230, 232,
                        240, 250, 252, 254, 258, 260, 264, 280, 288, 294, 300, 320, 336, 350, 360, 400)


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
