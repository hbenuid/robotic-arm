"""The belt joints' bearings - a leaf (lib/params.py re-exports it; the lib/ packages that size a seat, a stub or a
shoulder for them - lib/base/, lib/coupler/, lib/forearm/ - import it directly).

6806-2RS (61806), 30 x 42 x 7: a pair at each belt joint (base_yaw, elbow_pitch, wrist_pitch), one each side of the
lip that splits the housing's bore (parts/joints/bearing_6806.py; lib/mounts.py places them). The coupler's stub
turns in the upper one, the 90T pulley's hub in the lower one; bolting the pulley to the coupler clamps both inner
rings - the pulley's ring below, a shoulder on the stub above - while the lip holds the outer rings apart.
"""
BEARING_6806_BORE = 30.0          # [DATASHEET] 6806-2RS (61806)
BEARING_6806_OD = 42.0            # [DATASHEET]
BEARING_6806_WIDTH = 7.0          # [DATASHEET]
BEARING_6806_SHOULDER_DIA = 33.0  # [DATASHEET] class: a shoulder that bears on the inner ring only (above the Ø30 bore,
#                                   under the inner ring's outer edge, ~33.7-35 by brand); verify on the bearing in hand
BEARING_6806_MASS_G = 26.0        # [ESTIMATE] the 6808's 33 g scaled by the ring area (42^2 - 30^2) / (52^2 - 40^2); weigh one
