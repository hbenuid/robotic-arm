"""The belt joints' bearings - a leaf (lib/params.py re-exports it; the lib/ packages that size a seat, a stub or a
shoulder for them - lib/base/, lib/coupler/, lib/forearm/ - import it directly).

6806-2RS (61806), 30 x 42 x 7: a pair at each belt joint (base_yaw, elbow_pitch, wrist_pitch), one each side of the
lip that splits the housing's bore (parts/joints/bearing_6806.py; lib/mounts.py places them). The coupler's stub
turns in the upper one and reaches on through the lip to the 90T pulley, whose hub fills the lower one and bolts flat
onto the stub's end. The bolts clamp both inner rings - the pulley's ring below, a shoulder on the stub above - and
the stub's 2 mm inside the lip spaces them exactly as the lip spaces the outer rings: a solid joint, no play.
"""
BEARING_6806_BORE = 30.0          # [DATASHEET] 6806-2RS (61806)
BEARING_6806_OD = 42.0            # [DATASHEET]
BEARING_6806_WIDTH = 7.0          # [DATASHEET]
BEARING_6806_SHOULDER_DIA = 33.0  # [DATASHEET] class: a shoulder that bears on the inner ring only (above the Ø30 bore,
#                                   under the inner ring's outer edge, ~33.7-35 by brand); verify on the bearing in hand
BEARING_6806_MASS_G = 26.0        # [ESTIMATE] the 6808's 33 g scaled by the ring area (42^2 - 30^2) / (52^2 - 40^2); weigh one

# The 90T pulleys of the elbow and the wrist sit this far out along their axes from where the SolidWorks capture put
# them (lib/mounts.py re-seats them), and the stubs they bolt onto reach this much further (lib/forearm/params.py
# RollDriveParams.stub_x, lib/coupler/params.py DEFAULT): SolidWorks mated each hub straight onto its coupler's stub,
# as if there were no bearings; moved out by this much the hub's end lies on the lip's lower face, its Ø30 x 7
# journal in the lower 6806 and its Ø34.76 ring under that bearing's inner ring (tests/test_mounts.py test_bearing_stacks).
PULLEY_SEAT_SHIFT = 3.0          # [REFERENCE] the capture's stub end -> the lip's lower face (elbow and wrist alike)
