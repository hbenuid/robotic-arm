"""PulleyParams - every dimension of the printed 90T GT2 pulley (gt2_pulley_90t: the driven pulley of the elbow and the
wrist belts - its hub turns in the joint's lower 6806 and bolts flat onto the stub beyond it), in its part frame; YAW is
the base_yaw belt's 120T (gt2_pulley_120t), the same pulley with more teeth.

Frame (= the SolidWorks part frame of gt2_pulley_90t, which placements.json and lib/mounts.py place): origin on the axis
at the lower flange's top (where the teeth start), +Y up the axis toward the outer face, the pulley bolts' heads sunk in
it. Every feature is a disc or a bore along Y.

The body: the hub from its end - the Ø30 journal in the lower 6806, a ring under that bearing's inner ring, on up to a
step -, the web on top of it, the toothed rim round the web (the tooth band between two flanges, each chamfered on the
teeth's side; the groove is lib/belts.py's), open underneath between the step and the rim; the bore on the axis and the
4 bolts on the X / Z axes (lib/belts.py pulley_90t_bolt_points()), from the hub's end to the outer face, each with a
counterbore from the face for its head (DEFAULT).

Two configurations of the 90T: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the part builds: the bolt holes at M4 clearance, the heads in
counterbores. YAW is DEFAULT with 120 teeth and no plain hub between its ring and its web (below).

Every number below was measured on the reference 2026-09-27 (face census; tests/pulley/test_gt2_pulley_90t.py re-checks
the builds against it): [REFERENCE] unless tagged. Units mm. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.belts import (
    GT2_PULLEY_90T_BOLT_R,
    GT2_PULLEY_90T_FACE_Y,
    GT2_PULLEY_90T_HEAD_SEAT,
    GT2_PULLEY_90T_TEETH,
    GT2_PULLEY_120T_TEETH,
    pulley_od,
)
from lib.fasteners import M4_CLEAR, M4_SHCS
from lib.sensors import HOME_MAGNET_DIA, HOME_MAGNET_T, MAGNET_POCKET_CLEAR

_AXES = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))


@dataclass(frozen=True)
class PulleyParams:
    teeth: int = GT2_PULLEY_90T_TEETH
    end_y: float = GT2_PULLEY_90T_FACE_Y[0]    # the hub's end, on the stub it bolts to
    face_y: float = GT2_PULLEY_90T_FACE_Y[1]   # the outer face: the upper flange's top and the web's
    flange_t: float = 1.2              # each flange along Y (the lower one under the origin, the upper one under the face) ...
    flange_h: float = 1.2              # ... standing this far out from the land (radially)
    chamfer: float = 0.3               # 45 deg, on each flange's outer edge on the teeth's side
    rim_dia: float = 53.0              # the rim's inside (both flanges and the tooth band): the web inside it ...
    web_y0: float = 1.8                # ... from here up to the face, open underneath
    step: tuple | None = (35.0, 0.8)   # (dia, y0): a step under the web, from y0 up (None: the hub runs on up to the web)
    hub_dia: float = 30.0              # the hub, from its end up to the step (its journal: below the ring)
    ring: tuple = (34.7592031398, -6.2, -4.7)   # (dia, y0, y1): a ring round the hub, under the lower 6806's inner ring
    bore_dia: float = 12.5
    bolt_r: float = GT2_PULLEY_90T_BOLT_R   # the 4 bolts, on the axes ...
    hole_dia: float = 3.9              # ... in holes under an M4's shank (the export's) ...
    counterbore: tuple = (0.0, 0.0)    # ... (dia, depth): a counterbore from the outer face round each, for its head (none)
    magnet: tuple | None = None        # [DESIGN] (r, deg, dia, depth): a blind pocket from the outer face for a disc magnet,
    #                                    its axis r off the pulley's and deg in the XZ plane from +X toward +Z (none)

    @property
    def band_y1(self) -> float:
        """The tooth band's top (it starts at the origin): the upper flange's underside."""
        return self.face_y - self.flange_t

    @property
    def flange_dia(self) -> float:
        return pulley_od(self.teeth) + 2.0 * self.flange_h

    def bolt_points(self) -> list[tuple[float, float]]:
        """(x, z) of the 4 bolts: on the axes at bolt_r."""
        return [(self.bolt_r * a, self.bolt_r * b) for a, b in _AXES]


LEGACY = PulleyParams()     # the SolidWorks part, exactly

# What the part builds: the bolt holes opened to M4 clearance (the export's Ø3.9 sits under an M4's shank) - the bolts
# pass through the pulley into the nuts under the stub -, each head sunk in a counterbore: 0.4 over the head (j1_coupler's
# hub leg's and the drive's housing bolts' 7.4), its floor GT2_PULLEY_90T_HEAD_SEAT under the face (the head 1 under it);
# 1.05 of wall left to the bore.
DEFAULT = replace(LEGACY, hole_dia=M4_CLEAR, counterbore=(M4_SHCS.head_dia + 0.4, GT2_PULLEY_90T_HEAD_SEAT))   # [DESIGN]

# The base_yaw joint's driven pulley (gt2_pulley_120t): DEFAULT with GT2_PULLEY_120T_TEETH - the base_yaw ratio 6:1 on the
# motor's 20T, where the elbow's and the wrist's 90T give 4.5:1 - and its rim's inside grown with the teeth, keeping the
# 90T's wall under them. The journal, the ring, the web's height, the bore, the bolts and their counterbores are the
# 90T's: it sits on j1_coupler's stub in the lower base bearing the same way, clamped by the same screws. Its plain hub
# between the ring and the web (the 90T's Ø30 and its step) is gone - YAW_HUB_CUT: the ring stands straight on the web,
# the journal and the ring that much nearer the outer face. That plain hub only held the belt plane down: without it the
# web's top comes up to the ring's height under the lower bearing - 1.5 under it and under the base's bearing boss, which
# ends flush with it (lib/base/params.py _BOSS_Y0; the rim's inside is wider than the boss, so the rim rises beside it) -,
# and the belt, the motor plate and the base's bottom face come YAW_HUB_CUT nearer the cap with it (lib/base/params.py
# YAW_BELT_Y).
YAW_HUB_CUT = round(LEGACY.web_y0 - LEGACY.ring[2], 6)   # 6.5 [DESIGN]
# The base_yaw home magnet (lib/sensors.py HOME_MAGNET_*, parts/base/yaw_home_magnet) in a pocket in the 120T's outer
# face - which faces down, the pulley hanging under j1_coupler -, its south pole flush with the face, over the KY-003
# hung under the base's motor plate (lib/base/params.py YawHomeParams: its chip on this radius) at one base_yaw angle,
# BASE_YAW_HOME_DEG (lib/params.py, which this angle sets: the pulley turned onto the coupler's holes, lib/mounts.py).
# YAW_MAGNET_R: on the web, the pocket 12.2 off the nearest bolt's counterbore and 5.95 in from the rim's inside. The
# angle centres the switch's zone in the 10 deg between the travel's ends (tests/base/test_yaw_home.py).
YAW_MAGNET_R = 27.0         # [DESIGN]
YAW_MAGNET_DEG = -145.5     # [DESIGN] in the pulley's own frame: base_yaw -169.5 (lib/params.py BASE_YAW_HOME_DEG)
YAW = replace(DEFAULT, teeth=GT2_PULLEY_120T_TEETH,
              rim_dia=round(pulley_od(GT2_PULLEY_120T_TEETH) - (pulley_od(LEGACY.teeth) - LEGACY.rim_dia), 6),
              end_y=round(LEGACY.end_y + YAW_HUB_CUT, 6),
              ring=(LEGACY.ring[0], round(LEGACY.ring[1] + YAW_HUB_CUT, 6), round(LEGACY.ring[2] + YAW_HUB_CUT, 6)),
              step=None,
              magnet=(YAW_MAGNET_R, YAW_MAGNET_DEG, HOME_MAGNET_DIA + 2.0 * MAGNET_POCKET_CLEAR,
                      HOME_MAGNET_T + MAGNET_POCKET_CLEAR))   # [DESIGN]
