"""PulleyParams - every dimension of the printed 90T GT2 pulley (gt2_pulley_90t: the driven pulley of the elbow, the
wrist and the base_yaw belts - its hub turns in the joint's lower 6806 and bolts flat onto the stub beyond it), in its
part frame.

Frame (= the SolidWorks part frame of gt2_pulley_90t, which placements.json and lib/mounts.py place): origin on the axis
at the lower flange's top (where the teeth start), +Y up the axis toward the outer face, under the pulley bolts' heads.
Every feature is a disc or a bore along Y.

The body: the hub from its end - the Ø30 journal in the lower 6806, a ring under that bearing's inner ring, on up to a
step -, the web on top of it, the toothed rim round the web (the tooth band between two flanges, each chamfered on the
teeth's side; the groove is lib/belts.py's), open underneath between the step and the rim; the bore on the axis and the
4 bolts on the X / Z axes (lib/belts.py pulley_90t_bolt_points()), from the hub's end to the outer face.

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the part builds: the bolt holes at M4 clearance.

Every number below was measured on the reference 2026-09-27 (face census; tests/pulley/test_gt2_pulley_90t.py re-checks
the builds against it): [REFERENCE] unless tagged. Units mm. Frozen dataclasses; variants via dataclasses.replace.

CompoundPulleyParams - the printed 20-60T compound pulley (gt2_pulley_20_60t), its own frame and provenance on the class.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.belts import (
    GT2_BLEND_R,
    GT2_PULLEY_20_60T_TEETH,
    GT2_PULLEY_90T_BOLT_R,
    GT2_PULLEY_90T_FACE_Y,
    GT2_PULLEY_90T_TEETH,
    pulley_od,
)
from lib.fasteners import M4_CLEAR

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
    step: tuple = (35.0, 0.8)          # (dia, y0): a step under the web, from y0 up
    hub_dia: float = 30.0              # the hub, from its end up to the step (its journal: below the ring)
    ring: tuple = (34.7592031398, -6.2, -4.7)   # (dia, y0, y1): a ring round the hub, under the lower 6806's inner ring
    bore_dia: float = 12.5
    bolt_r: float = GT2_PULLEY_90T_BOLT_R   # the 4 bolts, on the axes ...
    hole_dia: float = 3.9              # ... in holes under an M4's shank (the export's)

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
# pass through the pulley into the nuts under the stub.
DEFAULT = replace(LEGACY, hole_dia=M4_CLEAR)   # [DESIGN]


@dataclass(frozen=True)
class CompoundPulleyParams:
    """The printed 20-60T compound pulley (gt2_pulley_20_60t): two toothed bands on one bore, each between two
    flanges, the upper band's lower flange standing on the lower band's upper flange - solid, no hub, no web.

    Frame (= its SolidWorks part frame): origin on the axis at the lower band's bottom (its lower flange's top), +Y up the
    axis toward the upper band. Measured 2026-09-29 on the SolidWorks export ("GT2 Pulley - 20 - 60 teeth", sha256
    55733855bdc5..., not committed - lib/reference.py MEASURED; tests/pulley/test_gt2_pulley_20_60t.py holds the numbers):
    [REFERENCE] unless tagged. Per band: index 0 the lower band, 1 the upper."""
    teeth: tuple = GT2_PULLEY_20_60T_TEETH
    band_w: float = 7.0                # each tooth band along Y (the belt + 1, as the 90T's)
    flange_t: float = 1.2              # each flange along Y ...
    flange_h: float = 1.2              # ... standing this far out from its band's land (radially)
    chamfer: float = 0.3               # 45 deg, on each flange's outer edge on the teeth's side
    blend_r: tuple = (GT2_BLEND_R, 0.6160365848)   # the groove's blend (lib/pulley/teeth.py): the 60T's is the 90T's,
    #                                                the 20T's the export's own value (the constraint behind it is not in it)
    bore_dia: float = 8.0              # through, end to end

    def band_y0(self, i: int) -> float:
        """Where band i's teeth start (its lower flange's top): the lower band at the origin, the upper one on the
        lower band's upper flange plus its own lower flange."""
        return i * (self.band_w + 2.0 * self.flange_t)

    def flange_dia(self, i: int) -> float:
        return pulley_od(self.teeth[i]) + 2.0 * self.flange_h

    @property
    def end_y(self) -> float:
        """The lower band's lower flange's underside."""
        return -self.flange_t

    @property
    def top_y(self) -> float:
        """The upper band's upper flange's top."""
        return self.band_y0(len(self.teeth) - 1) + self.band_w + self.flange_t


COMPOUND = CompoundPulleyParams()   # the SolidWorks part, exactly - and what the part builds
