"""CouplerParams - every dimension of the J3 coupler (j3_coupler: the driven side of a belt joint - the 90T pulley bolts
onto its stub's end, the stub turns in the link's bearing bore), in its part frame.

Frame (= the SolidWorks part frame of j3_coupler, which placements.json places): origin on the joint axis at the
flange's underside, +Y up the axis toward the stub's end (the 90T's face). Every feature is a disc or a bore along Y.

The body: a Ø78 flange (4 counterbored M4 on the axes into the wrist link), a dust-lip ring on its top that turns in
the link's Ø80 recess, the Ø40 journal just inside the link's bore, the Ø30 stub through the bearings, the Ø12.5 bore
on the axis; the 90T's 4x M4 on the axes run from hex nut pockets in the flange's underside up through the stub's end.
The forearm roll drive's elbow block repeats the lip / boss / journal / stub at the elbow (lib/forearm/params.py
RollDriveParams; j3_coupler#1 is retired).

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the part builds: an inner-ring shoulder on the stub, the stub long
enough to reach the re-seated pulley.

Every number below was measured on the reference 2026-09-25 (vertex / face census; tests/coupler/test_j3_coupler.py
re-checks the builds against it): [REFERENCE] unless tagged. Units mm. Frozen dataclasses; variants via
dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import BEARING_6806_SHOULDER_DIA, PULLEY_SEAT_SHIFT


@dataclass(frozen=True)
class CouplerParams:
    flange_dia: float = 78.0
    flange_y1: float = 10.0
    lip_dia: tuple = (58.0, 62.0)      # the dust-lip ring: its inside and outside ...
    lip_y1: float = 14.0               # ... from the flange's top up to here
    journal_dia: float = 40.0
    journal_y1: float = 15.3
    stub_dia: float = 30.0
    stub_y1: float = 22.0              # the stub's end: the 90T's mating face
    bore_dia: float = 12.5
    pulley_bolt_r: float = 11.0        # the 90T's 4x M4, on the axes ...
    pulley_bolt_dia: float = 4.1
    nut_af: float = 6.85               # ... their nuts in hex pockets in the flange's underside, a corner along +/-Z
    nut_depth: float = 2.8
    pulley_screw_len: float = 50.0     # [DESIGN] M4 x 50 (ISO 4762, parts/joints/wrist_pulley_screws) from the 90T's outer face: 46.8
    #                                    ends flush with the nut's outer face (its 3.2 in the 2.8 pocket), 50 runs 3.2 past it
    flange_bolt_r: float = 35.0        # the flange's 4x M4, on the axes ...
    flange_bolt_dia: float = 4.1
    counterbore_dia: float = 7.1       # ... their heads in counterbores from the flange's top ...
    counterbore_y0: float = 6.0        # ... down to here
    step: tuple | None = None          # (dia, y1): an inner-ring shoulder from the journal's face up to y1, the stub above it


LEGACY = CouplerParams()     # the SolidWorks part, exactly

# What the part builds: a shoulder on the journal's face down onto the upper wrist bearing's inner ring (the journal's
# Ø40 face stopped 0.7 above the bearing and would have borne on its outer ring too), and the stub PULLEY_SEAT_SHIFT
# longer, on through the lip to the re-seated wrist 90T, which bolts flat onto its end - so the bolts clamp both inner
# rings (the pulley's ring below, the shoulder above), spaced as the lip spaces the outer rings
# (tests/test_mounts.py test_bearing_stacks).
DEFAULT = replace(LEGACY, step=(BEARING_6806_SHOULDER_DIA, 16.0),   # [DESIGN] y1: the upper wrist bearing's top face
                  stub_y1=LEGACY.stub_y1 + PULLEY_SEAT_SHIFT)       # [DESIGN] the lip's lower face: the pulley's hub end
