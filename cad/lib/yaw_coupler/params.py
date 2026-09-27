"""YawCouplerConfig - every dimension of the base_yaw coupler, j1_coupler (the yoke that turns on the base about the
base_yaw axis and cradles the cycloidal drive's housing - the shoulder_pitch stator), in its part frame.

Frame (= the SolidWorks part frame of j1_coupler, which placements.json places): origin on the base_yaw axis at the
height of the base's seat-ring top (lib/base/params.py CapParams.ring_top_y), +Y up the axis; the drive's axis (the
shoulder_pitch axis) runs along X at (y = YokeParams.axis_y, z = 0), its motor plate toward +X, its output hub
through the -X cheek.

The body, bottom up: the hub on the axis (a stub down into the upper base bearing, a bore, 4 small holes) under a
drafted disc (flats at x = +/-40, an ear of the Ø96 disc beyond each flat) with a recess in its underside; a ring on
the disc, then the yoke flaring out of it - a -X cheek (outboard of the housing's output face) and a middle body that
holds the housing's bottom and +/-45 degree pillars (a channel and two V-grooves), all under the housing's cradle,
the housing's 3 bottom bolts' nut pockets in the cheek's outer face, a pocket open to the cradle over the hub.
The yoke's shapes follow the drive's housing (lib/cycloidal/params.py HousingParams: the cradle r against the bore,
the V-grooves on the pillars' sides, their points on the housing's od, the nut pockets on its bolt circle -
tests/yaw_coupler/ checks they agree).

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the part builds.

Every number below was measured on the reference 2026-09-27 (face census; tests/yaw_coupler/test_j1_coupler.py
re-checks the builds against it): [REFERENCE] unless tagged. Three simplifications, all inside the reference-match
tolerance: the yoke's +/-Z outer faces are vertical and symmetric through the points on the housing's od (the
reference's lean 0.44 / 0.36 degrees), the flare is one cone over the whole yoke (the reference's last 3 mm toward +X
are the section at x 28.5 carried on), the channel has no 0.2 mm corner bevels. Units mm, degrees where named
*_deg. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class HubParams:
    """The underside and the hub on the axis: a recess in the disc's underside (inside the rim that stands on the
    base's top face), the stub down into the upper base bearing (a round on its end), the bore, 4 small holes on the
    diagonals - bore and holes from the stub's end up to the pocket's floor."""

    recess_dia: float = 90.05
    recess_y1: float = 0.5           # the recess's ceiling (the rim below it: DiscParams.y0)
    stub_dia: float = 29.8
    stub_y0: float = -8.2            # the stub's end (up to the recess's ceiling)
    stub_round: float = 1.0          # r of the round on the end's outer edge
    bore_dia: float = 15.0
    hole_dia: float = 3.3
    hole_r: float = 10.600708        # 4 holes on this r ...
    hole_deg: float = 45.0           # ... the first at this angle (from +X toward +Z), 90 apart


@dataclass(frozen=True)
class DiscParams:
    """The disc: a band of band_r from the underside (y0) up to band_y1, then drafted in to top_r at top_y; cut by
    the flats x = +/- flat_x (the +X one on up through the ring). Beyond the flats the ears of an ear_r disc up to
    ear_y1. On the disc a ring of ring_r up to ring_y1, from x = ring_x0 (the cheek's outer face) to the +X flat."""

    y0: float = -0.4                 # the underside (the rim, flush on the base's top face)
    band_r: float = 53.082571
    band_y1: float = 0.0
    top_r: float = 50.843625
    top_y: float = 19.98843
    flat_x: float = 40.0
    ear_r: float = 48.0
    ear_y1: float = 8.0
    ring_r: float = 40.728657
    ring_y1: float = 24.82211
    ring_x0: float = -32.8


@dataclass(frozen=True)
class YokeParams:
    """The yoke on the ring, under the housing's cradle: a flare (a cone out of the ring's top edge, its apex on the
    axis at flare_apex_y) bounds its underside; the -X cheek (cheek_x) and the middle body (on to body_x1) are
    prisms along X between the outer faces z = +/- outer_z and the cradle (cradle_r about the drive's axis). Their
    tops, as points on circles about the drive's axis (given by |z|): the cheek from its outer face straight to
    cheek_top_z on the cradle - across the +/-45 degree pillars' far side; the middle body in a V-groove round each
    pillar - its end (a chord on the housing's od, groove_z[1] .. the outer face) and its near side (down to
    groove_z[0] on the cradle). The channel under the bottom pillar (its floor, and walls that open at the pillar's
    side slope) runs from the cheek to channel_x1; the notch round the bottom housing bolt's head from channel_x1 to
    the +X flat. The pocket over the hub, open into the cradle. The nut pockets of 3 housing bolts (the bottom one
    and those at +/- 45 degrees, on bolt_circle_r) in the cheek's outer face, a hole on through the rest of it."""

    flare_apex_y: float = 10.285583
    cheek_x: tuple = (-32.8, -28.5)
    body_x1: float = 31.5
    outer_z: float = 53.131924
    axis_y: float = 90.0             # the drive's axis (along X)
    cradle_r: float = 58.0           # the housing's bore is 116: the cradle is its circle
    od_r: float = 70.0               # the housing's od / 2: the points of the V-grooves' ends
    cheek_top_z: float = 46.826264   # on the cradle
    groove_z: tuple = (34.224206, 45.574176)   # on the cradle, on od_r
    channel_floor_y: float = 20.204293
    channel_half_z: float = 5.552097           # at the floor ...
    channel_slope: float = 2.0 / 7.0           # ... opening dz / dy (the pillar's sides)
    channel_x1: float = 32.7
    notch_r: float = 4.817811
    notch_y: float = 27.5
    pocket_half: tuple = (15.2, 31.0)          # x, z
    pocket_y0: float = 8.0
    bolt_circle_r: float = 62.5
    bolt_deg: tuple = (-45.0, 0.0, 45.0)       # from straight down (-Y), toward +Z
    nut_af: float = 7.196671
    nut_depth: float = 4.0
    bolt_hole_dia: float = 4.4


@dataclass(frozen=True)
class YawCouplerConfig:
    hub: HubParams = HubParams()
    disc: DiscParams = DiscParams()
    yoke: YokeParams = YokeParams()


LEGACY = YawCouplerConfig()     # the SolidWorks part, exactly
DEFAULT = LEGACY                # what the part builds
