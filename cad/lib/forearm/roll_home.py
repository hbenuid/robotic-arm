"""build123d builder of the forearm_roll home sensor's carrier (lib/forearm/params.py RollHomeParams;
parts/joints/roll_home_carrier), in the module frame of forearm_roll_drive: the KY-003's plate behind the roll shaft, a
leg back out of the bay's bore, a strip down the back panel's opening's floor to its bottom, the notch the panel's rim
keys, the tube that plugs into the frame's wire hole (lib/forearm/roll.py home_*). Imported explicitly by the part, so
importing lib.forearm never pulls in OCCT."""
from __future__ import annotations

import build123d as bd

from lib.datum import to_location
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.forearm.roll import (
    back_point,
    home_board_points,
    home_sensor_frame,
    home_strip_x,
    home_wire_axis,
    teardrop_along,
)
from lib.geom import box, cylinder, hull_solid, single_solid
from lib.units import NUDGE


def _on_floor(x, s, depth, cfg: ForearmConfig):
    """A box on the rear face's frame (roll.py back_point): module x over x, along the face over s, depth (in from the
    rear face) over depth - the opening's floor is at its opening_depth."""
    return hull_solid([back_point(xx, ss, dd, cfg) for xx in x for ss in s for dd in depth])


def build_home_carrier(cfg: ForearmConfig = DEFAULT):
    """The carrier, one solid in the module frame (RollHomeParams): the board's plate under the KY-003's sensor end
    (plate_x, plate_w, plate_t) with its 2 M2 teardrop holes (peaks toward +X: up as it prints on its -X side) and the
    slot under the chip's leads' tails, a pocket under each M2's head; the leg - the hull of the plate's lower edge and the strip's top end on the
    floor's plane inside the bay's bore (knee_s); the strip down the opening's floor to floor_gap off its bottom wall; the
    rim's notch - a lip under the back panel's rim's bottom run and a block beside it, key_gap off it -; a boss round the
    tube's root; the tube (tube_od) down and back along the wire hole's axis, its bore (tube_id) the lead's way, from the
    boss's upper face out through the tube's end. Every part of it spans the strip's x (home_strip_x), the tube flush
    with its -X side."""
    b, h = cfg.drive.back, cfg.drive.home
    sensor = to_location(home_sensor_frame(cfg))
    x0, x1 = home_strip_x(cfg)
    d0, d1 = b.opening_depth - h.plate_t, b.opening_depth
    s0 = b.opening_bottom + h.floor_gap
    plate = sensor * box(h.plate_x, (-h.plate_w / 2.0, h.plate_w / 2.0), (-h.plate_t, 0.0))
    edge = [sensor * bd.Location((x, y, z)) for x in h.plate_x for y in (-h.plate_w / 2.0, -h.plate_w / 2.0 + h.plate_t)
            for z in (-h.plate_t, 0.0)]
    knee = [back_point(x, s, dd, cfg) for x in (x0, x1) for s in h.knee_s for dd in (d0, d1)]
    leg = hull_solid([(p.position.X, p.position.Y, p.position.Z) for p in edge] + knee)
    strip = _on_floor((x0, x1), (s0, h.knee_s[1]), (d0, d1), cfg)
    rim_top = b.t + b.rim_h
    s_in = b.opening_bottom + b.rim_clear + b.rim_w + h.key_gap
    lip = _on_floor((x0, x1), (s0, s_in - h.key_gap - 0.1), (rim_top + h.key_gap, d0 + 0.5), cfg)
    block = _on_floor((x0, x1), (s_in, s_in + h.key_block), (rim_top - 1.2, d0 + 0.5), cfg)
    boss = _on_floor((x0, x1), (s0, s0 + 6.0), (rim_top + h.key_gap, d1), cfg)
    p, u, _n = home_wire_axis(cfg)
    axis = bd.Plane(origin=p, z_dir=u)
    tube = bd.extrude(axis * bd.Circle(h.tube_od / 2.0), amount=h.tube_len)
    body = plate + leg + strip + lip + block + boss + tube
    back = bd.Vector(*p) - bd.Vector(*u) * 6.0                      # the bore from out past the boss's upper face
    body = body - bd.extrude(bd.Plane(origin=back, z_dir=u) * bd.Circle(h.tube_id / 2.0), amount=h.tube_len + 6.0 + 1.0)
    rx0, rx1, rlen, rdepth = h.lead_relief
    body = body - sensor * box((rx0, rx1), (-rlen / 2.0, rlen / 2.0), (-rdepth, NUDGE))
    head_r, head_h = h.board_screw.head_dia / 2.0 + h.head_clear, h.board_screw.head_h + h.head_clear
    for x, y in home_board_points(cfg):
        body = body - sensor * cylinder(head_r, head_h, (x, y), z0=-h.plate_t - head_h)
        start = (sensor * bd.Location((x, y, -h.plate_t - 1.0))).position
        body = body - teardrop_along((start.X, start.Y, start.Z), (0.0, 0.0, 1.0), h.board_hole_dia / 2.0, h.plate_t + 2.0, (1.0, 0.0, 0.0))
    return single_solid(body)


def tube_end(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float]:
    """The tube's far end on its axis (module): it lies in the frame's wire hole."""
    p, u, _n = home_wire_axis(cfg)
    t = cfg.drive.home.tube_len
    return (p[0] + u[0] * t, p[1] + u[1] * t, p[2] + u[2] * t)

