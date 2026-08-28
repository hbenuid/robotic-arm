"""cycloidal_ring_gear_body - the housing cylinder: ring-pin holes, 6814 seat, retention lip,
captive nut pockets (local z 0..51; sits at stack z 9..60).

Ported from cycloidal_drive@2f1f67d src/ring_gear_body.py (build_ring_gear_body()); reference/
cycloidal_ring_gear_body.step is that builder's export (kind "designed"). PETG.

Stepped bore: 116 mm (z 0..28, disc orbit + clearance), 90.15 mm 6814 press-fit seat (28..48),
86.15 mm hub clearance behind the integral 2 mm retention lip (48..51). 21 blind ring-pin holes
(31.5 deep) with 1 mm entry funnels at the bore/bearing transition; 8x M4 through-holes with hex
nut pockets on the output face; the shared 8-pillar reveal-window silhouette; the output face
(z=51, external) and barrel edges chamfered, the motor-plate face (z=0) sharp.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Cone, Location, Pos  # noqa: E402  (import after shim)
from lib.cycloidal import (  # noqa: E402
    DEFAULT_CONFIG, DriveConfig, compute_housing_bolt_angles, housing_bolt_points, ring_pin_hole_depth,
    ring_pin_hole_dia, ring_pin_points,
)
from lib.cycloidal.geom import MIN, cylinder, through  # noqa: E402
from lib.cycloidal.housing import chamfer_outer_silhouette, hex_pocket, reveal_window_cutter  # noqa: E402
from lib.params import NUDGE  # noqa: E402

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME
CONVERTED = True
LOCAL_FROM_REF = Location()
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    h, tol, stack = cfg.housing, cfg.tolerances, cfg.stack_up
    height = stack.ring_gear_body_height                # 51
    bore_zone = stack.bore_zone                         # 28
    seat_top = bore_zone + stack.output_bearing_total   # 48

    result = cylinder(h.od / 2.0, height)
    result = result - cylinder(h.bore_dia / 2.0, bore_zone + NUDGE, z0=-NUDGE)                      # 116 bore
    result = result - cylinder(h.output_bearing_seat_dia / 2.0, stack.output_bearing_total, z0=bore_zone)   # 6814 seat
    result = result - cylinder(h.lip_bore_dia / 2.0, height - seat_top + NUDGE, z0=seat_top)         # hub clearance / lip
    # 21 blind ring-pin holes from the input face + entry funnels where the pins leave the bore
    pin_dia = ring_pin_hole_dia(cfg)
    depth = ring_pin_hole_depth(cfg)
    funnel_d, funnel_r = h.ring_pin_entry_chamfer_depth, (pin_dia + h.ring_pin_entry_chamfer_add) / 2.0
    slope = (funnel_r - pin_dia / 2.0) / funnel_d
    for xy in ring_pin_points(cfg):
        result = result - cylinder(pin_dia / 2.0, depth + NUDGE, xy, z0=-NUDGE)
        result = result - Pos(xy[0], xy[1], bore_zone - NUDGE) * Cone(funnel_r + slope * NUDGE, pin_dia / 2.0, funnel_d + NUDGE, align=MIN)
    # 8x M4 through-holes
    m4_r = (h.bolt_dia + tol.bolt_clearance_add) / 2.0
    for xy in housing_bolt_points(cfg):
        result = result - through(m4_r, height, xy)
    # shared outer profile, captive nut pockets on the output face, bevel
    result = result - reveal_window_cutter(cfg, height)
    for angle, xy in zip(compute_housing_bolt_angles(cfg), housing_bolt_points(cfg)):
        result = result - hex_pocket(cfg, xy, angle, h.bolt_nut_depth + NUDGE, z0=height - h.bolt_nut_depth)
    return chamfer_outer_silhouette(result, cfg, external_z=height)


def gen_step():
    """Return the body at its LOCAL origin (input face at z=0; the assembly lifts it to z=9)."""
    part = build()
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
