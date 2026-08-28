"""cycloidal_motor_plate - the NEMA 17 side half of the drive housing (z 0..9).

Ported from cycloidal_drive@2f1f67d src/motor_plate.py (build_motor_plate()); reference/
cycloidal_motor_plate.step is that builder's export (kind "designed"). PETG.

Outer face z=0 (motor mounting face, external): pilot recess, 4x M3 clearance holes (heads
pocketed from the inner face), 15 mm shaft pass-through; 21 ring-pin through-holes on the 108 mm
circle (pins are inserted one at a time from this face); 8x M4 through-holes with counterbores
on the 125 mm circle; the shared 8-pillar reveal-window silhouette; outer silhouette chamfered.
The inner face z=9 seats on the ring gear body and stays sharp.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from build123d import Location  # noqa: E402  (import after shim)
from lib.cycloidal import (  # noqa: E402
    DEFAULT_CONFIG, DriveConfig, housing_bolt_points, motor_bolt_counterbore_depth, motor_bolt_points,
    ring_pin_hole_dia, ring_pin_points,
)
from lib.cycloidal.geom import cylinder, through  # noqa: E402
from lib.cycloidal.housing import chamfer_outer_silhouette, reveal_window_cutter  # noqa: E402
from lib.params import NUDGE  # noqa: E402

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME
CONVERTED = True
LOCAL_FROM_REF = Location()
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    h, m, tol, stack = cfg.housing, cfg.motor, cfg.tolerances, cfg.stack_up
    t = stack.motor_plate_wall + stack.motor_plate_inner_wall      # 9

    result = cylinder(h.od / 2.0, t)
    # motor pilot recess on the outer face, central shaft pass-through
    result = result - cylinder((m.pilot_dia + 2 * tol.mating_surface_add) / 2.0, m.pilot_height + NUDGE, z0=-NUDGE)
    result = result - through(h.motor_plate_shaft_bore / 2.0, t)
    # 4x M3 clearance + head pockets from the inner face (heads flush with z=9)
    m3_r = (m.bolt_dia + tol.bolt_clearance_add) / 2.0
    cb_r = (m.motor_bolt_head_dia + tol.bolt_clearance_add) / 2.0
    cb_d = motor_bolt_counterbore_depth(cfg)
    for xy in motor_bolt_points(cfg):
        result = result - through(m3_r, t, xy)
        result = result - cylinder(cb_r, cb_d + NUDGE, xy, z0=t - cb_d)
    # 21 ring-pin through-holes (clearance fit; the ring gear body retains the pins)
    pin_r = ring_pin_hole_dia(cfg) / 2.0
    for xy in ring_pin_points(cfg):
        result = result - through(pin_r, t, xy)
    # 8x M4 through-holes + counterbores on the outer face
    m4_r = (h.bolt_dia + tol.bolt_clearance_add) / 2.0
    for xy in housing_bolt_points(cfg):
        result = result - through(m4_r, t, xy)
        result = result - cylinder(h.bolt_counterbore_dia / 2.0, h.bolt_counterbore_depth + NUDGE, xy, z0=-NUDGE)
    # shared outer profile, then the bevel (outer face z=0 is external, inner face z=9 mates)
    result = result - reveal_window_cutter(cfg, t)
    return chamfer_outer_silhouette(result, cfg, external_z=0.0)


def gen_step():
    """Return the plate at its LOCAL origin (= its stack position: outer face at z=0)."""
    part = build()
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
