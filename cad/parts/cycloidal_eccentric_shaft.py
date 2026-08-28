"""cycloidal_eccentric_shaft - converts motor rotation into the discs' wobble.

Ported from cycloidal_drive@2f1f67d src/eccentric_shaft.py (build_eccentric_shaft()); reference/
cycloidal_eccentric_shaft.step is that builder's export (kind "designed"). PETG, 100 % infill.

Two 17.10 mm lobes 180 deg apart (one per disc, centres at +/-e), a 5 mm spine, a ruled-loft
bridge/retention flange (23.10 mm) between the lobes, a 10 mm input collar with the motor-shaft
D-bore, and a blind hole for the output-side 5 mm support dowel. Like the source, the geometry is
built at its STACK position (z 9..35, datum = the motor-plate outer face), so the assembly places
it with an identity Location.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Box, Circle, Location, Pos, loft  # noqa: E402  (import after shim)
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig  # noqa: E402
from lib.cycloidal.geom import cylinder, single_solid  # noqa: E402
from lib.params import NUDGE  # noqa: E402

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME
CONVERTED = True
LOCAL_FROM_REF = Location()
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    shaft, tol, stack = cfg.shaft, cfg.tolerances, cfg.stack_up
    disc_t = cfg.disc.thickness
    e = shaft.eccentricity
    lobe_r, spine_r, collar_r = shaft.bearing_seat_od / 2.0, shaft.spine_od / 2.0, shaft.input_collar_od / 2.0
    z_start, z_lobe1, z_lobe2 = stack.z_motor_plate_inner, stack.z_disc1, stack.z_disc2   # 9, 13, 25
    z_end = z_lobe2 + disc_t                                                           # 35 (output face)

    spine = cylinder(spine_r, z_end - z_start, z0=z_start)
    collar = cylinder(collar_r, z_lobe1 - z_start, z0=z_start)          # input_clearance long (4)
    lobe1 = cylinder(lobe_r, disc_t, (+e, 0.0), z_lobe1)
    lobe2 = cylinder(lobe_r, disc_t, (-e, 0.0), z_lobe2)
    # Bridge + retention flange: ruled loft from lobe 1's centre (+e) to lobe 2's (-e) across the
    # inter-disc spacer, oversized so the two 6003 bearings cannot slide toward each other.
    z_bridge = z_lobe1 + disc_t
    flange_r = shaft.bridge_flange_od / 2.0
    bridge = loft(
        [Pos(+e, 0, z_bridge) * Circle(flange_r), Pos(-e, 0, z_bridge + cfg.disc.inter_disc_spacer) * Circle(flange_r)],
        ruled=True,
    )
    result = spine + collar + lobe1 + lobe2 + bridge

    # Blind hole for the steel support dowel, from the output face (z 24..35).
    pin_bore_r = (shaft.support_pin_dia + 2 * tol.dowel_bore_clearance_add) / 2.0     # 5.15 / 2
    depth = shaft.support_pin_hole_depth
    result = result - cylinder(pin_bore_r, depth + NUDGE, z0=z_end - depth)

    # Motor-shaft D-bore from the input face: round bore minus the half-space beyond the flat
    # (identical to the source's cut-then-union of the 0.25 mm key sliver, without the sliver).
    bore_r = (shaft.d_bore_dia + 2 * tol.d_bore_clearance_add) / 2.0     # 5.13 / 2
    flat_y = shaft.d_bore_flat / 2.0 + tol.d_bore_clearance_add          # 2.315, flat on +Y
    bore = cylinder(bore_r, shaft.d_bore_depth + NUDGE, z0=z_start - NUDGE)
    if bore_r > flat_y:
        keep = Pos(0, flat_y + 5.0, z_start + shaft.d_bore_depth / 2.0) * Box(10.0, 10.0, shaft.d_bore_depth + 4 * NUDGE)
        bore = bore - keep
    result = result - bore
    return single_solid(result)


def gen_step():
    """Return the shaft at its stack position (z 9..35 - see the module docstring)."""
    part = build()
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
