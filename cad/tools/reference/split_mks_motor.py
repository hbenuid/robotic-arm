"""Turn the two "NEMA 17 + MKS SERVO42D" kit exports (one SolidWorks assembly each: motor body, D-shaft,
driver board + cover, 4 standoffs, 4 screws) into the vendor files the parts import:

    vendor/nema17_40mm.step   (from the x40 export) the motor body (parts/joints/nema17_40mm.py), re-framed like
                              the drive motor (mounting face z=0, body -Z, pilot + shaft +Z, D-flat +Y)
    vendor/mks_servo42d.step  (from the x40 export) the board kit (parts/joints/mks_servo42d.py) - same orientation,
                              z=0 at the motor's REAR face, the board stack in -Z
    vendor/nema17_48mm.step   (from the x48 export) the drive motor (parts/cycloidal/nema17_48mm.py): its 48 mm
                              body - front plate, housing, back plate, its two Ø22 x 7 bearings, cable connector
                              and rotor (7 solids); not its leads, its tie-rod screws (the kit's M3x30 replace
                              them) or its board

Every motor gets THE SAME pilot boss and shaft: the exports' own bosses and shafts (23 mm on the x40; 24 mm with
a 15 mm D-cut on the x48 - a 17HS19-2004S1's) are cut off at the mounting face, and the drive's
lib/cycloidal/motor.py pilot() (Ø22 x 2, bored for the shaft) is fused onto the front plate and its shaft()
(Ø5 x 22: 4 round + 18 D-cut, flat +Y - MotorParams, what the eccentric shaft's D-bore was designed around)
onto the rotor (x48) or added as the motor's second solid (x40).

    ./cadtool python tools/reference/split_mks_motor.py [--write all|kit|drive]
            [--src  ~/Documents/arm_assembly_organized/mks/nema17x40_with_mks.step]
            [--src48 ~/Documents/arm_assembly_organized/mks/nema17x48_with_mks.step]
            [--motor-out vendor/nema17_40mm.step] [--board-out vendor/mks_servo42d.step] [--drive-out vendor/nema17_48mm.step]
    ./cadtool python tools/reference/import_solidworks.py               # then: the kit parts' reference copies + manifest entries
    ./cadtool python tools/cycloidal/import_cadquery.py --only nema17_48mm   # and the drive motor's vendor block
    ./cadtool gen parts/<group>/<name>.py --force                       # a NEW vendor file is not yet a tracked input

The solids are told apart by GEOMETRY, not by the exports' product labels (NAUO ids): the shaft is the Ø5
solid, the body the largest one (x40) / the three 42-square solids (x48), the kit everything behind the body,
the tie rods the solids centred on the bolt pattern, the front plate the solid with the mounting face. The x48
export is authored in centimetres and sits ~(673, 881, 1302) mm off the origin; OCCT converts, the tool
re-frames from the measured axis and mounting face. build123d's STEP writer stamps the time into the header,
so every run writes new bytes: the files are written ONCE (here) and committed as Git LFS inputs - `--write
kit|drive` leaves the others alone. The raw exports stay outside the repo next to the monolith
(lib.reference.MKS_EXPORT_NAME / MKS48_EXPORT_NAME); reference/README.md records their sha256.
"""
from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys

from build123d import Align, Box, Compound, GeomType, Location, export_step, import_step
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

from lib import reference as R
from lib.cycloidal import DEFAULT_CONFIG, motor_bolt_points
from lib.cycloidal.motor import pilot, shaft
from lib.geom import single_solid

MOTOR_NAME, BOARD_NAME, DRIVE_NAME = "nema17_40mm", "mks_servo42d", "nema17_48mm"
TO_PART = Location((0.0, 0.0, 0.0), (-90.0, 0.0, 0.0))   # export frame (face y=0, body +Y, shaft -Y) -> part frame (body -Z, shaft +Z)
BODY_TOL = 0.5                                            # a "42-square" solid: both cross extents within this of the body width


def bbox(shape):
    bb = shape.bounding_box()
    return (bb.min.X, bb.min.Y, bb.min.Z), (bb.size.X, bb.size.Y, bb.size.Z)


def fmt(v):
    return "(" + ", ".join(f"{x:.3f}" for x in v) + ")"


def cylinders(shape):
    """(radius, axis point, axis direction) of every cylindrical face of `shape` (world frame)."""
    out = []
    for f in shape.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() == GeomAbs_Cylinder:
            c = ad.Cylinder()
            p, d = c.Axis().Location(), c.Axis().Direction()
            out.append((c.Radius(), (p.X(), p.Y(), p.Z()), (d.X(), d.Y(), d.Z())))
    return out


def shaft_axis(solid, m):
    """The (x, z) of the motor axis in the export frame, from the solid's Ø5 shaft cylinder (axis along Y)."""
    for r, p, d in cylinders(solid):
        if abs(r - m.shaft_dia / 2.0) < 0.05 and abs(abs(d[1]) - 1.0) < 1e-6:
            return p[0], p[2]
    return None


def planar_faces(shape, normal, min_area=1.0):
    """Planar faces of `shape` whose normal is `normal` (within 1e-6), largest first."""
    out = []
    for f in shape.faces().filter_by(GeomType.PLANE):
        n = f.normal_at()
        if all(abs(a - b) < 1e-6 for a, b in zip((n.X, n.Y, n.Z), normal, strict=True)) and f.area >= min_area:
            out.append(f)
    return sorted(out, key=lambda f: f.area, reverse=True)


def face_area_at(shape, z: float) -> float:
    """Total area of the +Z planar faces of `shape` on the plane z."""
    return sum(f.area for f in planar_faces(shape, (0, 0, 1)) if abs(f.center().Z - z) < 1e-3)


def with_drive_interface(solids, m):
    """`solids` (part frame): everything above the mounting face - the export's own pilot boss and shaft - cut
    off, the drive's pilot() fused onto the front plate (the solid carrying the mounting face). Returns the new
    list (same order) and the front plate's index."""
    cutter = Box(3 * m.body_width, 3 * m.body_width, 3 * m.body_length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    out = [single_solid(s - cutter) if bbox(s)[0][2] + bbox(s)[1][2] > 1e-6 else s for s in solids]
    front = max(range(len(out)), key=lambda i: face_area_at(out[i], 0.0))
    if face_area_at(out[front], 0.0) < 800:
        raise SystemExit("no mounting face on z = 0 after the cut")
    out[front] = single_solid(out[front] + pilot(m, bore=m.shaft_dia))
    return out, front


def check_interface(shape, m, what: str):
    """The drive's interface: shaft tip at +shaft_length, the D-flat on +Y, the pilot face at +pilot_height."""
    lo, size = bbox(shape)
    if abs(lo[2] + size[2] - m.shaft_length) > 1e-6:
        raise SystemExit(f"{what}: shaft tip at {lo[2] + size[2]:.3f}, expected {m.shaft_length:g}")
    flats = [f for f in shape.faces().filter_by(GeomType.PLANE)
             if abs(f.normal_at().Z) < 1e-6 and abs(f.center().X) < 2.6 and abs(f.center().Y) < 2.6 and f.area > 5]
    if len(flats) != 1 or max(abs(flats[0].normal_at().X), abs(flats[0].normal_at().Y - 1.0)) > 1e-6:
        raise SystemExit(f"{what}: expected one D-flat facing +Y, found {len(flats)}")
    zs = [v.Z for v in flats[0].vertices()]
    if abs(max(zs) - min(zs) - m.shaft_dcut_length) > 1e-6 or abs(max(zs) - m.shaft_length) > 1e-6:
        raise SystemExit(f"{what}: D-cut z {min(zs):.3f}..{max(zs):.3f}, expected the last {m.shaft_dcut_length:g} mm")
    if face_area_at(shape, m.pilot_height) < math.pi * ((m.pilot_dia / 2.0) ** 2 - (m.shaft_dia / 2.0) ** 2) - 1.0:
        raise SystemExit(f"{what}: no Ø{m.pilot_dia:g} pilot face at z = {m.pilot_height:g}")


def split_kit(src: pathlib.Path, m):
    """The x40 export -> (motor Compound, board Compound)."""
    print(f"importing {src.name} ({src.stat().st_size / 1e3:.0f} kB, sha256 {R.sha256(src)[:12]}) ...")
    solids = import_step(str(src)).solids()

    def across(s):
        return sorted(bbox(s)[1])[:2]   # the two smaller bbox extents: 5 x 5 for the shaft

    shafts = [s for s in solids if all(abs(v - m.shaft_dia) < 0.05 for v in across(s)) and s.volume < 1000]
    if len(shafts) != 1:
        raise SystemExit(f"expected one Ø{m.shaft_dia:g} shaft solid, found {len(shafts)}")
    export_shaft = shafts[0]
    body = max((s for s in solids if s is not export_shaft), key=lambda s: s.volume)
    kit = [s for s in solids if s is not export_shaft and s is not body]
    print(f"  {len(solids)} solids: body {body.volume:.1f} mm^3 {fmt(bbox(body)[1])}, export shaft {export_shaft.volume:.1f} mm^3 "
          f"(tip {-bbox(export_shaft)[0][1]:.3f} from the face - replaced), kit {len(kit)} solids")

    # The export's frame: the mounting face is the body's largest -Y face and must sit at y = 0.
    face_y = planar_faces(body, (0, -1, 0))[0].center().Y
    rear_y = planar_faces(body, (0, 1, 0))[0].center().Y
    if abs(face_y) > 1e-6:
        raise SystemExit(f"the export's mounting face is at y = {face_y}, expected 0 - re-export with the origin on it")
    print(f"  mounting face y = {face_y:.6f}, rear face y = {rear_y:.3f} (body length {rear_y - face_y:.3f})")

    (body_part,), _ = with_drive_interface([body.moved(TO_PART)], m)
    motor = Compound([body_part, shaft(m)])
    motor.label = MOTOR_NAME
    check_interface(motor, m, MOTOR_NAME)
    board = Compound([s.moved(Location((0.0, 0.0, rear_y)) * TO_PART) for s in kit])
    board.label = BOARD_NAME

    lo, size = bbox(motor)
    if not (len(motor.solids()) == 2 and lo[2] < -rear_y and abs(lo[0] + size[0] / 2.0) < 1e-6):
        raise SystemExit(f"motor geometry check failed: {len(motor.solids())} solids, bbox min {fmt(lo)} size {fmt(size)}")
    lo, size = bbox(board)
    if not (len(board.solids()) == len(kit) and lo[2] < 0 < lo[2] + size[2]
            and abs(lo[0] + size[0] / 2.0) < 1e-6 and abs(lo[1] + size[1] / 2.0) < 1e-6):
        raise SystemExit(f"board geometry check failed: {len(board.solids())} solids, bbox min {fmt(lo)} size {fmt(size)}")
    return motor, board


def compose_drive_motor(src48: pathlib.Path, m):
    """The x48 export's motor body with the drive's pilot and shaft -> the drive motor Compound."""
    print(f"importing {src48.name} ({src48.stat().st_size / 1e6:.1f} MB, sha256 {R.sha256(src48)[:12]}) ...")
    solids = import_step(str(src48)).solids()
    rotors = [(s, shaft_axis(s, m)) for s in solids]
    rotors = [(s, a) for s, a in rotors if a is not None]
    if len(rotors) != 1:
        raise SystemExit(f"expected one solid carrying the Ø{m.shaft_dia:g} shaft, found {len(rotors)}")
    rotor, (ax, az) = rotors[0]
    bodies = [s for s in solids if all(abs(v - m.body_width) < BODY_TOL for v in (bbox(s)[1][0], bbox(s)[1][2]))]
    if len(bodies) != 3:
        raise SystemExit(f"expected the 3 square body solids (front plate, housing, back plate), found {len(bodies)}")
    mount_y = min(bbox(s)[0][1] for s in bodies) + m.pilot_height     # the export's pilot boss stands on the mounting face
    rear_y = max(bbox(s)[0][1] + bbox(s)[1][1] for s in bodies)
    if abs(rear_y - mount_y - m.body_length) > 0.05:
        raise SystemExit(f"body length {rear_y - mount_y:.3f} != {m.body_length:g}")
    print(f"  {len(solids)} solids; axis x={ax:.3f} z={az:.3f}, mounting face y={mount_y:.3f}, rear y={rear_y:.3f}; "
          f"export shaft tip {mount_y - bbox(rotor)[0][1]:.3f} from the face - replaced")

    half = m.body_width / 2.0 + BODY_TOL
    bolts = motor_bolt_points()

    def keep(s):
        """Inside the body envelope (not the leads, not the board kit behind the rear face), not a tie rod."""
        lo, size = bbox(s)
        cx, cz = lo[0] + size[0] / 2.0 - ax, lo[2] + size[2] / 2.0 - az
        inside = (lo[0] >= ax - half and lo[0] + size[0] <= ax + half and lo[2] >= az - half - 0.5 and lo[2] + size[2] <= az + half + 0.5
                  and lo[1] >= mount_y - m.pilot_height - 0.1 and lo[1] + size[1] <= rear_y + 0.1)
        tie_rod = any(math.hypot(cx - bx, cz - bz) < 1.0 for bx, bz in bolts)
        return inside and not tie_rod

    to_part = TO_PART * Location((-ax, -mount_y, -az))
    kept = [s.moved(to_part) for s in solids if s is not rotor and keep(s)]
    kept, _ = with_drive_interface(kept, m)
    rotor_part = single_solid(rotor.moved(to_part) - Box(3 * m.body_width, 3 * m.body_width, 3 * m.body_length,
                                                         align=(Align.CENTER, Align.CENTER, Align.MIN)))
    rotor_shaft = single_solid(rotor_part + shaft(m))
    drive = Compound(kept + [rotor_shaft])
    drive.label = DRIVE_NAME
    print(f"  kept {len(kept)} body solids + the rotor with the drive shaft = {len(drive.solids())} solids, "
          f"dropped {len(solids) - len(kept) - 1}")
    check_interface(drive, m, DRIVE_NAME)

    # The reference's bounding box (tests/test_parts_convention.py::test_cots_vendor_matches_reference_frame, 1.5 mm).
    lo, size = bbox(drive)
    ref_lo, ref_size = bbox(R.load(DRIVE_NAME))
    if not (abs(lo[2] + m.body_length) < 1e-6
            and all(abs(a - b) <= 1.5 for a, b in zip(lo, ref_lo, strict=True)) and all(abs(a - b) <= 1.5 for a, b in zip(size, ref_size, strict=True))):
        raise SystemExit(f"drive motor bbox min {fmt(lo)} size {fmt(size)} vs reference {fmt(ref_lo)} {fmt(ref_size)}")
    return drive


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src_dir = pathlib.Path(os.environ.get("ARM_REFERENCE_SRC", R.DEFAULT_SOURCE_DIR))
    ap.add_argument("--src", type=pathlib.Path, default=src_dir / R.MKS_EXPORT_NAME, help="the x40 kit export")
    ap.add_argument("--src48", type=pathlib.Path, default=src_dir / R.MKS48_EXPORT_NAME, help="the x48 kit export")
    ap.add_argument("--write", choices=("all", "kit", "drive"), default="all",
                    help="which vendor files to write (the writer stamps the time: rewriting changes bytes)")
    ap.add_argument("--motor-out", type=pathlib.Path, default=R.VENDOR_DIR / f"{MOTOR_NAME}.step")
    ap.add_argument("--board-out", type=pathlib.Path, default=R.VENDOR_DIR / f"{BOARD_NAME}.step")
    ap.add_argument("--drive-out", type=pathlib.Path, default=R.VENDOR_DIR / f"{DRIVE_NAME}.step")
    args = ap.parse_args(argv)
    m = DEFAULT_CONFIG.motor
    sources = ((args.src,) if args.write != "drive" else ()) + ((args.src48,) if args.write != "kit" else ())
    for src in sources:
        if not src.expanduser().exists():
            print(f"kit export not found: {src}", file=sys.stderr)
            return 1

    outputs = []
    if args.write != "drive":
        motor, board = split_kit(args.src.expanduser(), m)
        outputs += [(motor, args.motor_out), (board, args.board_out)]
    if args.write != "kit":
        outputs.append((compose_drive_motor(args.src48.expanduser(), m), args.drive_out))
    for shape, out in outputs:
        lo, size = bbox(shape)
        print(f"  {shape.label}: {len(shape.solids())} solids, volume {R.solid_volume(shape):.3f} mm^3, bbox min {fmt(lo)} size {fmt(size)}")
        out.parent.mkdir(exist_ok=True)
        export_step(shape, str(out))
        print(f"wrote {out}: {len(shape.solids())} solids")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
