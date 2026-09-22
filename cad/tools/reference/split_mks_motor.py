"""Turn the two "NEMA 17 + MKS SERVO42D" kit exports (one SolidWorks assembly each: motor body, D-shaft,
driver board + cover, 4 standoffs, 4 screws) into the vendor files the parts import:

    vendor/nema17_40mm.step   (from the x40 export) the motor body + shaft (parts/joints/nema17_40mm.py) - re-framed
                              like the drive motor (mounting face z=0, body -Z, pilot + shaft +Z, D-flat +Y) and the
                              shaft trimmed to MotorParams.shaft_length (22; the export's is 23)
    vendor/mks_servo42d.step  (from the x40 export) the board kit (parts/joints/mks_servo42d.py) - same orientation,
                              z=0 at the motor's REAR face, the board stack in -Z
    vendor/nema17_48mm.step   (from BOTH) the drive motor (parts/cycloidal/nema17_48mm.py): the x48 export's 48 mm
                              body - front plate, housing, back plate, its two Ø22 x 7 bearings, cable connector and
                              rotor (7 solids); not its leads, its tie-rod screws (the kit's M3x30 replace them) or its
                              board - with the rotor's 24 mm / 15 mm-D-cut shaft (a 17HS19-2004S1's) cut off at the
                              mounting face and the x40's shaft (the drive-spec Ø5 / 4.5 flat / 18 mm D-cut, trimmed
                              to 22) fused on instead, in the same frame

    ./cadtool python tools/reference/split_mks_motor.py [--write all|kit|drive]
            [--src  ~/Documents/arm_assembly_organized/mks/nema17x40_with_mks.step]
            [--src48 ~/Documents/arm_assembly_organized/mks/nema17x48_with_mks.step]
            [--motor-out vendor/nema17_40mm.step] [--board-out vendor/mks_servo42d.step] [--drive-out vendor/nema17_48mm.step]
    ./cadtool python tools/reference/import_solidworks.py               # then: the kit parts' reference copies + manifest entries
    ./cadtool python tools/cycloidal/import_cadquery.py --only nema17_48mm   # and the drive motor's vendor block

The solids are told apart by GEOMETRY, not by the exports' product labels (NAUO ids): the shaft is the Ø5
solid, the body the largest one (x40) / the three 42-square solids (x48), the kit everything behind the body,
the tie rods the solids centred on the bolt pattern. The x40's shaft carries its D-flat spun ~3.44 deg about
the axis (the rotor was modelled at an angle) - only the shaft is un-spun, the bodies' squares stay
axis-aligned. The x48 export is authored in centimetres and sits ~(673, 881, 1302) mm off the origin; OCCT
converts, the tool re-frames from the measured axis and mounting face. build123d's STEP writer stamps the
time into the header, so every run writes new bytes: the files are written ONCE (here) and committed as Git
LFS inputs - `--write drive` leaves the two kit files alone. The raw exports stay outside the repo next to
the monolith (lib.reference.MKS_EXPORT_NAME / MKS48_EXPORT_NAME); reference/README.md records their sha256.
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
        if all(abs(a - b) < 1e-6 for a, b in zip((n.X, n.Y, n.Z), normal)) and f.area >= min_area:
            out.append(f)
    return sorted(out, key=lambda f: f.area, reverse=True)


def unspun_shaft(shaft, m):
    """The export's D-shaft in the part frame: its D-flat un-spun onto +Z (-> +Y after TO_PART), the tip trimmed
    to m.shaft_length. Returns (solid, spin_deg, tip_before)."""
    flats = [f for f in shaft.faces().filter_by(GeomType.PLANE) if abs(f.normal_at().Y) < 0.5 and f.area > 5.0]
    if len(flats) != 1:
        raise SystemExit(f"expected one D-flat on the shaft, found {len(flats)}")
    n = flats[0].normal_at()
    spin = math.degrees(math.atan2(-n.X, n.Z))
    part = shaft.moved(Location((0.0, 0.0, 0.0), (-90.0, spin, 0.0)))
    tip = part.bounding_box().max.Z
    if tip > m.shaft_length + 1e-6:
        trim = Box(4 * m.shaft_dia, 4 * m.shaft_dia, tip - m.shaft_length + 1.0,
                   align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0.0, 0.0, m.shaft_length)))
        part = part - trim
    flat_n = [f.normal_at() for f in part.faces().filter_by(GeomType.PLANE) if abs(f.normal_at().Z) < 0.5 and f.area > 5.0][0]
    if max(abs(flat_n.X), abs(flat_n.Y - 1.0), abs(flat_n.Z)) > 1e-6:
        raise SystemExit(f"D-flat normal after re-framing {tuple(flat_n)} != (0, 1, 0)")
    return part, spin, tip


def split_kit(src: pathlib.Path, m):
    """The x40 export -> (motor Compound, board Compound, the re-framed shaft solid)."""
    print(f"importing {src.name} ({src.stat().st_size / 1e3:.0f} kB, sha256 {R.sha256(src)[:12]}) ...")
    solids = import_step(str(src)).solids()

    def across(s):
        return sorted(bbox(s)[1])[:2]   # the two smaller bbox extents: 5 x 5 for the shaft

    shafts = [s for s in solids if all(abs(v - m.shaft_dia) < 0.05 for v in across(s)) and s.volume < 1000]
    if len(shafts) != 1:
        raise SystemExit(f"expected one Ø{m.shaft_dia:g} shaft solid, found {len(shafts)}")
    shaft = shafts[0]
    body = max((s for s in solids if s is not shaft), key=lambda s: s.volume)
    kit = [s for s in solids if s is not shaft and s is not body]
    print(f"  {len(solids)} solids: body {body.volume:.1f} mm^3 {fmt(bbox(body)[1])}, shaft {shaft.volume:.1f} mm^3, kit {len(kit)} solids")

    # The export's frame: the mounting face is the body's largest -Y face and must sit at y = 0.
    face_y = planar_faces(body, (0, -1, 0))[0].center().Y
    rear_y = planar_faces(body, (0, 1, 0))[0].center().Y
    if abs(face_y) > 1e-6:
        raise SystemExit(f"the export's mounting face is at y = {face_y}, expected 0 - re-export with the origin on it")
    print(f"  mounting face y = {face_y:.6f}, rear face y = {rear_y:.3f} (body length {rear_y - face_y:.3f})")

    motor_shaft, spin, tip = unspun_shaft(shaft, m)
    print(f"  D-flat spun {spin:.6f} deg about the axis; shaft tip {tip:.3f} -> {motor_shaft.bounding_box().max.Z:g}")
    motor = Compound([body.moved(TO_PART), motor_shaft])
    motor.label = MOTOR_NAME
    board = Compound([s.moved(Location((0.0, 0.0, rear_y)) * TO_PART) for s in kit])
    board.label = BOARD_NAME

    # The part frames: motor mounting face z=0, shaft tip at +shaft_length, body below z=0 (rear at -rear_y);
    # board kit centred on the axis, its stack in -Z, its screws reaching into the motor (+Z).
    lo, size = bbox(motor)
    if not (len(motor.solids()) == 2 and abs(lo[2] + size[2] - m.shaft_length) < 1e-6 and lo[2] < -rear_y
            and abs(lo[0] + size[0] / 2.0) < 1e-6):
        raise SystemExit(f"motor geometry check failed: {len(motor.solids())} solids, bbox min {fmt(lo)} size {fmt(size)}")
    lo, size = bbox(board)
    if not (len(board.solids()) == len(kit) and lo[2] < 0 < lo[2] + size[2]
            and abs(lo[0] + size[0] / 2.0) < 1e-6 and abs(lo[1] + size[1] / 2.0) < 1e-6):
        raise SystemExit(f"board geometry check failed: {len(board.solids())} solids, bbox min {fmt(lo)} size {fmt(size)}")
    return motor, board, motor_shaft


def compose_drive_motor(src48: pathlib.Path, shaft, m):
    """The x48 export's motor body + `shaft` (the x40's, already in the part frame) -> the drive motor Compound."""
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
    mount_y = min(bbox(s)[0][1] for s in bodies) + m.pilot_height     # the pilot boss stands on the mounting face
    rear_y = max(bbox(s)[0][1] + bbox(s)[1][1] for s in bodies)
    if abs(rear_y - mount_y - m.body_length) > 0.05:
        raise SystemExit(f"body length {rear_y - mount_y:.3f} != {m.body_length:g}")
    print(f"  {len(solids)} solids; axis x={ax:.3f} z={az:.3f}, mounting face y={mount_y:.3f}, rear y={rear_y:.3f}")

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
    # the rotor: everything on the body side of the mounting face, with the x40's shaft fused on
    rotor_part = rotor.moved(to_part) - Box(3 * m.body_width, 3 * m.body_width, 2 * m.body_length,
                                            align=(Align.CENTER, Align.CENTER, Align.MIN))
    rotor_shaft = rotor_part + shaft
    if len(rotor_shaft.solids()) != 1:
        raise SystemExit(f"rotor + shaft fused into {len(rotor_shaft.solids())} solids, expected 1")
    drive = Compound(kept + [rotor_shaft.solids()[0]])
    drive.label = DRIVE_NAME
    print(f"  kept {len(kept)} body solids + the rotor with the x40 shaft = {len(drive.solids())} solids, "
          f"dropped {len(solids) - len(kept) - 1}")

    # Checks: the frame of parts/cycloidal/nema17_48mm.py (face z=0, shaft tip +22, body to -48) and the
    # reference's bounding box (tests/test_parts_convention.py::test_cots_vendor_matches_reference_frame, 1.5 mm).
    lo, size = bbox(drive)
    ref_lo, ref_size = bbox(R.load(DRIVE_NAME))
    if not (abs(lo[2] + size[2] - m.shaft_length) < 1e-6 and abs(lo[2] + m.body_length) < 1e-6
            and all(abs(a - b) <= 1.5 for a, b in zip(lo, ref_lo)) and all(abs(a - b) <= 1.5 for a, b in zip(size, ref_size))):
        raise SystemExit(f"drive motor bbox min {fmt(lo)} size {fmt(size)} vs reference {fmt(ref_lo)} {fmt(ref_size)}")
    if not planar_faces(drive, (0, 0, 1), min_area=800):
        raise SystemExit("no mounting face on z = 0")
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
    for src in (args.src,) + ((args.src48,) if args.write != "kit" else ()):
        if not src.expanduser().exists():
            print(f"kit export not found: {src}", file=sys.stderr)
            return 1

    motor, board, shaft = split_kit(args.src.expanduser(), m)
    outputs = [] if args.write == "drive" else [(motor, args.motor_out), (board, args.board_out)]
    if args.write != "kit":
        outputs.append((compose_drive_motor(args.src48.expanduser(), shaft, m), args.drive_out))
    for shape, out in outputs:
        lo, size = bbox(shape)
        print(f"  {shape.label}: {len(shape.solids())} solids, volume {R.solid_volume(shape):.3f} mm^3, bbox min {fmt(lo)} size {fmt(size)}")
        out.parent.mkdir(exist_ok=True)
        export_step(shape, str(out))
        print(f"wrote {out}: {len(shape.solids())} solids")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
