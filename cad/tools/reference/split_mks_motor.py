"""Split the "NEMA 17 x 40 + MKS SERVO42D" kit export (one SolidWorks assembly: motor body, D-shaft,
driver board + cover, 4 standoffs, 4 M3x30) into the two vendor files the parts import:

    vendor/nema17_40mm.step   the motor body + shaft (parts/joints/nema17_40mm.py) - re-framed like the
                              drive motor (mounting face z=0, body -Z, pilot + shaft +Z, D-flat +Y) and the
                              shaft trimmed to MotorParams.shaft_length (22; the export's is 23)
    vendor/mks_servo42d.step  the board kit (parts/joints/mks_servo42d.py) - same orientation, z=0 at the
                              motor's REAR face, the board stack in -Z

    ./cadtool python tools/reference/split_mks_motor.py [--src ~/Documents/arm_assembly_organized/mks/nema17x40_with_mks.step]
                                                        [--motor-out vendor/nema17_40mm.step] [--board-out vendor/mks_servo42d.step]
    ./cadtool python tools/reference/import_solidworks.py       # then: mirrors vendor -> reference/solidworks, manifest entries

The solids are told apart by GEOMETRY, not by the export's product labels (NAUO ids): the shaft is the
Ø5 solid, the body the largest one, the kit everything else. The export's shaft carries its D-flat spun
~3.44 deg about the axis (the rotor was modelled at an angle) - only the shaft is un-spun, the body's square
stays axis-aligned. Written ONCE (here) and committed as Git LFS inputs: STEP bytes differ per machine, so
never regenerate them elsewhere. The raw export stays outside the repo next to the monolith
(lib.reference.MKS_EXPORT_NAME); reference/README.md records its sha256.
"""
from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys

from build123d import Align, Axis, Box, Compound, GeomType, Location, export_step, import_step
from lib import reference as R
from lib.cycloidal import DEFAULT_CONFIG

MOTOR_NAME, BOARD_NAME = "nema17_40mm", "mks_servo42d"
TO_PART = Location((0.0, 0.0, 0.0), (-90.0, 0.0, 0.0))   # export frame (face y=0, body +Y, shaft -Y) -> part frame (body -Z, shaft +Z)


def bbox(shape):
    bb = shape.bounding_box()
    return (bb.min.X, bb.min.Y, bb.min.Z), (bb.size.X, bb.size.Y, bb.size.Z)


def fmt(v):
    return "(" + ", ".join(f"{x:.3f}" for x in v) + ")"


def classify(solids):
    """(body, shaft, kit solids): the shaft is the Ø5 solid (5 x 5 across the axis, small), the body the
    largest solid, the kit everything else."""
    def across(s):
        return sorted(bbox(s)[1])[:2]   # the two smaller bbox extents: 5 x 5 for the shaft

    shafts = [s for s in solids if all(abs(v - 5.0) < 0.05 for v in across(s)) and s.volume < 1000]
    if len(shafts) != 1:
        raise SystemExit(f"expected one Ø5 shaft solid, found {len(shafts)}")
    shaft = shafts[0]
    body = max((s for s in solids if s is not shaft), key=lambda s: s.volume)
    kit = [s for s in solids if s is not shaft and s is not body]
    return body, shaft, kit


def planar_faces(shape, normal, min_area=1.0):
    """Planar faces of `shape` whose normal is `normal` (within 1e-6), largest first."""
    out = []
    for f in shape.faces().filter_by(GeomType.PLANE):
        n = f.normal_at()
        if all(abs(a - b) < 1e-6 for a, b in zip((n.X, n.Y, n.Z), normal)) and f.area >= min_area:
            out.append(f)
    return sorted(out, key=lambda f: f.area, reverse=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("ARM_REFERENCE_SRC", R.DEFAULT_SOURCE_DIR)) / R.MKS_EXPORT_NAME)
    ap.add_argument("--motor-out", type=pathlib.Path, default=R.VENDOR_DIR / f"{MOTOR_NAME}.step")
    ap.add_argument("--board-out", type=pathlib.Path, default=R.VENDOR_DIR / f"{BOARD_NAME}.step")
    args = ap.parse_args(argv)
    src = args.src.expanduser()
    if not src.exists():
        print(f"kit export not found: {src}", file=sys.stderr)
        return 1
    m = DEFAULT_CONFIG.motor

    print(f"importing {src.name} ({src.stat().st_size / 1e3:.0f} kB, sha256 {R.sha256(src)[:12]}) ...")
    root = import_step(str(src))
    solids = root.solids()
    body, shaft, kit = classify(solids)
    print(f"  {len(solids)} solids: body {body.volume:.1f} mm^3 {fmt(bbox(body)[1])}, shaft {shaft.volume:.1f} mm^3, kit {len(kit)} solids")

    # The export's frame: the mounting face is the body's largest -Y face and must sit at y = 0.
    face_y = planar_faces(body, (0, -1, 0))[0].center().Y
    rear_y = planar_faces(body, (0, 1, 0))[0].center().Y
    if abs(face_y) > 1e-6:
        raise SystemExit(f"the export's mounting face is at y = {face_y}, expected 0 - re-export with the origin on it")
    print(f"  mounting face y = {face_y:.6f}, rear face y = {rear_y:.3f} (body length {rear_y - face_y:.3f})")

    # The shaft's D-flat: a planar face parallel to the axis (Y); un-spin it onto +Z (-> +Y after TO_PART).
    flats = [f for f in shaft.faces().filter_by(GeomType.PLANE) if abs(f.normal_at().Y) < 0.5 and f.area > 5.0]
    if len(flats) != 1:
        raise SystemExit(f"expected one D-flat on the shaft, found {len(flats)}")
    n = flats[0].normal_at()
    spin = math.degrees(math.atan2(-n.X, n.Z))
    print(f"  D-flat normal ({n.X:.6f}, {n.Y:.6f}, {n.Z:.6f}) -> spin {spin:.6f} deg about the axis")

    motor_body = body.moved(TO_PART)
    motor_shaft = shaft.moved(Location((0.0, 0.0, 0.0), (-90.0, spin, 0.0)))
    tip = motor_shaft.bounding_box().max.Z
    if tip > m.shaft_length + 1e-6:
        trim = Box(4 * m.shaft_dia, 4 * m.shaft_dia, tip - m.shaft_length + 1.0,
                   align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0.0, 0.0, m.shaft_length)))
        motor_shaft = motor_shaft - trim
        print(f"  shaft tip {tip:.3f} -> trimmed to {m.shaft_length:g}")
    flat_n = [f.normal_at() for f in motor_shaft.faces().filter_by(GeomType.PLANE) if abs(f.normal_at().Z) < 0.5 and f.area > 5.0][0]
    if max(abs(flat_n.X), abs(flat_n.Y - 1.0), abs(flat_n.Z)) > 1e-6:
        raise SystemExit(f"D-flat normal after re-framing {tuple(flat_n)} != (0, 1, 0)")
    motor = Compound([motor_body, motor_shaft])
    motor.label = MOTOR_NAME

    board = Compound([s.moved(Location((0.0, 0.0, rear_y)) * TO_PART) for s in kit])
    board.label = BOARD_NAME

    for name, shape, out in ((MOTOR_NAME, motor, args.motor_out), (BOARD_NAME, board, args.board_out)):
        lo, size = bbox(shape)
        print(f"  {name}: {len(shape.solids())} solids, volume {R.solid_volume(shape):.3f} mm^3, bbox min {fmt(lo)} size {fmt(size)}")
    # The part frames: motor mounting face z=0, shaft tip at +shaft_length, body below z=0 (rear at -rear_y);
    # board kit centred on the axis, its stack in -Z, its screws reaching into the motor (+Z).
    lo, size = bbox(motor)
    hi_z = lo[2] + size[2]
    if not (len(motor.solids()) == 2 and abs(hi_z - m.shaft_length) < 1e-6 and lo[2] < -rear_y
            and abs(lo[0] + size[0] / 2.0) < 1e-6):
        raise SystemExit(f"motor geometry check failed: {len(motor.solids())} solids, bbox min {fmt(lo)} size {fmt(size)}")
    lo, size = bbox(board)
    if not (len(board.solids()) == len(kit) and lo[2] < 0 < lo[2] + size[2]
            and abs(lo[0] + size[0] / 2.0) < 1e-6 and abs(lo[1] + size[1] / 2.0) < 1e-6):
        raise SystemExit(f"board geometry check failed: {len(board.solids())} solids, bbox min {fmt(lo)} size {fmt(size)}")

    for shape, out in ((motor, args.motor_out), (board, args.board_out)):
        out.parent.mkdir(exist_ok=True)
        export_step(shape, str(out))
        print(f"wrote {out}: {len(shape.solids())} solids")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
