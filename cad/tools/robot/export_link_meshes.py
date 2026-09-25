"""Export robot/meshes/<link>.stl for every physical link in robot/frames.py.

    ./cadtool python tools/robot/export_link_meshes.py [--links shoulder_link upper_arm_link ...] [--tolerance 0.1] [--angular 0.3]

Meshes are written in MILLIMETRES in each link's own frame (robot/arm.urdf references them
with scale="0.001 0.001 0.001" and an identity origin). Binary STL via build123d; coarse
tolerances keep the committed files small (the GT2 90T pulleys are the heavy ones).
tests/test_robot.py re-exports every link with export_link() and fails when a committed mesh is stale.
"""
from __future__ import annotations

import argparse
import pathlib

from build123d import export_stl

from lib import reference as R
from robot import frames as F
from robot._links import build_link

MESH_DIR = pathlib.Path(__file__).resolve().parents[2] / "robot" / "meshes"
TOLERANCE = 0.1   # linear deflection, mm
ANGULAR = 0.3     # angular deflection, rad


def export_link(link: str, path: pathlib.Path, tolerance: float = TOLERANCE, angular: float = ANGULAR):
    """build_link(link) -> binary STL at `path`; returns (shape, ok)."""
    shape = build_link(link)
    return shape, export_stl(shape, str(path), tolerance=tolerance, angular_tolerance=angular)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--links", nargs="*", default=None)
    ap.add_argument("--tolerance", type=float, default=TOLERANCE, help="linear deflection, mm")
    ap.add_argument("--angular", type=float, default=ANGULAR, help="angular deflection, rad")
    args = ap.parse_args(argv)
    MESH_DIR.mkdir(exist_ok=True)
    links = args.links or [l for l in F.LINK_ORDER if F.LINKS.get(l)]
    for link in links:
        path = MESH_DIR / f"{link}.stl"
        shape, ok = export_link(link, path, args.tolerance, args.angular)
        size = path.stat().st_size if path.exists() else 0
        tris = max(0, (size - 84) // 50)
        print(f"{link:16s} solids={len(shape.solids()):2d} vol={R.solid_volume(shape):11.1f} mm3 "
              f"bbox={[round(v, 1) for v in R.bbox_size(shape)]} -> {path.name} {size/1e6:.2f} MB ~{tris} tris {'OK' if ok else 'FAILED'}")
        if not ok:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
