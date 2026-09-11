"""Export robot/meshes/<link>.stl for every physical link in robot/frames.py.

    ./cadtool python tools/robot/export_link_meshes.py [--links link1 link2 ...] [--tolerance 0.1] [--angular 0.3]

Meshes are written in MILLIMETRES in each link's own frame (robot/arm.urdf references them
with scale="0.001 0.001 0.001" and an identity origin). Binary STL via build123d; coarse
tolerances keep the committed files small (the GT2 90T pulleys are the heavy ones).
"""
from __future__ import annotations

import argparse
import pathlib

from build123d import export_stl
from lib import reference as R
from robot import frames as F
from robot._links import build_link

MESH_DIR = pathlib.Path(__file__).resolve().parents[2] / "robot" / "meshes"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--links", nargs="*", default=None)
    ap.add_argument("--tolerance", type=float, default=0.1, help="linear deflection, mm")
    ap.add_argument("--angular", type=float, default=0.3, help="angular deflection, rad")
    args = ap.parse_args(argv)
    MESH_DIR.mkdir(exist_ok=True)
    links = args.links or [l for l in F.LINK_ORDER if F.LINKS.get(l)]
    for link in links:
        shape = build_link(link)
        path = MESH_DIR / f"{link}.stl"
        ok = export_stl(shape, str(path), tolerance=args.tolerance, angular_tolerance=args.angular)
        size = path.stat().st_size if path.exists() else 0
        tris = max(0, (size - 84) // 50)
        print(f"{link:16s} solids={len(shape.solids()):2d} vol={R.solid_volume(shape):11.1f} mm3 "
              f"bbox={[round(v, 1) for v in R.bbox_size(shape)]} -> {path.name} {size/1e6:.2f} MB ~{tris} tris {'OK' if ok else 'FAILED'}")
        if not ok:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
