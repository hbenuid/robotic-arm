"""Materialise the motor mounts declared in lib/mounts.py as reference/placements.json part records.

    ./cadtool python tools/reference/mount_placements.py [--out reference/placements.json]

The SolidWorks capture never contained the belt joints' motors; lib/mounts.py declares each one (and its
MKS board) as a frame in its host occurrence's frame. This tool resolves `world = host world * frame`
(the host is a SolidWorks record, or the motor record for a board - declaration order), builds the part
in-process (parts.build) to fill `solids` / `solid_volume` / `world_bbox_*`, and writes ordinary
`kind: "part"` records with parent None, rel == world and a `mount` block naming the host, link, joint
and the frame - so assemblies/arm.py, robot/frames.py LINKS, the inertials and tools/bom.py read them
like any other occurrence. tools/reference/extract_placements.py calls mounted_records() on every
extraction; this tool's own main() is the MERGE mode: it replaces the mounted records inside the current
placements.json and leaves every SolidWorks record byte-for-byte alone - it needs no monolith, so a
changed mount (a spin, the wrist slide position) is regenerated on either machine.

Checks: every motor's +Z must be parallel to its joint's axis (robot/frames.py JOINTS) - a wrong
rotation convention in a mount fails here, not silently in the assembly.
"""
from __future__ import annotations

import argparse
import json
import pathlib

from build123d import Location

import parts
from lib import mounts
from lib import placements as P
from lib import reference as R
from lib.datum import to_location
from robot import frames as RF

AXIS_TOL = 1e-6


def _axis_z(loc: Location) -> tuple[float, float, float]:
    """World direction of the frame's +Z."""
    tip = (loc * Location((0.0, 0.0, 1.0))).position
    return tuple(a - b for a, b in zip(tip, loc.position))


def mounted_records(records: list[dict]) -> list[dict]:
    """The lib/mounts.py occurrences as placements.json part records, resolved against `records`
    (the SolidWorks occurrences; the boards resolve against the motors declared before them)."""
    worlds = {o["key"]: P.to_location(o["world"]) for o in records}
    joints = {j.name: j for j in RF.JOINTS}
    out = []
    for m in mounts.MOUNTS:
        if m.host not in worlds:
            raise SystemExit(f"{m.key}: host {m.host!r} has no placement record (declare the motor before its board)")
        world = worlds[m.host] * to_location(m.frame)
        if m.part == mounts.MOTOR:
            z, axis = _axis_z(world), joints[m.joint].axis_w
            if abs(abs(sum(a * b for a, b in zip(z, axis))) - 1.0) > AXIS_TOL:
                raise SystemExit(f"{m.key}: motor +Z {z} is not parallel to the {m.joint} axis {axis}")
        shape = parts.build(m.part).moved(world)
        bb = shape.bounding_box()
        out.append({
            "key": m.key, "part": m.part, "kind": "part", "path": None, "parent": None, "label_in_monolith": None,
            "mount": {
                "host": m.host, "link": m.link, "joint": m.joint,
                "frame_in_host": {"position": list(m.frame[0]), "rotation_xyz_deg": list(m.frame[1])},
                "source": "lib/mounts.py", "note": m.note,
            },
            "solids": len(shape.solids()), "solid_volume": round(R.solid_volume(shape), 3),
            "rel": P.to_record(world), "world": P.to_record(world),
            "world_bbox_min": [round(v, 3) for v in (bb.min.X, bb.min.Y, bb.min.Z)],
            "world_bbox_size": [round(v, 3) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
        })
        worlds[m.key] = world
    return out


def expected_totals(records: list[dict]) -> dict:
    """The `expected` block: totals over the part records (mounted ones included)."""
    parts_ = [o for o in records if o["kind"] == "part"]
    return {
        "leaf_occurrences": len(parts_),
        "solids": sum(o["solids"] for o in parts_),
        "solid_volume": round(sum(o["solid_volume"] for o in parts_), 3),
    }


def with_mounted(data: dict, records: list[dict], mounted: list[dict]) -> dict:
    """The placements document with `records` + `mounted` as its occurrences, `expected` recomputed and the
    `mounted` key list right after `designed_modules` (the same key order whichever tool writes it)."""
    out = {}
    for key, value in data.items():
        if key == "mounted":
            continue
        if key == "expected":
            value = expected_totals(records + mounted)
        elif key == "occurrences":
            value = records + mounted
        out[key] = value
        if key == "designed_modules":
            out["mounted"] = [o["key"] for o in mounted]
    return out


def write(path: pathlib.Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=pathlib.Path, default=P.PLACEMENTS_PATH)
    args = ap.parse_args(argv)
    data = json.loads(args.out.read_text(encoding="utf-8"))
    kept = [o for o in data["occurrences"] if "mount" not in o]
    mounted = mounted_records(kept)
    write(args.out, with_mounted(data, kept, mounted))
    ex = expected_totals(kept + mounted)
    print(f"wrote {args.out}: {len(kept)} SolidWorks records kept, {len(mounted)} mounted records; "
          f"parts={ex['leaf_occurrences']} solids={ex['solids']} volume={ex['solid_volume']}")
    for o in mounted:
        p = o["world"]["position"]
        print(f"  {o['key']:16s} on {o['mount']['host']:14s} {o['mount']['link']:15s} solids={o['solids']:2d} "
              f"world=({p[0]:8.2f},{p[1]:8.2f},{p[2]:8.2f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
