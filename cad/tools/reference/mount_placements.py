"""Materialise the mounts declared in lib/mounts.py - the belt joints' motors and the pose of a code-driven
module - as reference/placements.json records.

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
changed mount (a spin, the wrist slide position) is regenerated on either machine. One exception: a part record
whose SolidWorks product has since been put in lib/reference.py SKIPPED_PRODUCTS (a part the design dropped - the
link caps, 2026-09-25) moves to `skipped` as the entry extract_placements.py would write for it, and every `skipped`
entry's reason is re-read from there, so the document equals what a fresh extraction gives.

A ModuleMount (lib/mounts.py MODULE_MOUNTS - the forearm roll drive, which no SolidWorks node places) becomes
a `kind: "module", designed: true` record with the same `mount` block and no solids / volume (the module's
own totals are its EXPECTED; expected_totals() counts part records only), listed under `designed_modules`
AND `mounted`, exactly like the drive's SolidWorks-placed record otherwise.

Checks: every motor's +Z must be parallel to its joint's axis (robot/frames.py JOINTS), a module's +Z must
lie ON that axis (parallel, origin on the line) - a wrong rotation convention in a mount fails here, not
silently in the assembly.
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

AXIS_TOL = 1e-6        # direction cosine
ORIGIN_TOL = 0.01      # mm: a module's origin off its joint's axis (the six-decimal placements round to ~1 um)


def _axis_z(loc: Location) -> tuple[float, float, float]:
    """World direction of the frame's +Z."""
    tip = (loc * Location((0.0, 0.0, 1.0))).position
    return tuple(a - b for a, b in zip(tip, loc.position, strict=True))


def _mount_block(m) -> dict:
    """The `mount` block of a record (key order = the file's: a part mount has a link, a module mount is
    split over links by its bodies)."""
    return {
        "host": m.host, **({"link": m.link} if hasattr(m, "link") else {}), "joint": m.joint,
        "frame_in_host": {"position": list(m.frame[0]), "rotation_xyz_deg": list(m.frame[1])},
        "source": "lib/mounts.py", "note": m.note,
    }


def _check_axis(key: str, world: Location, joint, *, on_axis: bool) -> None:
    z, axis = _axis_z(world), joint.axis_w
    if abs(abs(sum(a * b for a, b in zip(z, axis, strict=True))) - 1.0) > AXIS_TOL:
        raise SystemExit(f"{key}: +Z {z} is not parallel to the {joint.name} axis {axis}")
    if on_axis:
        d = tuple(a - b for a, b in zip(world.position, joint.origin_w, strict=True))
        off = tuple(a - sum(x * y for x, y in zip(d, axis, strict=True)) * b for a, b in zip(d, axis, strict=True))
        if sum(v * v for v in off) ** 0.5 > ORIGIN_TOL:
            raise SystemExit(f"{key}: origin {tuple(world.position)} is {sum(v * v for v in off) ** 0.5:.4f} mm off the {joint.name} axis")


def mounted_records(records: list[dict]) -> list[dict]:
    """The lib/mounts.py occurrences as placements.json records, resolved against `records` (the SolidWorks
    occurrences; the boards resolve against the motors declared before them): the part mounts first, then the
    module mounts."""
    worlds = {o["key"]: P.to_location(o["world"]) for o in records}
    joints = RF.JOINT_BY_NAME
    out = []
    for m in mounts.MOUNTS:
        if m.host not in worlds:
            raise SystemExit(f"{m.key}: host {m.host!r} has no placement record (declare the motor before its board)")
        world = worlds[m.host] * to_location(m.frame)
        if m.part in mounts.MOTORS:
            _check_axis(m.key, world, joints[m.joint], on_axis=False)
        shape = parts.build(m.part).moved(world)
        bb = shape.bounding_box()
        out.append({
            "key": m.key, "part": m.part, "kind": "part", "path": None, "parent": None, "label_in_monolith": None,
            "mount": _mount_block(m),
            "solids": len(shape.solids()), "solid_volume": round(R.solid_volume(shape), 3),
            "rel": P.to_record(world), "world": P.to_record(world),
            "world_bbox_min": [round(v, 3) for v in (bb.min.X, bb.min.Y, bb.min.Z)],
            "world_bbox_size": [round(v, 3) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
        })
        worlds[m.key] = world
    for m in mounts.MODULE_MOUNTS:
        if m.host not in worlds:
            raise SystemExit(f"{m.key}: host {m.host!r} has no placement record")
        if m.module not in R.DESIGNED_MODULES:
            raise SystemExit(f"{m.key}: {m.module!r} is not in lib/reference.py DESIGNED_MODULES")
        world = worlds[m.host] * to_location(m.frame)
        _check_axis(m.key, world, joints[m.joint], on_axis=True)
        out.append({
            "key": m.key, "part": m.module, "kind": "module", "designed": True, "path": None, "parent": None,
            "label_in_monolith": None, "mount": _mount_block(m),
            "rel": P.to_record(world), "world": P.to_record(world),
            "source": f"assemblies/{m.module}.py",
        })
        worlds[m.key] = world
    return out


def skipped_entry(o: dict) -> dict:
    """A part record whose product is in lib/reference.py SKIPPED_PRODUCTS, as the `skipped` entry
    tools/reference/extract_placements.py writes for that node (a part record is one leaf)."""
    if o["kind"] != "part":
        raise SystemExit(f"{o['key']}: only a part record can move to `skipped` here - re-run extract_placements.py")
    return {
        "path": o["path"], "label": o["label_in_monolith"], "reason": R.SKIPPED_LABELS[o["label_in_monolith"]],
        "leaves": 1, "solids": o["solids"], "solid_volume": o["solid_volume"], "rel": o["rel"], "world": o["world"],
        "world_bbox_min": o["world_bbox_min"], "world_bbox_size": o["world_bbox_size"],
    }


def expected_totals(records: list[dict]) -> dict:
    """The `expected` block: totals over the part records (mounted ones included)."""
    parts_ = [o for o in records if o["kind"] == "part"]
    return {
        "leaf_occurrences": len(parts_),
        "solids": sum(o["solids"] for o in parts_),
        "solid_volume": round(sum(o["solid_volume"] for o in parts_), 3),
    }


def with_mounted(data: dict, records: list[dict], mounted: list[dict]) -> dict:
    """The placements document with `records` + `mounted` as its occurrences, `expected` and
    `designed_modules` recomputed (a mounted module is a designed module too) and the `mounted` key list
    right after `designed_modules` (the same key order whichever tool writes it)."""
    out = {}
    for key, value in data.items():
        if key == "mounted":
            continue
        if key == "expected":
            value = expected_totals(records + mounted)
        elif key == "designed_modules":
            value = [o["key"] for o in records + mounted if o["kind"] == "module" and o.get("designed")]
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
    solidworks = [o for o in data["occurrences"] if "mount" not in o]
    kept = [o for o in solidworks if o["label_in_monolith"] not in R.SKIPPED_LABELS]
    dropped = [skipped_entry(o) for o in solidworks if o["label_in_monolith"] in R.SKIPPED_LABELS]
    known = [{**s, "reason": R.SKIPPED_LABELS.get(s["label"], s["reason"])} for s in data["skipped"]]   # reasons as lib/reference.py words them
    data["skipped"] = sorted(known + dropped, key=lambda s: tuple(int(i) for i in s["path"].split(".")))
    mounted = mounted_records(kept)
    write(args.out, with_mounted(data, kept, mounted))
    ex = expected_totals(kept + mounted)
    for s in dropped:
        print(f"  {s['label']} (path {s['path']}) -> skipped: {s['reason']}")
    print(f"wrote {args.out}: {len(kept)} SolidWorks records kept, {len(mounted)} mounted records; "
          f"parts={ex['leaf_occurrences']} solids={ex['solids']} volume={ex['solid_volume']}")
    for o in mounted:
        p = o["world"]["position"]
        what = f"solids={o['solids']:2d}" if o["kind"] == "part" else f"module {o['mount']['joint']}"
        print(f"  {o['key']:20s} on {o['mount']['host']:14s} {o['mount'].get('link', '-'):15s} {what} "
              f"world=({p[0]:8.2f},{p[1]:8.2f},{p[2]:8.2f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
