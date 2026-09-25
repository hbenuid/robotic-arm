"""Shared helpers for assemblies/*.py: place part occurrences from reference/placements.json.

An OCCURRENCES table row is (part_or_module_name, role_or_None, placement_key), in the
SolidWorks document order. Roles make duplicate parts' labels unique (`j3_coupler:j2`);
they are positional/ordinal for now - rename them when the joint semantics are modelled.
Code-driven modules (assemblies/cycloidal_drive.py, forearm_roll_drive.py) use (part, role, placement)
rows instead - the placement in the MODULE frame, as data: a position `(x, y, z)` for a part whose own
frame is already aligned with the module's, or a frame `((x, y, z), (rx, ry, rz))` (lib.datum) when it
needs a rotation (row_location). Their placement key is a SolidWorks node or a lib/mounts.py
ModuleMount, their contents are not (located_children / world_rows), and their BODIES split the rows
into rigid bodies ("cycloidal_drive#1:rotor", split_key) for the robot description.

A child is a cadgen MODEL (parts.model(name), assemblies.<module>.<module>) reached through
lib.models.geometry(): inside a cadgen build that is the linked child (built in parallel, the
parent's STEP links its tree) - or, for the tinted GROUPS of the arm, an inline copy that still
pins and rebuilds the child; outside one (tests, tools, previews) it is the model body run
in-process, so nothing here ever starts a build or writes a file.
"""
from __future__ import annotations

import importlib

from cadgen import build123d as bd

import parts  # noqa: E402  (stdlib-only package index; parts.load() imports a part lazily)
from lib import placements as P
from lib.assembly import assembly, label_shape
from lib.datum import IDENTITY, to_location
from lib.models import geometry


def _local_from_ref(part_name: str) -> bd.Location:
    """The part's LOCAL_FROM_REF (frame data, lib.datum) as a Location; identity when it declares none."""
    return to_location(getattr(parts.load(part_name), "LOCAL_FROM_REF", IDENTITY))


def _part(part_name: str, *, inline: bool = False):
    """The part's geometry: linked child in a build (or an inline copy that still pins the child),
    a fresh in-process build otherwise."""
    return geometry(parts.model(part_name), inline=inline)


def place(part_name: str, key: str, *, inline: bool = False, root: bd.Location | None = None):
    """parts.model(part_name) placed at occurrence `key`.

    The part is modelled in its LOCAL frame (= LOCAL_FROM_REF * reference frame); the
    extracted placement maps the REFERENCE frame into the parent frame, so compose
    rel * LOCAL_FROM_REF^-1 and MOVE (compose) rather than locate (replace). `root` (parent
    frame -> output frame) re-expresses the occurrence in another frame: root * rel * ..."""
    return _part(part_name, inline=inline).moved(
        (root or bd.Location()) * P.location(key, "rel") * _local_from_ref(part_name).inverse())


def _details(role) -> tuple:
    """The label details of a row: its role, if it has one (`j3_coupler:j2`)."""
    return () if role is None else (role,)


def occurrence_children(rows, modules: dict | None = None, tint: str | None = None) -> list:
    """Every row of an OCCURRENCES table as a placed, labelled child (for lib.assembly.assembly()).

    `modules` maps a module name to its MODEL (a zero-arg cadgen model returning the module
    Compound in its own frame); it is moved to the module's placement key. With `tint` the module
    is colored like the arm: printed parts `tint`, purchased parts BOUGHT_TINT (_tint_parts)."""
    modules = modules or {}
    children = []
    for name, role, key in rows:
        if name in modules:
            children.append(label_shape(geometry(modules[name]).moved(P.location(key, "rel")), name))
        else:
            children.append(label_shape(place(name, key), name, *_details(role)))
    if tint is not None:
        for child in children:
            _tint_parts(child, bd.Color(tint))
    return children


def _tint(shape, color: bd.Color) -> None:
    """Set `color` on `shape` and every descendant. A leaf is rendered in its own color (a
    compound-level color did not cascade in the OCP CAD Viewer), so tint the whole subtree - which is why the
    grouped occurrences are inline copies: a linked child keeps its own (untinted) leaves."""
    shape.color = color
    for child in getattr(shape, "children", ()) or ():
        _tint(child, color)


# The one color of a PURCHASED part (parts.bought(): COTS = True) wherever an assembly tints its
# leaves - the arm's link GROUPS, the gripper and the drive. No group / module tint may reuse it.
BOUGHT_TINT = "#9AA0A6"


def _tint_parts(shape, color: bd.Color) -> None:
    """_tint(), except that every purchased part inside `shape` - a node labelled '<part>[:role]'
    with parts.bought(part), at any depth (the modules' parts too) - gets BOUGHT_TINT instead: in a
    tinted assembly, grey always means bought and a link / module color means printed."""
    name = (shape.label or "").split(":")[0]
    if name in parts.MODULES:
        _tint(shape, bd.Color(BOUGHT_TINT) if parts.bought(name) else color)
        return
    shape.color = color
    for child in getattr(shape, "children", ()) or ():
        _tint_parts(child, color)


def grouped_children(rows, groups, modules: dict | None = None,
                     module_tints: dict | None = None, root: bd.Location | None = None) -> list:
    """occurrence_children(), but bucketed into labelled group Compounds (viewer/STEP tree nodes).

    `groups` rows are (group_label, tint, occurrence keys); together the keys must cover the
    OCCURRENCES rows' keys exactly once. Every PRINTED part in a group is tinted with the group's
    color, except under a key in `module_tints` which keeps its own (the named modules stay visually
    distinct inside their group); every PURCHASED part, inside the modules too, gets BOUGHT_TINT
    (_tint_parts) - so the assembly shows at a glance what was bought. Tints overwrite
    imported-STEP colors on the per-build copies only — standalone part/module previews are
    untouched. Because of the tints the members are INLINE copies (geometry(..., inline=True)):
    inside a cadgen build the child models are still called - pinned, rebuilt in parallel, their
    committed STEPs rewritten when stale - but the assembly's STEP carries its own recoloured
    geometry instead of links.

    `root` (placements' parent frame -> output frame) re-expresses the whole assembly in another
    frame by composing into every occurrence's placement (root * rel). It has to go there: cadgen's
    STEP packager reads the children's locations only, so a .moved() on the built root Compound
    would move the in-process shape but not the written STEP."""
    modules = modules or {}
    module_tints = module_tints or {}
    root = root or bd.Location()
    row_keys = sorted(key for _, _, key in rows)
    group_keys = sorted(key for _, _, keys in groups for key in keys)
    if group_keys != row_keys:
        raise ValueError(f"groups must cover the occurrence keys exactly once:\n{group_keys}\n{row_keys}")
    shapes = {}
    for name, role, key in rows:
        if name in modules:
            shape = label_shape(geometry(modules[name], inline=True).moved(root * P.location(key, "rel")), name)
        else:
            shape = label_shape(place(name, key, inline=True, root=root), name, *_details(role))
        shapes[key] = shape
    nodes = []
    for group_label, tint, keys in groups:
        members = []
        for key in keys:
            _tint_parts(shapes[key], bd.Color(module_tints.get(key, tint)))
            members.append(shapes[key])
        nodes.append(assembly(group_label, members, color=bd.Color(tint)))
    return nodes


def place_at(part_name: str, loc: bd.Location):
    """parts.model(part_name) at an explicit placement of its reference frame (the rows of a
    code-driven module such as assemblies/cycloidal_drive.py)."""
    return _part(part_name).moved(loc * _local_from_ref(part_name).inverse())


def row_location(placement) -> bd.Location:
    """The Location of a module row's placement: a position `(x, y, z)` (no rotation - the part's
    frame is the module's, shifted) or a frame `((x, y, z), (rx, ry, rz))` (lib.datum.to_location)."""
    if len(placement) == 2 and not isinstance(placement[0], (int, float)):
        return to_location(placement)
    return bd.Location(tuple(placement))


def located_children(rows, tint: str | None = None) -> list:
    """Every (part, role|None, placement) row of a code-driven module table as a placed, labelled child
    (row_location). With `tint` the module is colored like the arm: printed parts `tint`, purchased
    parts BOUGHT_TINT."""
    children = [label_shape(place_at(name, row_location(pos)), name, *_details(role)) for name, role, pos in rows]
    if tint is not None:
        for child in children:
            _tint_parts(child, bd.Color(tint))
    return children


def module_rows(module_name: str) -> list:
    """The (part, role, placement-in-module-frame) rows of a code-driven assemblies/<module>.py."""
    return list(importlib.import_module(f"assemblies.{module_name}").OCCURRENCES)


def module_bodies(module_name: str) -> dict:
    """The {body: frozenset(part names)} partition of a code-driven module's rows into rigid
    bodies (assemblies/<module>.py BODIES)."""
    return dict(importlib.import_module(f"assemblies.{module_name}").BODIES)


def split_key(key: str) -> tuple[str, str | None]:
    """"<occurrence key>[:<body>]" -> (occurrence key, body). A body suffix names one rigid body
    of a designed module (module_bodies); robot/frames.py LINKS uses it to put the cycloidal
    drive's stator and rotor in different links."""
    occ, _, body = key.partition(":")
    return occ, (body or None)


def world_rows(key: str) -> list:
    """(part, role, WORLD placement of the part's reference frame) for a placement key.

    A part key gives one row. A designed module key (placements.json kind "module",
    designed: true - the cycloidal drive, the forearm roll drive) expands to its module's rows
    composed with the module's world pose, so links and inertials see every part inside it; with a
    ":<body>" suffix (split_key) only the rows of that rigid body."""
    occ, body = split_key(key)
    o = P.OCCURRENCES[occ]
    if o["kind"] != "module":
        if body is not None:
            raise ValueError(f"{key}: only a designed module has bodies")
        return [(o["part"], None, P.location(occ, "world"))]
    if not o.get("designed"):
        raise ValueError(f"{key}: only designed (code-driven) modules expand into world rows")
    rows = module_rows(o["part"])
    if body is not None:
        bodies = module_bodies(o["part"])
        if body not in bodies:
            raise ValueError(f"{key}: unknown body {body!r} (have {sorted(bodies)})")
        rows = [r for r in rows if r[0] in bodies[body]]
    world = P.location(occ, "world")
    return [(part, role, world * row_location(pos)) for part, role, pos in rows]


def place_world_at(part_name: str, world: bd.Location, into=None):
    """parts.model(part_name) at a WORLD placement of its reference frame, optionally
    re-expressed in another frame (`into` = that frame's world Location, so the result is
    `into^-1 * world * LOCAL_FROM_REF^-1 * local`). Used for per-link meshes."""
    loc = world * _local_from_ref(part_name).inverse()
    if into is not None:
        loc = into.inverse() * loc
    return _part(part_name).moved(loc)


def place_world(part_name: str, key: str, into=None):
    """place_world_at() for a part occurrence key of placements.json."""
    return place_world_at(part_name, P.location(key, "world"), into)
