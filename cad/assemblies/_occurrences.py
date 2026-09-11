"""Shared helpers for assemblies/*.py: place part occurrences from reference/placements.json.

An OCCURRENCES table row is (part_or_module_name, role_or_None, placement_key), in the
SolidWorks document order. Roles make duplicate parts' labels unique (`j3_coupler:j2`);
they are positional/ordinal for now - rename them when the joint semantics are modelled.
Code-driven modules (assemblies/cycloidal_drive.py) use (part, role, Location) rows instead:
their placement key is a SolidWorks node, their contents are not (add_located / world_rows).

A child is a cadgen MODEL (parts.model(name), assemblies.<module>.<module>) reached through
lib.models.geometry(): inside a cadgen build that is the linked child (built in parallel, the
parent's STEP links its tree) - or, for the tinted GROUPS of the arm, an inline copy that still
pins and rebuilds the child; outside one (tests, tools, previews) it is the model body run
in-process, so nothing here ever starts a build or writes a file.
"""
from __future__ import annotations

import importlib

from build123d import Color, Location

import parts  # noqa: E402  (stdlib-only package index; parts.load() imports a part lazily)
from lib import placements as P
from lib.assembly import label_shape
from lib.models import geometry


def _part(part_name: str, *, inline: bool = False):
    """The part's geometry: linked child in a build (or an inline copy that still pins the child),
    a fresh in-process build otherwise."""
    return geometry(parts.model(part_name), inline=inline)


def place(part_name: str, key: str, *, inline: bool = False):
    """parts.model(part_name) placed at occurrence `key`.

    The part is modelled in its LOCAL frame (= LOCAL_FROM_REF * reference frame); the
    extracted placement maps the REFERENCE frame into the parent frame, so compose
    rel * LOCAL_FROM_REF^-1 and MOVE (compose) rather than locate (replace)."""
    mod = parts.load(part_name)
    local_from_ref = getattr(mod, "LOCAL_FROM_REF", None) or Location()
    return _part(part_name, inline=inline).moved(P.location(key, "rel") * local_from_ref.inverse())


def add_occurrences(asm, rows, modules: dict | None = None) -> None:
    """Add every row of an OCCURRENCES table to AssemblyHelper `asm`.

    `modules` maps a module name to its MODEL (a zero-arg cadgen model returning the module
    Compound in its own frame); it is moved to the module's placement key."""
    modules = modules or {}
    for name, role, key in rows:
        if name in modules:
            asm.add(geometry(modules[name]).moved(P.location(key, "rel")), name)
        elif role is None:
            asm.add(place(name, key), name)
        else:
            asm.add(place(name, key), name, role)


def _tint(shape, color: Color) -> None:
    """Set `color` on `shape` and every descendant. ocp_tessellate renders a leaf's own
    color (a compound-level color does not cascade), so tint the whole subtree - which is why the
    grouped occurrences are inline copies: a linked child keeps its own (untinted) leaves."""
    shape.color = color
    for child in getattr(shape, "children", ()) or ():
        _tint(child, color)


def add_grouped_occurrences(asm, rows, groups, modules: dict | None = None,
                            module_tints: dict | None = None) -> None:
    """add_occurrences(), but bucketed into labelled group Compounds (viewer/STEP tree nodes).

    `groups` rows are (group_label, tint, occurrence keys); together the keys must cover the
    OCCURRENCES rows' keys exactly once. Every shape in a group is tinted with the group's
    color, except a key in `module_tints` which keeps its own (the named modules stay visually
    distinct inside their group). Tints overwrite imported-STEP colors on the per-build
    copies only — standalone part/module previews are untouched. Because of the tints the
    members are INLINE copies (geometry(..., inline=True)): inside a cadgen build the child models
    are still called - pinned, rebuilt in parallel, their committed STEPs rewritten when stale -
    but the assembly's STEP carries its own recoloured geometry instead of links."""
    modules = modules or {}
    module_tints = module_tints or {}
    row_keys = sorted(key for _, _, key in rows)
    group_keys = sorted(key for _, _, keys in groups for key in keys)
    if group_keys != row_keys:
        raise ValueError(f"groups must cover the occurrence keys exactly once:\n{group_keys}\n{row_keys}")
    shapes = {}
    for name, role, key in rows:
        if name in modules:
            shape = label_shape(geometry(modules[name], inline=True).moved(P.location(key, "rel")), name)
        else:
            shape = label_shape(place(name, key, inline=True), name, *(() if role is None else (role,)))
        shapes[key] = shape
    for group_label, tint, keys in groups:
        members = []
        for key in keys:
            _tint(shapes[key], Color(module_tints.get(key, tint)))
            members.append(shapes[key])
        asm.add_module(group_label, members, color=Color(tint))


def place_at(part_name: str, loc: Location):
    """parts.model(part_name) at an explicit placement of its reference frame (the rows of a
    code-driven module such as assemblies/cycloidal_drive.py)."""
    mod = parts.load(part_name)
    local_from_ref = getattr(mod, "LOCAL_FROM_REF", None) or Location()
    return _part(part_name).moved(loc * local_from_ref.inverse())


def add_located(asm, rows) -> None:
    """Add every (part, role|None, Location) row of a code-driven module table to `asm`."""
    for name, role, loc in rows:
        if role is None:
            asm.add(place_at(name, loc), name)
        else:
            asm.add(place_at(name, loc), name, role)


def module_rows(module_name: str) -> list:
    """The (part, role, Location-in-module-frame) rows of a code-driven assemblies/<module>.py."""
    return list(importlib.import_module(f"assemblies.{module_name}").OCCURRENCES)


def world_rows(key: str) -> list:
    """(part, role, WORLD placement of the part's reference frame) for a placement key.

    A part key gives one row. A designed module key (placements.json kind "module",
    designed: true - the cycloidal drive) expands to its module's rows composed with the
    module's world pose, so links and inertials see every part inside it."""
    o = P.OCCURRENCES[key]
    if o["kind"] == "module":
        if not o.get("designed"):
            raise ValueError(f"{key}: only designed (code-driven) modules expand into world rows")
        world = P.location(key, "world")
        return [(part, role, world * loc) for part, role, loc in module_rows(o["part"])]
    return [(o["part"], None, P.location(key, "world"))]


def place_world_at(part_name: str, world: Location, into=None):
    """parts.model(part_name) at a WORLD placement of its reference frame, optionally
    re-expressed in another frame (`into` = that frame's world Location, so the result is
    `into^-1 * world * LOCAL_FROM_REF^-1 * local`). Used for per-link meshes."""
    mod = parts.load(part_name)
    local_from_ref = getattr(mod, "LOCAL_FROM_REF", None) or Location()
    loc = world * local_from_ref.inverse()
    if into is not None:
        loc = into.inverse() * loc
    return _part(part_name).moved(loc)


def place_world(part_name: str, key: str, into=None):
    """place_world_at() for a part occurrence key of placements.json."""
    return place_world_at(part_name, P.location(key, "world"), into)
