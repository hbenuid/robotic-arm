"""Shared helpers for assemblies/*.py: place part occurrences from reference/placements.json.

An OCCURRENCES table row is (part_or_module_name, role_or_None, placement_key), in the
SolidWorks document order. Roles make duplicate parts' labels unique (`j3_coupler:j2`);
they are positional/ordinal for now - rename them when the joint semantics are modelled.
Code-driven modules (assemblies/cycloidal_drive.py) use (part, role, Location) rows instead:
their placement key is a SolidWorks node, their contents are not (add_located / world_rows).
"""
from __future__ import annotations

import importlib
import pathlib
import sys

from build123d import Color, Location

import parts  # noqa: E402  (stdlib-only package index; parts.load() imports a part lazily)
from lib.assembly import label_shape
from lib import placements as P

CAD_DIR = str(pathlib.Path(__file__).resolve().parent.parent)


def place(part_name: str, key: str):
    """A fresh copy of parts.load(part_name).gen_step() placed at occurrence `key`.

    The part is modelled in its LOCAL frame (= LOCAL_FROM_REF * reference frame); the
    extracted placement maps the REFERENCE frame into the parent frame, so compose
    rel * LOCAL_FROM_REF^-1 and MOVE (compose) rather than locate (replace)."""
    # The plugin's generator runner (scripts/gen) restores sys.path after importing the generator
    # module and does not seed the cwd, so this lazy import must re-assert the cad/ root itself.
    if CAD_DIR not in sys.path:
        sys.path.insert(0, CAD_DIR)
    mod = parts.load(part_name)
    local_from_ref = getattr(mod, "LOCAL_FROM_REF", None) or Location()
    return mod.gen_step().moved(P.location(key, "rel") * local_from_ref.inverse())


def add_occurrences(asm, rows, modules: dict | None = None) -> None:
    """Add every row of an OCCURRENCES table to AssemblyHelper `asm`.

    `modules` maps a module name to a zero-arg builder returning the module Compound in
    its own frame; it is located in place at the module's placement key."""
    modules = modules or {}
    for name, role, key in rows:
        if name in modules:
            module = modules[name]()
            module.locate(P.location(key, "rel"))   # in place: .located() would deep-copy the tree
            asm.add(module, name)
        elif role is None:
            asm.add(place(name, key), name)
        else:
            asm.add(place(name, key), name, role)


def _tint(shape, color: Color) -> None:
    """Set `color` on `shape` and every descendant. ocp_tessellate renders a leaf's own
    color (a compound-level color does not cascade), so tint the whole subtree."""
    shape.color = color
    for child in getattr(shape, "children", ()) or ():
        _tint(child, color)


def add_grouped_occurrences(asm, rows, groups, modules: dict | None = None,
                            module_tints: dict | None = None) -> None:
    """add_occurrences(), but bucketed into labelled group Compounds (viewer/STEP tree nodes).

    `groups` rows are (group_label, tint, occurrence keys); together the keys must cover the
    OCCURRENCES rows' keys exactly once. Every shape in a group is tinted with the group's
    color, except a key in `module_tints` which keeps its own (the named modules stay visually
    distinct inside their group). Tints overwrite imported-STEP colors on the fresh per-build
    copies only — standalone part/module previews are untouched."""
    modules = modules or {}
    module_tints = module_tints or {}
    row_keys = sorted(key for _, _, key in rows)
    group_keys = sorted(key for _, _, keys in groups for key in keys)
    if group_keys != row_keys:
        raise ValueError(f"groups must cover the occurrence keys exactly once:\n{group_keys}\n{row_keys}")
    shapes = {}
    for name, role, key in rows:
        if name in modules:
            shape = modules[name]()
            shape.locate(P.location(key, "rel"))   # in place: .located() would deep-copy the tree
            label_shape(shape, name)
        else:
            shape = label_shape(place(name, key), name, *(() if role is None else (role,)))
        shapes[key] = shape
    for group_label, tint, keys in groups:
        members = []
        for key in keys:
            _tint(shapes[key], Color(module_tints.get(key, tint)))
            members.append(shapes[key])
        asm.add_module(group_label, members, color=Color(tint))


def place_at(part_name: str, loc: Location):
    """A fresh copy of parts.load(part_name).gen_step() at an explicit placement of its reference
    frame (the rows of a code-driven module such as assemblies/cycloidal_drive.py)."""
    if CAD_DIR not in sys.path:
        sys.path.insert(0, CAD_DIR)
    mod = parts.load(part_name)
    local_from_ref = getattr(mod, "LOCAL_FROM_REF", None) or Location()
    return mod.gen_step().moved(loc * local_from_ref.inverse())


def add_located(asm, rows) -> None:
    """Add every (part, role|None, Location) row of a code-driven module table to `asm`."""
    for name, role, loc in rows:
        if role is None:
            asm.add(place_at(name, loc), name)
        else:
            asm.add(place_at(name, loc), name, role)


def module_rows(module_name: str) -> list:
    """The (part, role, Location-in-module-frame) rows of a code-driven assemblies/<module>.py."""
    if CAD_DIR not in sys.path:
        sys.path.insert(0, CAD_DIR)
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
    """A fresh copy of parts.load(part_name).gen_step() at a WORLD placement of its reference frame,
    optionally re-expressed in another frame (`into` = that frame's world Location, so the result
    is `into^-1 * world * LOCAL_FROM_REF^-1 * local`). Used for per-link meshes."""
    if CAD_DIR not in sys.path:
        sys.path.insert(0, CAD_DIR)
    mod = parts.load(part_name)
    local_from_ref = getattr(mod, "LOCAL_FROM_REF", None) or Location()
    loc = world * local_from_ref.inverse()
    if into is not None:
        loc = into.inverse() * loc
    return mod.gen_step().moved(loc)


def place_world(part_name: str, key: str, into=None):
    """place_world_at() for a part occurrence key of placements.json."""
    return place_world_at(part_name, P.location(key, "world"), into)
