"""One build per test process: the parts, modules and placed occurrences the geometry tests share.

The geometry tests measure the same shapes over and over - a part in every file that places it, a module in every
clearance it takes part in. These return each one built once per process (functools.cache; a pytest-xdist worker is
a process of its own), so a cached shape is SHARED by every test that asks for it, and read-only:
- measure it: tests/helpers.py interference / is_inside, cadgen.geometry closest_points, distance_to, volume
  (lib.reference.solid_volume), bounding_box(), faces() / edges(), is_valid, BRepGProp;
- move a copy: .moved() and .rotate() return new shapes;
- never set its label / color / children / parent, .move() / .locate() it, mesh it (tessellate, export_stl: the mesh
  is stored on the shared faces), hand it to build123d's booleans (- + & .cut() .intersect(): their default operator
  may modify its operands) or to a Compound / assembly() as a child (that reparents it) - build your own for that
  (parts.build, lib.models.raw) or take a copy.deepcopy().
tests/conftest.py fails the test that changed one.
"""
from __future__ import annotations

import functools
import importlib

import parts
from assemblies._occurrences import placement_at
from lib import placements as P
from lib.models import model_of, raw

_KEPT: list[list] = []   # [shape, its fingerprint]: tests/conftest.py's check after every test


def _fingerprint(shape) -> tuple:
    """What a test could change on a shared shape without building anything: its label, colour, tree and location."""
    loc = shape.location
    color = getattr(shape, "color", None)
    return (shape.label, None if color is None else tuple(color), id(shape.parent), len(shape.children),
            tuple(loc.position), tuple(loc.orientation))


def changed() -> list[str]:
    """The cached shapes changed since they were built (or since the last call, which takes them as they are now)."""
    found = []
    for kept in _KEPT:
        now = _fingerprint(kept[0])
        if now != kept[1]:
            found.append(f"{kept[0].label or type(kept[0]).__name__}: {kept[1]} -> {now}")
            kept[1] = now
    return found


def _keep(shape):
    _KEPT.append([shape, _fingerprint(shape)])
    return shape


@functools.cache
def part(name: str):
    """parts.build(name): the part at its local origin."""
    return _keep(parts.build(name))


@functools.cache
def legacy(name: str):
    """The part's REFERENCE_BUILD(): the build a part that has left its reference is matched with."""
    return _keep(parts.load(name).REFERENCE_BUILD())


@functools.cache
def model(fn):
    """lib.models.raw(fn): a model's body - a module with its children in the module frame."""
    return _keep(raw(fn))


def leaf(fn, label: str):
    """model(fn)'s child labelled `label`, in the module frame. It keeps its parent: its .moved() copies the whole
    module, so cache what you move."""
    return next(c for c in model(fn).children if c.label == label)


@functools.cache
def placed(key: str, into: str | None = None):
    """An occurrence of placements.json at its world pose - a part as assemblies._occurrences.place_world() puts it, or a
    designed module - or re-expressed in the frame of the occurrence `into`."""
    o = P.OCCURRENCES[key]
    world = P.location(key, "world")
    frame = None if into is None else P.location(into, "world")
    if o["kind"] == "part":
        return _keep(part(o["part"]).moved(placement_at(o["part"], world, frame)))
    if not o.get("designed"):
        raise ValueError(f"{key}: only a part or a designed module is placed here")
    fn = model_of(importlib.import_module(f"assemblies.{o['part']}"))
    return _keep(model(fn).moved(world if frame is None else frame.inverse() * world))
