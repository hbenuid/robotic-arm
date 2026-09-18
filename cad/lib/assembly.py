"""Native build123d assembly nodes - what cadgen packages as the STEP component tree.

An assembly is a labelled `Compound(children=[...])`: `assembly(name, children)` builds one node
(the root of a model, or a group / module inside it), `label_shape(shape, name, *details)` labels a
child and `label_text` gives the compact STEP-friendly label (`j3_coupler:j2`) - both from cadgen,
which normalises the tokens. cadgen 0.6.5 deprecated its `AssemblyHelper` wrapper (its `add` /
`add_module` / `build` were exactly these three lines); joints are `@step(kinematics=...)` data
(`cadgen.revolute(...)` mates between `#label` refs), not helper frames.
"""
from build123d import Compound
from cadgen.assembly import label_shape, label_text  # noqa: F401

__all__ = ["assembly", "label_shape", "label_text"]


def assembly(name: str, children, *details, color=None) -> Compound:
    """`Compound(label=label_text(name, *details), children=children)`, optionally colored.

    The children keep their own locations (cadgen's STEP packager reads those, never a location on
    the node), labels and colors; a compound-level color does not cascade to them."""
    node = Compound(label=label_text(name, *details), children=list(children))
    if color is not None:
        node.color = color
    return node
