"""AssemblyHelper shim.

Uses the cad@text-to-cad plugin's `cadgen.assembly.AssemblyHelper` (cadgen is a locked
dependency of this project; the plugin's pre-0.4 name `cadpy` is tried second), else a minimal
drop-in with the same add()/add_module()/build() semantics - a labelled build123d Compound
tree, which is exactly what cadgen's build() returns when no joints are declared. So the
assemblies and tests work even without the plugin runtime.

Source-level joints/mates (rigid_frame, revolute_frame, face_to_face, ...) need the real
cadgen.
"""
from __future__ import annotations

try:
    from cadgen.assembly import AssemblyHelper, label_shape, label_text  # type: ignore  # noqa: F401

    HAVE_CADGEN = True
except ModuleNotFoundError:
    try:
        from cadpy.assembly import AssemblyHelper, label_shape, label_text  # type: ignore  # noqa: F401

        HAVE_CADGEN = True
    except ModuleNotFoundError:
        HAVE_CADGEN = False

if not HAVE_CADGEN:
    from build123d import Compound

    def label_text(name, *details) -> str:
        """Compact STEP-friendly label such as `j3_coupler:j2` (mirrors cadpy.assembly)."""
        tokens = []
        for value in (name, *details):
            token = "_".join(str(value).strip().replace(":", "_").split())
            if not token:
                raise ValueError("label tokens must be non-empty")
            tokens.append(token)
        return ":".join(tokens)

    def label_shape(shape, name, *details, color=None):
        shape.label = label_text(name, *details)
        if color is not None:
            shape.color = color
        return shape

    class AssemblyHelper:
        """Minimal stand-in for cadpy.assembly.AssemblyHelper (no joints/mates)."""

        def __init__(self, name: str) -> None:
            self.label = label_text(name)
            self.children: list = []
            self.relations: list = []

        def add(self, shape, name, *details, color=None):
            label_shape(shape, name, *details, color=color)
            self.children.append(shape)
            return shape

        def add_module(self, name, children, *details, color=None):
            module = Compound(label=label_text(name, *details), children=list(children))
            if color is not None:
                module.color = color
            self.children.append(module)
            return module

        def build(self):
            return Compound(label=self.label, children=list(self.children))

        def __getattr__(self, item):  # rigid_frame, revolute_frame, face_to_face, ...
            raise AttributeError(
                f"AssemblyHelper.{item} needs cadgen (cad@text-to-cad plugin runtime)"
            )
