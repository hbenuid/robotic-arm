"""The printed GT2 pulleys: the 90T pulley (gt2_pulley_90t) and its 120T variant (gt2_pulley_120t) as parametric
build123d, and the GT2 groove and toothed ring every printed pulley carries (the roll pulley's integral 90T too).

Pure-Python config here (lib/pulley/params.py); the build123d builders in lib/pulley/teeth.py and body.py (imported
explicitly by their users, so importing this package never pulls in OCCT). Never imports lib/params.py.
"""
from lib.pulley.params import DEFAULT, LEGACY, YAW, PulleyParams  # noqa: F401
