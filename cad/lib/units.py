"""Unit / modelling globals with no dependencies - a leaf module.

lib/params.py re-exports these (`from lib.params import NUDGE` keeps working everywhere);
lib/cycloidal/ imports them from HERE, because lib/params.py imports lib/cycloidal/params.py and
the drive package must never import back up into lib/params.py.
"""

IN = 25.4               # mm per inch (some SolidWorks exports are inch-unit; OCCT converts on import)
NUDGE = 0.01            # tiny overshoot so boolean cuts punch fully through a face
