import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from robot._links import build_link  # noqa: E402
def gen_step(): return build_link(pathlib.Path(__file__).stem)
