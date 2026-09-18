"""Fixtures shared by the cycloidal-drive tests (the session guards are in tests/conftest.py)."""
import pytest

from lib.cycloidal import stack_positions
from tests.cycloidal.helpers import CFG


@pytest.fixture(scope="module")
def stack():
    """Z (and eccentric X) position of every part in the module frame, for the default config."""
    return stack_positions(CFG)
