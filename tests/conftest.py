"""Configuración común de pytest: raíz del repo en sys.path y RNG reproducible."""
import os
import sys

import numpy as np
import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


@pytest.fixture
def rng():
    return np.random.default_rng(12345)
