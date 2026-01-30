"""Tests for ESKF implementation."""

import numpy as np
import pytest
from eskf.core import ESKF


class TestESKF:
    """Tests for Extended Kalman Filter."""

    def test_initialization(self):
        """Test ESKF initialization."""
        ekf = ESKF(state_dim=6, measurement_dim=3)
        assert ekf.state_dim == 6
        assert ekf.measurement_dim == 3

    def test_predict_step(self):
        """Test prediction step."""
        pass

    def test_update_step(self):
        """Test update step."""
        pass
