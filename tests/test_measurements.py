"""Tests for measurement models."""

import numpy as np
import pytest
from eskf.measurements import IMUMeasurement, GNSSMeasurement


class TestIMUMeasurement:
    """Tests for IMU measurement model."""

    def test_initialization(self):
        """Test IMU initialization."""
        imu = IMUMeasurement(accel_noise_std=0.1, gyro_noise_std=0.05)
        assert imu.accel_noise_std == 0.1
        assert imu.gyro_noise_std == 0.05


class TestGNSSMeasurement:
    """Tests for GNSS measurement model."""

    def test_initialization(self):
        """Test GNSS initialization."""
        gnss = GNSSMeasurement(position_noise_std=1.0)
        assert gnss.position_noise_std == 1.0
