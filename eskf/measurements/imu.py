"""IMU measurement model."""

import numpy as np


class IMUMeasurement:
    """Inertial Measurement Unit measurement model."""

    def __init__(self, accel_noise_std, gyro_noise_std):
        """
        Initialize IMU measurement model.

        Parameters
        ----------
        accel_noise_std : float
            Standard deviation of accelerometer noise
        gyro_noise_std : float
            Standard deviation of gyroscope noise
        """
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std

    def measure(self, state, dt):
        """
        Simulate IMU measurement.

        Parameters
        ----------
        state : np.ndarray
            Current state
        dt : float
            Time step

        Returns
        -------
        tuple
            Acceleration and angular velocity measurements
        """
        pass
