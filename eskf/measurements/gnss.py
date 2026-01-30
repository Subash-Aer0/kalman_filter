"""GNSS/GPS measurement model."""

import numpy as np


class GNSSMeasurement:
    """GNSS/GPS measurement model."""

    def __init__(self, position_noise_std):
        """
        Initialize GNSS measurement model.

        Parameters
        ----------
        position_noise_std : float
            Standard deviation of position measurement noise
        """
        self.position_noise_std = position_noise_std

    def measure(self, state):
        """
        Get GNSS position measurement.

        Parameters
        ----------
        state : np.ndarray
            Current state

        Returns
        -------
        np.ndarray
            Position measurement
        """
        pass
