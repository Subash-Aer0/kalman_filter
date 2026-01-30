"""Extended Kalman Filter implementation."""

import numpy as np


class ESKF:
    """Extended Kalman Filter for state estimation."""

    def __init__(self, x0):
        """
        Initialize the ESKF.

        Parameters
        ----------
        x0 : ndarray
            Nominal initial state vector
        measurement_dim : int
            Dimension of the measurement vector
        """
        self.nominal_state = x0  # State estimate
        self.P = np.eye(x0.shape[0])  # Covariance matrix

    def predict(self, u, dt):
        """
        Prediction step of the filter.

        Parameters
        ----------
        u : np.ndarray
            Control input
        dt : float
            Time step
        """
        pass

    def update(self, z):
        """
        Update step of the filter.

        Parameters
        ----------
        z : np.ndarray
            Measurement vector
        """
        pass

    def predict_nominal_state(self, imu_measurements, method="euler"):
        """
        Propagates the nominal state vector

        Parameters
        ----------
        imu_measurements: ndarray
        """

        p, v, q, a_bias, w_bias, g = self.unpack_nominal_state_vector()

        q_next = 



    def unpack_nominal_state_vector(self):
        """ "
        Unpacks the nominal state vector
        """

        p = self.nominal_state[0:3]
        v = self.nominal_state[3:6]
        q = self.nominal_state[6:10]
        a_bias = self.nominal_state[10:13]
        w_bias = self.nominal_state[13:16]
        g = self.nominal_state[16:19]

        return p, v, q, a_bias, w_bias, g
