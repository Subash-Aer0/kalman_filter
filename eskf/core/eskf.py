"""Extended Kalman Filter implementation."""

import numpy as np
from core.lie_groups import SO3
from utils.rotations import Quaternion


class ImuParameters:
    def __init__(self):
        self.frequency = 200
        self.sigma_a_n = 0.0  # acc noise.   m/(s*sqrt(s)), continuous noise sigma
        self.sigma_w_n = 0.0  # gyro noise.  rad/sqrt(s), continuous noise sigma
        self.sigma_a_b = 0.0  # acc bias     m/sqrt(s^5), continuous bias sigma
        self.sigma_w_b = 0.0  # gyro bias    rad/sqrt(s^3), continuous bias sigma


class ESKF:
    """Extended Kalman Filter for state estimation."""

    last_predicted_time = 0.0
    prev_imu = None

    def __init__(self, x0, imu_parameters):
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

        noise_covar = np.zeros((12, 12))
        # assume the noises (especially sigma_a_n) are isotropic so that we can precompute self.noise_covar and save it.
        noise_covar[0:3, 0:3] = (imu_parameters.sigma_a_n**2) * np.eye(3)
        noise_covar[3:6, 3:6] = (imu_parameters.sigma_w_n**2) * np.eye(3)
        noise_covar[6:9, 6:9] = (imu_parameters.sigma_a_b**2) * np.eye(3)
        noise_covar[9:12, 9:12] = (imu_parameters.sigma_w_b**2) * np.eye(3)
        G = np.zeros((18, 12))
        G[3:6, 3:6] = -np.eye(3)
        G[6:9, 0:3] = -np.eye(3)
        G[9:12, 6:9] = np.eye(3)
        G[12:15, 9:12] = np.eye(3)
        self.noise_covar = G @ noise_covar @ G.T

        # initialize error covariance matrix
        self.P = 0.01 * self.noise_covar

    def predict(self, imu_measurements):
        """
        Prediction step of the filter.

        Parameters
        ----------
        u : np.ndarray
            Control input
        dt : float
            Time step
        """
        if self.prev_imu is None:
            self.prev_imu = {
                "t": imu_measurements[6],
                "a": imu_measurements[0:3],
                "w": imu_measurements[3:6],
            }
            return None
        self.predict_error_cov(imu_measurements)
        self.nominal_state = self.predict_nominal_state(imu_measurements)

    def predict_nominal_state(self, imu_measurements):
        """
        Propagates the nominal state vector

        Parameters
        ----------
        imu_measurements: ndarray
        """

        p, v, q, a_bias, w_bias, g = self.unpack_nominal_state_vector()

        dt = imu_measurements[6] - self.prev_imu["t"]

        # IMU data at t_{n+1}
        a_n1 = imu_measurements[0:3]
        w_n1 = imu_measurements[3:6]

        # IMU data at t_{n}
        a_n = self.prev_imu["a"]
        w_n = self.prev_imu["w"]

        w_avg = 0.5 * (w_n + w_n1)

        # Midpoint integration for orientation
        q_nhalf = SO3.integrate(Quaternion(q), w_avg * dt / 2)
        q_n1 = SO3.integrate(Quaternion(q), w_avg * dt)

        # RK4 integration for velocity
        R_n = Quaternion(q).to_rotation_matrix()
        v_k1 = R_n @ (a_n - a_bias) + g

        R_nhalf = q_nhalf.to_rotation_matrix()
        v_k2 = R_nhalf @ (0.5 * (a_n + a_n1) - a_bias) + g

        v_k3 = v_k2

        R_n1 = q_n1.to_rotation_matrix()
        v_k4 = R_n1 @ (a_n1 - a_bias) + g

        v_n1 = v + (dt / 6) * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4)

        p_n1 = p + 0.5 * (v + v_n1) * dt

        state = np.array([*p_n1, *v_n1, *q_n1.to_numpy(), *a_bias, *w_bias, *g])
        self.prev_imu = {
            "t": imu_measurements[6],
            "a": imu_measurements[0:3],
            "w": imu_measurements[3:6],
        }

        return state

    def predict_error_cov(self, imu_measurements):

        am = imu_measurements[0:3]
        wm = imu_measurements[3:6]
        a_b = self.nominal_state[10:13]
        w_b = self.nominal_state[13:16]
        q = self.nominal_state[6:10]
        R = Quaternion(q).to_rotation_matrix()

        F = np.zeros((18, 18))
        F[0:3, 3:6] = np.eye(3)
        F[3:6, 6:9] = -R @ SO3.skew_matrix(am - a_b)
        F[3:6, 9:12] = -R
        F[6:9, 6:9] = -SO3.skew_matrix(wm - w_b)
        F[6:9, 12:15] = -np.eye(3)

        dt = imu_measurements[6] - self.prev_imu["t"]
        Fdt = F * dt
        Fdt2 = Fdt @ Fdt
        Fdt3 = Fdt2 @ Fdt

        Phi = np.eye(18) + Fdt + 0.5 * Fdt2 + (1.0 / 6.0) * Fdt3

        Qc_dt = 0.5 * dt * self.noise_covar
        self.P = Phi @ (self.P + Qc_dt) @ Phi.T + Qc_dt

    def update(self, measurements, measurement_cov):

        H = np.zeros((6, 18))
        H[0:3, 0:3] = np.eye(3, 3)
        H[3:6, 3:6] = np.eye(3, 3)

        PHt = self.P @ H.T

        K = PHt @ np.linalg.inv(H @ PHt + measurement_cov)

        self.P = (np.eye(18, 18) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

        ht_p = measurements[0:3]
        ht_q = measurements[3:7]
        q = Quaternion(self.nominal_state[6:10])

        delta_x = np.zeros((6, 1))
        delta_x[0:3, 0] = ht_p - self.nominal_state[0:3]
        delta_q = Quaternion.multiply(q.conjugate(), Quaternion(ht_q))
        delta_x[3:6] = SO3.log(delta_q.to_numpy()).reshape(-1, 1)

        errors = K @ delta_x

        self.nominal_state[0:3] += errors[0:3, 0]
        self.nominal_state[3:7] = SO3.integrate(
            Quaternion(self.nominal_state[3:7]), delta_x[3:6, 0]
        ).to_numpy()
        self.nominal_state[7:] = errors[6:, 0]

        G = np.eye(18)
        G[3:6, 3:6] = np.eye(3) - SO3.skew_matrix(0.5 * errors[3:6, 0])
        self.P = G @ self.P @ G.T

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
