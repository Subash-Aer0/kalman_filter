"""Lie groups for rotation and transformation matrices."""

import numpy as np
from utils.rotations import Quaternion


class SO3:
    """Special Orthogonal group SO(3) for 3D rotations."""

    @staticmethod
    def exp(phi):
        """
        Exponential map from so(3) to SO(3).

        Parameters
        ----------
        phi : np.ndarray
            Angular velocity vector (3,)

        Returns
        -------
        np.ndarray
            Quaternion (4, 1)
        """
        phi_norm = np.linalg.norm(phi)
        theta = phi_norm / 2

        if phi_norm < 1e-8:
            return np.array([1, phi[0] / 2, phi[1] / 2, phi[2] / 2])

        axis = phi / phi_norm
        print(axis)
        q = np.array(
            [
                np.cos(theta),
                axis[0] * np.sin(theta),
                axis[1] * np.sin(theta),
                axis[2] * np.sin(theta),
            ]
        )

        return q

    @staticmethod
    def log(q):
        """
        Logarithmic map from SO(3) to so(3).

        Parameters
        ----------
        q : np.ndarray
            Quaternion (4, 1)

        Returns
        -------
        np.ndarray
            Angular velocity vector (3,)
        """
        qv = q[1:4]
        qv_norm = np.linalg.norm(qv)
        phi = 2 * np.atan2(qv_norm, q[0])

        axis = qv / qv_norm

        return phi * axis

    @staticmethod
    def integrate(q: Quaternion, dtheta) -> Quaternion:
        """
        Zeroth order integration for the quaternion q

        Parameters
        ----------
        q: Quaternion Quaternion to integrate
        dtheta: NDArray

        Returns
        -------
        q_k+1 = q * dtheta
        """

        dtheta_exp = SO3.exp(dtheta)

        q_next = Quaternion.multiply(q, Quaternion(dtheta_exp))

        return q_next

    @staticmethod
    def skew_matrix(omega):

        return np.array(
            [
                [0, -omega[2], omega[1]],
                [omega[2], 0, -omega[1]],
                [-omega[2], omega[1], 0],
            ]
        )


class SE3:
    """Special Euclidean group SE(3) for 3D transformations."""

    @staticmethod
    def exp(xi):
        """
        Exponential map from se(3) to SE(3).

        Parameters
        ----------
        xi : np.ndarray
            Twist vector (6,)

        Returns
        -------
        np.ndarray
            Transformation matrix (4, 4)
        """
        pass

    @staticmethod
    def log(T):
        """
        Logarithmic map from SE(3) to se(3).

        Parameters
        ----------
        T : np.ndarray
            Transformation matrix (4, 4)

        Returns
        -------
        np.ndarray
            Twist vector (6,)
        """
        pass
