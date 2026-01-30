"""Rotation utility functions."""

import numpy as np


class Quaternion:
    """
    Implements quaternion based opertions
    """

    def __init__(self, q):

        q = np.asarray(q, dtype=float)
        assert q.shape == (4,)
        self.x = q[1]
        self.y = q[2]
        self.z = q[3]
        self.w = q[0]

    def get_quaternion_vec(self):
        """
        Returns the vector part of the quaternion

        Returns
        -------
        qv: ndarray
            [qx, qy, qz]
        """

        return np.array([self.x, self.y, self.z])

    def to_numpy(self):
        """
        Returns the quaternion as ndarray

         Returns
        -------
        qv: ndarray
            [qw, qx, qy, qz]
        """

        return np.array([self.w, self.x, self.y, self.z])

    def conjugate(self):
        """
        Returns the quaternion conjugate q = [w, x, y, z]

        Returns
        -------
        q* : Quaternion
             [w, -x, -y, -z]

        """

        q_conj = np.array([self.w, -self.x, -self.y, -self.z])

        return Quaternion(q_conj)

    def inverse(self):
        """
        Returns the quaternion inverse q = [w, x, y, z]

        Returns
        -------
        q* : Quaternion
             [w, -x, -y, -z] (normalized)
        """

        return self.conjugate().normalize()

    def normalize(self):
        """
        Normalize quaternion to unit length
        """

        norm = np.linalg.norm(self.to_numpy())

        q_norm = (
            self.to_numpy() / norm if norm > 1e-10 else np.array([1.0, 0.0, 0.0, 0.0])
        )
        return Quaternion(q_norm)

    def to_rotation_matrix(self):
        """
        Convert quaternion to rotation matrix.

        Returns
        -------
        np.ndarray
            Rotation matrix (3, 3)
        """
        qw, qx, qy, qz = self.w, self.x, self.y, self.z

        R = np.array(
            [
                [
                    qw**2 + qx**2 - qy**2 - qz**2,
                    2 * (qx * qy - qw * qz),
                    2 * (qx * qz + qw * qy),
                ],
                [
                    2 * (qx * qy + qw * qz),
                    qw**2 - qx**2 + qy**2 - qz**2,
                    2 * (qy * qz - qw * qx),
                ],
                [
                    2 * (qx * qz - qw * qy),
                    2 * (qy * qz + qw * qx),
                    qw**2 - qx**2 - qy**2 + qz**2,
                ],
            ]
        )

        return R

    @staticmethod
    def multiply(q1: "Quaternion", q2: "Quaternion") -> "Quaternion":
        """
        Multiply two quaternions

        Params
        ------
        q1: Quaternion
        q2: Quaternion

        Returns
        -------
        q: Quaternion
            q1 * q2
        """

        q1_vec = q1.get_quaternion_vec()
        q2_vec = q2.get_quaternion_vec()

        q_vec = q1.w * q2_vec + q2.w * q1_vec + np.cross(q1_vec, q2_vec)

        q = np.array(
            [
                q1.w * q2.w - np.dot(q1_vec, q2_vec),
                *q_vec,
            ]
        )
        return Quaternion(q).normalize()

    @staticmethod
    def rotate(x, q: "Quaternion"):
        """
        Rotate x vector using quaternion q

        :param x: Description
        :param q: Description
        :type q: "Quaternion"
        """

        q_conj = Quaternion.conjugate(q)
        x_quat = Quaternion(np.array([0, *x]))
        x_rotated_quat = Quaternion.multiply(q, Quaternion.multiply(x_quat, q_conj))

        x_rotated = x_rotated_quat.get_quaternion_vec()
        return x_rotated
