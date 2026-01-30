import numpy as np
from eskf.utils.rotations import Quaternion


def test_identity():
    q = Quaternion([1, 0, 0, 0])
    R = q.to_rotation_matrix()
    assert np.allclose(R, np.eye(3))


def test_normalization():
    q = Quaternion([2, 0, 0, 0]).normalize()
    assert np.allclose(q.to_numpy(), np.array([1, 0, 0, 0]))


def test_multiplication_identity():
    q = Quaternion([1, 0, 0, 0])
    p = Quaternion([0, 1, 0, 0])  # 180 deg about x-axis
    qp = Quaternion.multiply(q, p)
    assert np.allclose(qp.to_numpy(), p.to_numpy())


def test_inverse():
    q = Quaternion([0.9238795, 0.3826834, 0, 0])  # 45 deg about x
    q_inv = q.inverse()
    prod = Quaternion.multiply(q, q_inv)
    assert np.allclose(prod.to_numpy(), np.array([1, 0, 0, 0]), atol=1e-6)


def test_rotation_90deg_z():
    theta = np.pi / 2
    q = Quaternion([np.cos(theta / 2), 0, 0, np.sin(theta / 2)])  # rotation about z
    v = np.array([1, 0, 0])
    v_rot = Quaternion.rotate(v, q)
    assert np.allclose(v_rot, np.array([0, 1, 0]), atol=1e-6)


def test_rotation_matrix_consistency():
    q1 = Quaternion([0.9238795, 0.3826834, 0, 0])  # 45 deg x
    q2 = Quaternion([0.9238795, 0, 0.3826834, 0])  # 45 deg y

    q12 = Quaternion.multiply(q1, q2)

    R1 = q1.to_rotation_matrix()
    R2 = q2.to_rotation_matrix()
    R12 = q12.to_rotation_matrix()

    assert np.allclose(R12, R1 @ R2, atol=1e-6)


def test_small_angle_behavior():
    theta = np.array([1e-6, 0, 0])
    q = Quaternion([1, theta[0] / 2, theta[1] / 2, theta[2] / 2]).normalize()
    v = np.array([0, 1, 0])
    v_rot = Quaternion.rotate(v, q)

    # small angle approx: v_rot ≈ v + θ × v
    approx = v + np.cross(theta, v)
    assert np.allclose(v_rot, approx, atol=1e-6)
