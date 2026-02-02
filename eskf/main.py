import math
import os
import numpy as np
import matplotlib.pyplot as plt
from core.eskf import ESKF, ImuParameters
from core.lie_groups import SO3
from utils.rotations import Quaternion


def load_imu_parameters():
    params = ImuParameters()
    params.frequency = 200
    params.sigma_a_n = 0.019  # m/sqrt(s^3)
    params.sigma_w_n = 0.015  # rad/sqrt(s)
    params.sigma_a_b = 0.0001  # m/sqrt(s^5)
    params.sigma_w_b = 2.0e-5  # rad/sqrt(s^3)
    return params


def main():
    imu_data = np.loadtxt("./imu_noise.txt")
    gt_data = np.loadtxt("./traj_gt.txt")

    imu_parameters = load_imu_parameters()

    init_nominal_state = np.zeros((19,))
    init_nominal_state[:10] = gt_data[0, 1:]  # init p, q, v
    init_nominal_state[10:13] = 0  # init ba
    init_nominal_state[13:16] = 0  # init bg
    init_nominal_state[16:19] = np.array([0, 0, -9.81])  # init g
    estimator = ESKF(init_nominal_state, imu_parameters)

    test_duration_s = [0.0, 61.0]
    start_time = imu_data[0, 0]
    mask_imu = np.logical_and(
        imu_data[:, 0] <= start_time + test_duration_s[1],
        imu_data[:, 0] >= start_time + test_duration_s[0],
    )
    mask_gt = np.logical_and(
        gt_data[:, 0] <= start_time + test_duration_s[1],
        gt_data[:, 0] >= start_time + test_duration_s[0],
    )

    imu_data = imu_data[mask_imu, :]
    gt_data = gt_data[mask_gt, :]

    traj_est = [gt_data[0, :8]]
    update_ratio = 10  # control the frequency of ekf updating.
    sigma_measurement_p = 0.02  # in meters
    sigma_measurement_q = 0.015  # in rad
    sigma_measurement = np.eye(6)
    sigma_measurement[0:3, 0:3] *= sigma_measurement_p**2
    sigma_measurement[3:6, 3:6] *= sigma_measurement_q**2
    for i in range(1, imu_data.shape[0]):
        timestamp = imu_data[i, 0]

        # change the IMU data order q and v
        data = np.zeros((imu_data.shape[1],))
        print(imu_data[i, :])
        data[6] = imu_data[i, 0]
        data[0:6] = imu_data[i, 1:]
        estimator.predict(data)
        if i % update_ratio == 0:
            # we assume the timestamps are aligned.
            assert math.isclose(gt_data[i, 0], timestamp)
            gt_pose = gt_data[i, 1:8].copy()  # gt_pose = [p, q]
            # add position noise
            gt_pose[:3] += (
                np.random.randn(
                    3,
                )
                * sigma_measurement_p
            )
            # add rotation noise, u = [1, 0.5 * noise_angle_axis]
            # u = 0.5 * np.random.randn(4,) * sigma_measurement_q
            # u[0] = 1
            u = (
                np.random.randn(
                    3,
                )
                * sigma_measurement_q
            )
            qn = SO3.exp(u)
            print(gt_pose[3:])
            print(qn)
            gt_pose[3:] = Quaternion.multiply(
                Quaternion(gt_pose[3:]), Quaternion(qn)
            ).to_numpy()
            # update filter by measurement.
            estimator.update(gt_pose, sigma_measurement)

        print("[%f]:" % timestamp, estimator.nominal_state)
        frame_pose = np.zeros(
            8,
        )
        frame_pose[0] = timestamp
        frame_pose[1:] = estimator.nominal_state[:7]
        traj_est.append(frame_pose)

    # save trajectory to TUM format
    traj_est = np.array(traj_est)
    plot_traj(traj_est)
    # np.savetxt("./data/traj_gt_out.txt", gt_data[:, :8])
    # np.savetxt("./data/traj_esekf_out.txt", traj_est)


def plot_traj(data, title_prefix="traj", save_dir=None, show=True):
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]

    # 3D plot
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(x, y, z, label="est trajectory")
    ax.scatter(x[0], y[0], z[0], c="g", marker="o", label="start")
    ax.scatter(x[-1], y[-1], z[-1], c="r", marker="x", label="end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"{title_prefix} - 3D trajectory")
    ax.legend()
    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        fpath = os.path.join(save_dir, f"{title_prefix}_3d.png")
        fig.savefig(fpath)
        print(f"Saved 3D plot to {fpath}")

    # time vs x,y,z
    fig2, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axs[0].plot(t, x, label="x")
    axs[0].set_ylabel("x [m]")
    axs[0].grid(True)
    axs[1].plot(t, y, label="y")
    axs[1].set_ylabel("y [m]")
    axs[1].grid(True)
    axs[2].plot(t, z, label="z")
    axs[2].set_ylabel("z [m]")
    axs[2].set_xlabel("time [s]")
    axs[2].grid(True)
    fig2.suptitle(f"{title_prefix} - position vs time")
    plt.tight_layout(rect=[0, 0.0, 1, 0.96])
    if save_dir:
        fpath2 = os.path.join(save_dir, f"{title_prefix}_time_xyz.png")
        fig2.savefig(fpath2)
        print(f"Saved time-vs-xyz plot to {fpath2}")

    if show:
        plt.show()


if __name__ == "__main__":
    main()
