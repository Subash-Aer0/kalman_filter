#!/usr/bin/env python3
"""Plot 3D trajectory and time vs x,y,z from a TUM-like trajectory file.

Expected file format: N x 8 where columns are [t, px, py, pz, qx, qy, qz, qw]
Default path: ./data/traj_esekf_out.txt
"""
import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_traj(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Trajectory file not found: {path}")
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 4:
        raise ValueError("Trajectory file must have at least 4 columns: t, x, y, z")
    return data


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


def main():
    p = argparse.ArgumentParser(description="Plot trajectory (3D and time vs x,y,z)")
    p.add_argument(
        "file",
        nargs="?",
        default="./data/traj_esekf_out.txt",
        help="trajectory file path (default: ./data/traj_esekf_out.txt)",
    )
    p.add_argument("--out", "-o", default=None, help="directory to save plot images")
    p.add_argument("--no-show", action="store_true", help="do not call plt.show()")
    args = p.parse_args()

    try:
        data = load_traj(args.file)
    except Exception as e:
        print(f"Error loading trajectory: {e}")
        sys.exit(1)

    plot_traj(
        data,
        title_prefix=os.path.splitext(os.path.basename(args.file))[0],
        save_dir=args.out,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
