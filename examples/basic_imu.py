"""Basic example: ESKF with IMU measurements only."""

import numpy as np
from eskf.core import ESKF
from eskf.measurements import IMUMeasurement


def main():
    """Run basic IMU-only example."""
    # Initialize filter
    ekf = ESKF(state_dim=9, measurement_dim=3)
    imu = IMUMeasurement(accel_noise_std=0.1, gyro_noise_std=0.05)

    # Simulation parameters
    dt = 0.01  # 100 Hz
    duration = 10.0
    num_steps = int(duration / dt)

    # Storage for results
    states = np.zeros((num_steps, ekf.state_dim))

    # Main loop
    for i in range(num_steps):
        t = i * dt

        # Predict
        u = np.array([0, 0, 9.81])  # Gravity acceleration
        ekf.predict(u, dt)

        # Simulate measurement
        # z = imu.measure(ekf.x, dt)
        # ekf.update(z)

        states[i] = ekf.x

    print(f"Simulation complete: {num_steps} steps, {duration} seconds")
    print(f"Final state: {ekf.x}")


if __name__ == "__main__":
    main()
