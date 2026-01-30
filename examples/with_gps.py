"""Example: ESKF with IMU and GPS measurements."""

import numpy as np
from eskf.core import ESKF
from eskf.measurements import IMUMeasurement, GNSSMeasurement


def main():
    """Run example with IMU and GPS fusion."""
    # Initialize filter
    ekf = ESKF(state_dim=9, measurement_dim=6)
    imu = IMUMeasurement(accel_noise_std=0.1, gyro_noise_std=0.05)
    gnss = GNSSMeasurement(position_noise_std=1.0)

    # Simulation parameters
    dt = 0.01  # 100 Hz
    gps_dt = 1.0  # GPS at 1 Hz
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

        # GPS update every gps_dt seconds
        if (i * dt) % gps_dt < dt:
            # z_gps = gnss.measure(ekf.x)
            # ekf.update(z_gps)
            pass

        states[i] = ekf.x

    print(f"Simulation complete: {num_steps} steps, {duration} seconds")
    print(f"Final state: {ekf.x}")


if __name__ == "__main__":
    main()
