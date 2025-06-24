import numpy as np
from scipy.spatial.transform import Rotation as R

""" Dummy IMU Sensor
This class simulates an Inertial Measurement Unit (IMU) sensor for a robot.
It provides methods to update the IMU data based on the robot's state and retrieve the IMU data.
"""


class ImuSensor:
    """
    Class to handle IMU data.

    Attributes:
        orientation: Quaternion (x, y, z, w)
        angular_velocity: Angular velocity (x, y, z)
        linear_acceleration: Linear acceleration (x, y, z)
    """

    def __init__(self, env: str = None) -> None:
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.angular_velocity = (0.0, 0.0, 0.0)
        self.linear_acceleration = (0.0, 0.0, 0.0)

        self.prev_vx = 0.0
        self.prev_vy = 0.0

    def update(self, bot, dt):
        # Update orientation
        self.orientation = R.from_euler('z', bot.theta).as_quat()

        # Update angular velocity
        self.angular_velocity = [0.0, 0.0, (bot.theta - bot.prev_theta) / dt]

        # Update linear acceleration
        vx = (bot.pos[0] - bot.prev_pos[0]) / dt
        vy = (bot.pos[1] - bot.prev_pos[1]) / dt
        ax = (vx - self.prev_vx) / dt
        ay = (vy - self.prev_vy) / dt

        self.linear_acceleration = [ax, ay, 0.0]
        self.prev_vx = vx
        self.prev_vy = vy

    def get_imu_data(self) -> dict:
        return {
            "orientation": self.orientation,
            "angular_velocity": self.angular_velocity,
            "linear_acceleration": self.linear_acceleration
        }
