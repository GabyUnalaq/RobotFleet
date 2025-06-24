import numpy as np
from scipy.spatial.transform import Rotation as R

""" Dummy Odometry Sensor
This class simulates an odometry sensor for a robot.
It provides methods to update the odometry data based on the robot's
state and retrieve the odometry data.
"""


class OdometrySensor:
    """
    Class to handle the odometry data.

    Attributes:
        position (np.ndarray[3]): The position of the Robot.
        orientation (np.ndarray[4]): The orientation of the Robot.
        linear_velocity (np.ndarray[3]): The linear velocity of the Robot.
        angular_velocity (np.ndarray[3]): The angular velocity of the Robot.

        pose (Pose): The position of the Robot:
            - position (x, y, z)
            - orientation (x, y, z, w)
        twist (Twist): The velocity of the Robot.
            - linear (x, y, z)
            - angular (x, y, z)
    """
    def __init__(self, env: str = None) -> None:
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.linear = np.array([0.0, 0.0, 0.0])
        self.angular = np.array([0.0, 0.0, 0.0])

    def update(self, bot, dt):
        # Update position
        self.position = np.array([bot.pos[0], bot.pos[1], 0.0])

        # Update orientation
        self.orientation = np.array(R.from_euler('z', bot.theta).as_quat())

        # Update linear
        vx = (bot.pos[0] - bot.prev_pos[0]) / dt
        vy = (bot.pos[1] - bot.prev_pos[1]) / dt
        self.linear = np.array([vx, vy, 0.0])

        # Update angular
        self.angular = np.array([0.0, 0.0, (bot.theta - bot.prev_theta) / dt])

    def get_odom_data(self) -> dict:
        return {
            "pose": {
                "position": self.position,
                "orientation": self.orientation
            },
            "twist": {
                "linear": self.linear,
                "angular": self.angular
            }
        }

    @property
    def linear_velocity(self) -> np.ndarray:
        return self.linear

    @property
    def angular_velocity(self) -> np.ndarray:
        return self.angular
