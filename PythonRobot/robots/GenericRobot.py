import numpy as np
from utils import Transform

""" Generic Robot Class
This class serves as a base for all robot implementations in the PythonRobot framework.
It provides methods for starting the robot, advancing its state, setting command velocities,
and retrieving sensor data such as IMU, odometry, and LIDAR.
"""

MAX_LINEAR_SPEED = 0.45
MAX_ANGULAR_TURN = 2.0

__all__ = [
    "GenericRobot",
    "MAX_LINEAR_SPEED",
    "MAX_ANGULAR_TURN"
]


class GenericRobot:
    TRANSBOT_LENGTH = 0.35
    TRANSBOT_WIDTH = 0.25
    def __init__(self, env: str = None):
        self.has_env = env is not None
        self.robot_name = env

        self.imu_sensor = None
        self.odom_sensor = None
        self.lidar_sensor = None
        self.bot_ctrl = None
        self.frames = None
        self.odom_to_world = Transform()

        self.obstacle = None  # Used for visualisation

        self.start()

    def __del__(self):
        """
        Destructor calls the shutdown method.
        """
        self.shutdown()

    def start(self):
        """
        Method to start the robot.
        """
        raise NotImplementedError("start was called without implementation")
    
    def advance(self, **kwargs):
        """
        Advance the robot.
        """
        raise NotImplementedError("advance was called without implementation")

    def shutdown(self):
        """
        Method that gracefully shuts down the robot.
        """
        pass

    def initial_pose(self, x: float=0.0, y: float=0.0, theta: float=0.0) -> "GenericRobot":
        """
        Set the initial pose of the robot in the world frame.
        """
        self.odom_to_world = Transform(
            translation=np.array([x, y, 0.0]),
            rotation=np.array([0.0, 0.0, theta])
        ).mult(self.odom_matrix.inverse())
        return self

    def set_cmd_vel(self, linear_speed: float, angular_turn: float):
        """
        Method to set the command velocity of the robot.
        """
        raise NotImplementedError("set_cmd_vel was called without implementation")

    @property
    def position(self) -> np.ndarray:
        """
        Property to get the position of the robot in the world frame.
        """
        world_pos = self.odom_to_world.mult(Transform(translation=self.odom_sensor.position))
        return world_pos.translation

    @property
    def orientation(self) -> np.ndarray:
        """
        Property to get the orientation of the robot in the world frame.
        """
        world_rot = self.odom_to_world.mult(Transform(rotation=self.odom_sensor.orientation))
        return world_rot.rotation

    @property
    def odom_matrix(self) -> Transform:
        """
        Get the transformation matrix of the robot 
        """
        translation = self.position
        rotation = self.orientation
        return Transform(translation, rotation)

    @property
    def lidar_point_cloud(self) -> np.ndarray:
        """
        Get the point cloud from the Lidar sensor.
        """
        raise NotImplementedError("lidar_point_cloud was called without implementation")
