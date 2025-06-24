import rospy
import numpy as np
from nav_msgs.msg import Odometry

from utils import Transform
from .GenericTopic import GenericSubscriber

""" Odometry Sensor
This module provides an OdometrySensor class that subscribes to the odometry topic
and provides methods to access the robot's position, orientation, linear velocity,
and angular velocity data.
"""


class OdometrySensor(GenericSubscriber):
    """
    Class to handle the odometry data.

    Attributes:
        position (np.ndarray[3]): The position of the Robot.
        orientation (np.ndarray[4]): The orientation of the Robot.
        matrix (np.ndarray[4, 4]): The transformation matrix of the Robot.
        linear_velocity (np.ndarray[3]): The linear velocity of the Robot.
        angular_velocity (np.ndarray[3]): The angular velocity of the Robot.
    """
    TOPIC_NAME = "/odom"

    def __init__(self, env: str = None) -> None:
        """
        Initialize the Odometry class.
        """
        super().__init__(Odometry, env)

    def __str__(self) -> str:
        """
        Return a string representation of the Odometry data.
        """
        with self._lock:
            if self._data is None:
                return "No Odometry data available."
        return f"Odometry Data:\n" \
               f"   Position: {self.position}\n" \
               f"   Orientation: {self.orientation}\n" \
               f"   Linear Velocity: {self.linear_velocity}\n" \
               f"   Angular Velocity: {self.angular_velocity}"

    @property
    def position(self) -> np.ndarray:
        """
        Get the position of the robot.

        Returns:
            np.ndarray: Position as a 3D vector (x, y, z).
        """
        with self._lock:
            return np.array([
                self._data.pose.pose.position.x,
                self._data.pose.pose.position.y,
                self._data.pose.pose.position.z
            ])

    @property
    def orientation(self) -> np.ndarray:
        """
        Get the orientation of the robot.

        Returns:
            np.ndarray: Orientation as a quaternion (x, y, z, w).
        """
        with self._lock:
            return np.array([
                self._data.pose.pose.orientation.x,
                self._data.pose.pose.orientation.y,
                self._data.pose.pose.orientation.z,
                self._data.pose.pose.orientation.w
            ])

    @property
    def matrix(self) -> Transform:
        """
        Get the transformation matrix of the robot.

        Returns:
            Transform: Transformation matrix.
        """
        translation = self.position
        rotation = self.orientation
        return Transform(translation, rotation)

    @property
    def linear_velocity(self) -> np.ndarray:
        """
        Get the linear velocity of the robot.

        Returns:
            np.ndarray: Linear velocity as a 3D vector (x, y, z).
        """
        with self._lock:
            return np.array([
                self._data.twist.twist.linear.x,
                self._data.twist.twist.linear.y,
                self._data.twist.twist.linear.z
            ])

    @property
    def angular_velocity(self) -> np.ndarray:
        """
        Get the angular velocity of the robot.

        Returns:
            np.ndarray: Angular velocity as a 3D vector (x, y, z).
        """
        with self._lock:
            return np.array([
                self._data.twist.twist.angular.x,
                self._data.twist.twist.angular.y,
                self._data.twist.twist.angular.z
            ])


if __name__ == "__main__":
    import signal

    def shutdown_handler(signum, frame):
        rospy.signal_shutdown("Shutting down ArmCtrl.")
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.init_node('dev_odom_reader', anonymous=True)
    odom = OdometrySensor("bot01")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        print(odom)
        rate.sleep()

    print("Done.")
