import rospy
from sensor_msgs.msg import Imu
import numpy as np

from .GenericTopic import GenericSubscriber

""" IMU Sensor
This module provides an ImuSensor class that subscribes to the IMU data topic
and provides methods to access the orientation, angular velocity, and linear acceleration data.
"""


class ImuSensor(GenericSubscriber):
    """
    Class to handle IMU data.

    Attributes:
        orientation: Quaternion (x, y, z, w)
        angular_velocity: Angular velocity (x, y, z)
        linear_acceleration: Linear acceleration (x, y, z)
    """
    TOPIC_NAME = "/imu/data"

    def __init__(self, env: str = None) -> None:
        """
        Initialize the IMU class.
        """
        super().__init__(Imu, env)

    def __str__(self) -> str:
        """
        Return a string representation of the IMU data.
        """
        with self._lock:
            data = self._data
        if data is None:
            return "No IMU data available."
        return f"IMU Data:\n" \
               f"   Orientation: {data.orientation}\n" \
               f"   Angular Velocity: {data.angular_velocity}\n" \
               f"   Linear Acceleration: {data.linear_acceleration}"

    @property
    def orientation(self) -> np.ndarray:
        """
        Get the orientation data.

        Returns:
            np.ndarray: Orientation as a quaternion (x, y, z, w).
        """
        with self._lock:
            return np.array([
                self._data.orientation.x,
                self._data.orientation.y,
                self._data.orientation.z,
                self._data.orientation.w
            ])
    
    @property
    def angular_velocity(self) -> np.ndarray:
        """
        Get the angular velocity data.

        Returns:
            np.ndarray: Angular velocity (x, y, z).
        """
        with self._lock:
            return np.array([
                self._data.angular_velocity.x,
                self._data.angular_velocity.y,
                self._data.angular_velocity.z
            ])

    @property
    def linear_acceleration(self) -> np.ndarray:
        """
        Get the linear acceleration data.

        Returns:
            np.ndarray: Linear acceleration (x, y, z).
        """
        with self._lock:
            return np.array([
                self._data.linear_acceleration.x,
                self._data.linear_acceleration.y,
                self._data.linear_acceleration.z
            ])


if __name__ == "__main__":
    import signal
    from utils import quaternion_to_euler

    def shutdown_handler(signum, frame):
        rospy.signal_shutdown("Shutting down ImuSensor.")
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.init_node('dev_imu_reader', anonymous=True)
    imu = ImuSensor("bot01")
    rate = rospy.Rate(1)

    def extract_yaw_angle(quat):
        _, _, yaw = quaternion_to_euler(quat)
        yaw_deg = np.rad2deg(yaw)
        print("Yaw (deg): {:.2f}".format(yaw_deg))

    while not rospy.is_shutdown():
        print(imu)
        extract_yaw_angle(imu.orientation)
        rate.sleep()

    print("Done.")
