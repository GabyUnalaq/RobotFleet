import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

from .GenericTopic import GenericSubscriber

""" Lidar Sensor
This module provides a LidarSensor class that subscribes to the Lidar scan topic
and processes the Lidar data to generate a point cloud.
"""


class LidarSensor(GenericSubscriber):
    """
    Class to handle the Lidar scan.

    Attributes:
        point_cloud (np.array[2, n]): List of points in the point cloud.
    """
    TOPIC_NAME = "/scan"

    def __init__(self, env: str = None) -> None:
        """
        Initialize the Lidar class.
        """
        super().__init__(LaserScan, env)
        self._point_cloud = None

    def __str__(self) -> str:
        """
        Return a string representation of the Lidar data.
        """
        with self._lock:
            if self._data is None:
                return "No Lidar data available."
        return f"Lidar Data:\n" \
               f"   Point Cloud: {self.point_cloud}"

    def _topic_callback(self, data):
        """
        Callback function to handle Lidar data.
        """
        super()._topic_callback(data)
        self._process_lidar_data(data)

    def _process_lidar_data(self, data) -> np.array:
        """
        Process the Lidar data.
        """
        ranges = np.array(data.ranges)
        # angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        angles = data.angle_min + np.arange(len(data.ranges)) * data.angle_increment

        # Trim to match lengths
        min_len = min(len(ranges), len(angles))
        ranges = ranges[:min_len]
        angles = angles[:min_len]

        # Validate points
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        valid = np.isfinite(x) & np.isfinite(y)
        x = x[valid]
        y = y[valid]

        point_cloud = np.vstack((x, y)).T
        
        with self._lock:
            self._point_cloud = point_cloud

    @property
    def point_cloud(self) -> np.array:
        """
        Get the point cloud data.
        """
        with self._lock:
            return self._point_cloud


if __name__ == "__main__":
    """
    Test the LidarSensor class by visualizing the point cloud
    in a Pygame window.
    """
    import signal
    import pygame

    WINDOW_WIDTH = 800
    WINDOW_HEIGHT = 800
    SCALE = 100.0  # 1 meter = 100 pixels

    def shutdown_handler(signum, frame):
        rospy.signal_shutdown("Shutting down LidarSensor.")
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.init_node('dev_lidar_reader', anonymous=True)
    lidar = LidarSensor("bot02")
    rate = rospy.Rate(10)

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Lidar Visualization")
    center = (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2)

    clock = pygame.time.Clock()
    while not rospy.is_shutdown():
        if pygame.event.peek(pygame.QUIT):
            rospy.signal_shutdown("Shutting down LidarSensor.")
            break

        if not lidar.is_data_stale():
            cloud = lidar.point_cloud
            screen.fill((0, 0, 0))

            # Draw robot center
            pygame.draw.circle(screen, (0, 255, 0), center, 5)

            for x, y in cloud:
                sx = int(center[0] + x * SCALE)
                sy = int(center[1] - y * SCALE)
                pygame.draw.circle(screen, (255, 255, 255), (sx, sy), 2)

        pygame.display.flip()
        clock.tick(10)

    pygame.quit()
    print("Done.")
