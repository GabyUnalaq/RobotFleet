import numpy as np

""" Dummy Lidar Sensor
This class simulates a Lidar sensor that scans a predefined map and detects obstacles.
It provides methods to start, shutdown, and simulate the Lidar scan based on the robot's position and angle.
"""

LIDAR_MIN = 0.15  # meters
LIDAR_MAX = 12.0  # meters
LIDAR_ANGULAR_RESOLUTION = 1  # 1  # degree
LIDAR_ANGLE_JUMPS = 5


class LidarSensor:
    """
    Class to simulate a Lidar sensor in a predefined map

    Attributes:
        ranges (np.array): List of ranges from the Lidar scan.
        angles (np.array): List of angles corresponding to the ranges.
        point_cloud (np.array): List of points in the point cloud.
    """

    def __init__(self, env: str = None) -> None:
        """
        Initialize the Lidar class.
        """
        self.ranges: np.array = np.array([])
        self.angles: np.array = np.array([])
        self.point_cloud: np.array = np.array([])
        self.step = 0

        self.start()

    def __del__(self):
        """
        Destructor to clean up the Lidar sensor.
        """
        self.shutdown()

    def start(self):
        """
        Start the Lidar sensor.
        """
        # print("Lidar Sensor started.")
        pass

    def shutdown(self):
        """
        Shutdown the Lidar sensor.
        """
        # print("Lidar Sensor shutdown.")
        pass

    def simulate_lidar(self, pos: np.ndarray, obstacles: list, robot_angle: float = 0.0):
        """
        Simulate the Lidar scan at a given position.
        """
        self.ranges: np.array = np.array([])
        self.angles: np.array = np.array([])
        self.point_cloud: np.array = np.array([])

        # Prepare transform matrix
        c, s = np.cos(robot_angle), np.sin(robot_angle)
        to_robot_mat = np.linalg.inv(np.array([
            [c, -s, pos[0]],
            [s, c, pos[1]],
            [0, 0, 1]
        ]))

        # Simulate Lidar scan
        for angle in np.arange(self.step, 360 + self.step, LIDAR_ANGULAR_RESOLUTION + LIDAR_ANGLE_JUMPS - 1):
            angle_rad = np.deg2rad(angle)
            intersection_point = None
            distance = None
            for obstacle in obstacles:
                (obst_intersect, obst_dist) = obstacle.ray_intersect(pos, angle_rad)
                if obst_intersect is None:
                    continue
                if obst_dist > LIDAR_MAX or obst_dist < LIDAR_MIN:
                    continue
                if distance is None or obst_dist < distance:
                    distance = obst_dist
                    intersection_point = obst_intersect

            if intersection_point is not None:
                # Populate ranges and angles
                self.ranges = np.append(self.ranges, distance if distance is not None else LIDAR_MAX)
                self.angles = np.append(self.angles, angle_rad)

                # Populate point cloud
                if self.point_cloud.shape[0] == 0:
                    self.point_cloud = np.array([intersection_point])
                else:
                    self.point_cloud = np.vstack([self.point_cloud, intersection_point])

        self.step = (self.step + 1) % LIDAR_ANGLE_JUMPS
