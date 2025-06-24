import numpy as np

""" Obstacle Class
This class represents a rectangular obstacle in a 2D space.
It can be rotated and provides methods to get its corners, edges,
and check for ray intersections for Lidar simulation.
"""


class Obstacle:
    def __init__(self, x: float, y: float, w: float, h: float = None, angle: float = 0.0):
        """
        Initialize a rectangle obstacle centered at (cx, cy) with width and height, rotated by angle (in radians).

        Parameters:
            x, y (float): Top-left corner of the obstacle.
            cx, cy (float): Center position of the obstacle.
            w (float): Width of the obstacle.
            h (float, optional): Height. If not specified, defaults to width (i.e., square).
            angle (float): Rotation angle in radians (default = 0).
        """
        self.x = x
        self.y = y
        self.width = w
        self.height = h if h is not None else w
        self.cx = self.x + self.width / 2.0
        self.cy = self.y + self.height / 2.0
        self.angle = angle

    def set_new_pose(self, cx: float, cy: float, angle: float = None):
        self.cx = cx
        self.cy = cy
        if angle is not None:
            self.angle = angle

    def get_corners(self):
        """
        Get the four corners of the rotated rectangle as (x, y) pairs in world coordinates.
        """
        dx = self.width / 2
        dy = self.height / 2

        # Define rectangle in local space (relative to center)
        local_corners = np.array([
            [-dx, -dy],
            [+dx, -dy],
            [+dx, +dy],
            [-dx, +dy]
        ])

        # Rotation matrix
        c, s = np.cos(self.angle), np.sin(self.angle)
        R = np.array([[c, -s], [s, c]])

        # Rotate and translate to world space
        world_corners = local_corners @ R.T + np.array([self.cx, self.cy])
        return world_corners

    def get_edges(self):
        """
        Return list of edges as line segments (p1, p2), where each is a NumPy array [x, y]
        """
        corners = self.get_corners()
        return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]

    def ray_intersect(self, origin: np.array, angle: float):
        """
        Find the intersection point (if any) between a ray and the obstacle.

        Parameters:
            origin (np.array): Ray start point [x, y]
            angle (float): Ray direction angle in radians

        Returns:
            tuple: (intersection_point (np.array), distance) or (None, None)
        """
        direction = np.array([np.cos(angle), np.sin(angle)])
        closest_intersection = None
        calculated_distance = None
        min_dist = np.inf
        edges = self.get_edges()

        # Return if the obstacle is behind the ray - does not work in some cases: long obstacles
        # to_obstacle = np.array([self.cx, self.cy]) - origin
        # if np.dot(to_obstacle, direction) < 0:
        #     return None, None

        # Take the 2 edges the origin
        edge_midpoints = [(i, 0.5 * (p1 + p2)) for i, (p1, p2) in enumerate(edges)]
        edge_midpoints.sort(key=lambda item: np.linalg.norm(item[1] - origin))
        candidate_indices = [idx for idx, _ in edge_midpoints[:2]]

        for i in candidate_indices:
            p1, p2 = edges[i]
            edge = p2 - p1
            edge_normal = np.array([-edge[1], edge[0]])  # Perpendicular vector

            denom = np.dot(direction, edge_normal)
            if np.abs(denom) < 1e-8:
                continue  # Parallel

            t = np.dot(p1 - origin, edge_normal) / denom
            intersection_point = origin + t * direction

            edge_length_sq = np.dot(edge, edge)
            proj = np.dot(intersection_point - p1, edge)

            if 0 <= proj <= edge_length_sq and t >= 0:
                dist = np.linalg.norm(intersection_point - origin)
                if dist < min_dist:
                    min_dist = dist
                    closest_intersection = intersection_point
                    calculated_distance = dist

        return closest_intersection, calculated_distance
