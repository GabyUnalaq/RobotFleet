import numpy as np
from utils import quaternion_to_euler
from .GenericController import GenericController
from robots.GenericRobot import GenericRobot, MAX_ANGULAR_TURN, MAX_LINEAR_SPEED

""" Flocking Obstacle Controller

This controller implements a flocking behavior for a group of robots,
with the addition of a obstacle avoidance mechanism.

It does not work properly, I suspect the Lidar is not interpreted as
it should be and also the density of the points contributes to the issue.
"""

SEPARATION_WEIGHT = 5.0
COHESION_WEIGHT = 1.0
ALIGNMENT_WEIGHT = 1.0
TARGET_WEIGHT = 1.5
OBSTACLE_WEIGHT = 0.0

MIN_SEPARATION = 0.8
TARGET_STOP_RADIUS = 0.5
ANGLE_THRESHOLD = np.deg2rad(20)
OBSTACLE_RADIUS = 1.0


class FlockObstController(GenericController):
    def __init__(self):
        super().__init__()

    def generate_ctrl(self):
        # Fetch odometry data
        odom_data = []
        for bot in self.robots:
            odom_data.append({
                "position": bot.odom_sensor.position,
                "orientation": bot.odom_sensor.orientation,
                "linear_velocity": bot.odom_sensor.linear_velocity,
                # "angular_velocity": bot.odom_sensor.angular_velocity,
            })

        positions = np.array([odom["position"][:2] for odom in odom_data])
        cluster_pos = np.mean(positions, axis=0)

        bot: GenericRobot
        for i, bot in enumerate(self.robots):
            pos_i = np.array(odom_data[i]["position"][:2])
            yaw_i = quaternion_to_euler(odom_data[i]["orientation"])[2]
            vel_i = np.array(odom_data[i]["linear_velocity"][:2])

            separation = np.zeros(2)
            cohesion = np.zeros(2)
            alignment = np.zeros(2)
            neighbors = 0

            to_target = np.array([0.0, 0.0])
            if self.target is not None:
                to_target = self.target - pos_i
                if np.linalg.norm(cluster_pos - self.target) < TARGET_STOP_RADIUS:
                    self.ctrl[i] = (0.0, 0.0)
                    continue

            for j, other in enumerate(self.robots):
                if i == j:
                    continue
                pos_j = np.array(odom_data[j]["position"][:2])
                vel_j = np.array(odom_data[j]["linear_velocity"][:2])
                diff = pos_i - pos_j
                dist = np.linalg.norm(diff)

                neighbors += 1
                cohesion += pos_j
                alignment += vel_j

                if 1e-3 < dist < MIN_SEPARATION:
                    repulsion = (diff / dist) * ((MIN_SEPARATION - dist) / dist)
                    separation += repulsion

            if neighbors > 0:
                cohesion = (cohesion / neighbors - pos_i)
                alignment = (alignment / neighbors - vel_i)

            obstacle_avoidance = np.zeros(2)
            lidar_points = bot.lidar_sensor.point_cloud
            heading = np.array([np.cos(yaw_i), np.sin(yaw_i)])
            for point in lidar_points:
                dist = np.linalg.norm(point)
                if 1e-3 < dist < OBSTACLE_RADIUS:
                    direction = point / dist
                    # Only consider points in front (dot > 0)
                    if np.dot(direction, heading) > 0.5:
                        obstacle_avoidance += direction / (dist ** 2)

            flocking = (SEPARATION_WEIGHT * separation +
                        COHESION_WEIGHT * cohesion +
                        ALIGNMENT_WEIGHT * alignment +
                        TARGET_WEIGHT * to_target)

            if np.linalg.norm(obstacle_avoidance) > 0.01:
                # Remove forward component if obstacle ahead
                forward_component = np.dot(flocking, heading) * heading
                flocking -= forward_component  # Cancel forward motion

                # Add obstacle avoidance to steer away
                flocking += OBSTACLE_WEIGHT * obstacle_avoidance

            rot = np.array([[np.cos(-yaw_i), -np.sin(-yaw_i)],
                            [np.sin(-yaw_i),  np.cos(-yaw_i)]])
            cmd_vel = rot @ flocking

            desired_angle = np.arctan2(cmd_vel[1], cmd_vel[0])

            if abs(desired_angle) > ANGLE_THRESHOLD:
                linear_speed = 0.0
            else:
                linear_speed = np.clip(np.linalg.norm(cmd_vel), 0.0, MAX_LINEAR_SPEED)

            angular_turn = np.clip(desired_angle, -MAX_ANGULAR_TURN, MAX_ANGULAR_TURN)

            self.ctrl[i] = (linear_speed, angular_turn)
