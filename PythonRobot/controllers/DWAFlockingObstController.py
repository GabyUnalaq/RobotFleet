import numpy as np
from utils import quaternion_to_euler
from .GenericController import GenericController
from robots.GenericRobot import MAX_ANGULAR_TURN, MAX_LINEAR_SPEED

""" DWA Flocking Obstacle Controller

This controller implement a Dynamic Window Approach (DWA) and takes into
consideration the flocking control strategy for a group of robots.
It takes into consideration the Lidar point clouds to avoid obstacles.
"""

SEPARATION_WEIGHT = 3.0
COHESION_WEIGHT = 1.0
ALIGNMENT_WEIGHT = 1.5
TARGET_WEIGHT = 2.0
MIN_SEPARATION = 0.8
TARGET_STOP_RADIUS = 0.2
ANGLE_THRESHOLD = 0.3


class DWAFlockingObstController(GenericController):
    FUTURE_STEPS = 30
    V_SAMPLES = 10
    W_SAMPLES = 17
    def __init__(self):
        super().__init__()

    def generate_ctrl(self):
        if self.target is None:
            self.ctrl = np.zeros_like(self.ctrl)
            return

        odom_data = []
        for bot in self.robots:
            odom_data.append({
                "position": bot.position,
                "orientation": bot.orientation,
                "linear_velocity": bot.odom_sensor.linear_velocity,
            })

        positions = np.array([odom["position"][:2] for odom in odom_data])
        cluster_pos = np.mean(positions, axis=0)

        dt = 0.1
        for i, bot in enumerate(self.robots):
            pos_i = np.array(odom_data[i]["position"][:2])
            yaw_i = quaternion_to_euler(odom_data[i]["orientation"])[2]
            vel_i = np.array(odom_data[i]["linear_velocity"][:2])

            separation = np.zeros(2)
            cohesion = np.zeros(2)
            alignment = np.zeros(2)
            neighbors = 0

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

            flocking_vec = (SEPARATION_WEIGHT * separation +
                            COHESION_WEIGHT * cohesion +
                            ALIGNMENT_WEIGHT * alignment +
                            TARGET_WEIGHT * to_target)

            flocking_vec_mag = np.linalg.norm(flocking_vec)
            if flocking_vec_mag < 1e-3:
                self.ctrl[i] = (0.0, 0.0)
                continue

            rot = np.array([[np.cos(-yaw_i), -np.sin(-yaw_i)],
                            [np.sin(-yaw_i),  np.cos(-yaw_i)]])
            flocking_vec_robot = rot @ flocking_vec

            desired_angle = np.arctan2(flocking_vec_robot[1], flocking_vec_robot[0])

            if abs(desired_angle) > ANGLE_THRESHOLD:
                self.ctrl[i] = (0.0, np.clip(desired_angle, -MAX_ANGULAR_TURN, MAX_ANGULAR_TURN))
                continue

            v_samples = np.linspace(0.0, MAX_LINEAR_SPEED, DWAFlockingObstController.V_SAMPLES)
            w_samples = np.random.normal(loc=0.0, scale=MAX_ANGULAR_TURN / 2, size=DWAFlockingObstController.W_SAMPLES)

            best_score = -np.inf
            best_ctrl = (0.0, 0.0)

            for v in v_samples:
                for w in w_samples:
                    x, y, theta = pos_i[0], pos_i[1], yaw_i
                    for _ in range(DWAFlockingObstController.FUTURE_STEPS):
                        x += v * np.cos(theta) * dt
                        y += v * np.sin(theta) * dt
                        theta += w * dt
                    end_vec = np.array([x, y]) - pos_i
                    if np.linalg.norm(end_vec) < 1e-3:
                        continue
                    end_vec /= np.linalg.norm(end_vec)
                    desired_vec = flocking_vec / (np.linalg.norm(flocking_vec) + 1e-6)
                    alignment_score = np.dot(end_vec, desired_vec)
                    velocity_score = v
                    score = alignment_score + 0.1 * velocity_score
                    if score > best_score:
                        best_score = score
                        best_ctrl = (v, w)

            self.ctrl[i] = best_ctrl
