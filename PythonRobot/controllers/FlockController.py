import numpy as np
from utils import quaternion_to_euler
from .GenericController import GenericController
from robots.GenericRobot import GenericRobot, MAX_ANGULAR_TURN, MAX_LINEAR_SPEED

""" Flocking Controller

This controller implements a flocking behavior for a group of robots, without any obstacle avoidance.

Works well, in simulation and on the real robot.
"""

SEPARATION_WEIGHT = 1.75
COHESION_WEIGHT = 1.0
ALIGNMENT_WEIGHT = 0.5
TARGET_WEIGHT = 1.5
MIN_SEPARATION = 0.8
TARGET_STOP_RADIUS = 0.2
ANGLE_THRESHOLD = 0.3


class FlockController(GenericController):
    def __init__(self):
        super().__init__()

    def generate_ctrl(self):
        if self.target is None:
            self.ctrl = np.zeros_like(self.ctrl)
            return

        # Fetch odometry data
        odom_data = []
        for bot in self.robots:
            odom_data.append({
                "position": bot.position,
                "orientation": bot.orientation,
                "linear_velocity": bot.odom_sensor.linear_velocity,
                # "angular_velocity": bot.odom_sensor.angular_velocity,
            })

        positions = np.array([odom["position"][:2] for odom in odom_data])
        cluster_pos = np.mean(positions, axis=0)

        if np.linalg.norm(cluster_pos - self.target) < TARGET_STOP_RADIUS:
            self.ctrl = np.zeros_like(self.ctrl)
            self.target_reached()
            return

        bot: GenericRobot
        for i, bot in enumerate(self.robots):
            pos_i = np.array(odom_data[i]["position"][:2])
            yaw_i = quaternion_to_euler(odom_data[i]["orientation"])[2]
            vel_i = np.array(odom_data[i]["linear_velocity"][:2])

            separation = np.zeros(2)
            cohesion = np.zeros(2)
            alignment = np.zeros(2)
            neighbors = 0
            to_target = self.target - pos_i

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

            flocking = (SEPARATION_WEIGHT * separation +
                        COHESION_WEIGHT * cohesion +
                        ALIGNMENT_WEIGHT * alignment +
                        TARGET_WEIGHT * to_target)

            rot = np.array([[np.cos(-yaw_i), -np.sin(-yaw_i)],
                            [np.sin(-yaw_i),  np.cos(-yaw_i)]])
            cmd_vel = rot @ flocking

            desired_angle = np.arctan2(cmd_vel[1], cmd_vel[0])

            # If angle to target is large, rotate in place
            if abs(desired_angle) > ANGLE_THRESHOLD:
                linear_speed = 0.0
            else:
                linear_speed = np.clip(np.linalg.norm(cmd_vel), 0.0, MAX_LINEAR_SPEED)

            angular_turn = np.clip(desired_angle, -MAX_ANGULAR_TURN, MAX_ANGULAR_TURN)

            self.ctrl[i] = (linear_speed, angular_turn)
