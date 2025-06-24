import numpy as np
from utils import quaternion_to_euler
from .GenericController import GenericController
from robots.GenericRobot import MAX_ANGULAR_TURN, MAX_LINEAR_SPEED

""" Simple Controller
This controller implements a simple control strategy for a single robot.
It moves towards a target position without considering obstacles.
"""

OBSTACLE_WEIGHT = 500.0
OBSTACLE_RADIUS = 100.0
ANGLE_THRESHOLD = 0.3
TARGET_STOP_RADIUS = 0.2


class SimpleController(GenericController):
    def __init__(self):
        super().__init__()

    def generate_ctrl(self):
        if len(self.robots) != 1:
            raise Exception("Simple Controller works only with one bot.")

        if self.target is None:
            self.ctrl[0] = (0.0, 0.0)
            return

        # Fetch odometry data
        bot = self.robots[0]
        pos = bot.position[:2]
        yaw = quaternion_to_euler(bot.orientation)[2]

        # Check target
        to_target = self.target - pos
        distance = np.linalg.norm(to_target)

        # Stop near the target
        if distance < TARGET_STOP_RADIUS:
            self.ctrl[0] = (0.0, 0.0)
            return

        # Calculate to target control
        target_angle = np.arctan2(to_target[1], to_target[0])
        desired_angle = np.arctan2(np.sin(target_angle - yaw), np.cos(target_angle - yaw))
        cmd_vel = to_target / distance * MAX_LINEAR_SPEED if distance > 0 else np.zeros(2)

        # If angle to target is large, rotate in place
        if abs(desired_angle) > ANGLE_THRESHOLD:
            linear_speed = 0.0
        else:
            linear_speed = np.clip(np.linalg.norm(cmd_vel), 0.0, MAX_LINEAR_SPEED)

        angular_turn = np.clip(desired_angle, -MAX_ANGULAR_TURN, MAX_ANGULAR_TURN)

        # print(f"linear_speed: {linear_speed}, angular_turn: {angular_turn}")
        self.ctrl[0] = (linear_speed, angular_turn)
