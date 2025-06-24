import numpy as np
from utils import quaternion_to_euler
from .GenericController import GenericController
from robots.GenericRobot import MAX_ANGULAR_TURN, MAX_LINEAR_SPEED

""" DWA Obstacle Controller

This controller implements a Dynamic Window Approach (DWA) for a single robot.
It calculates the best linear and angular velocities to reach a target,
while avoiding obstacles using a simple distance-based approach.
"""

ANGLE_THRESHOLD = 0.5
TARGET_STOP_RADIUS = 0.2


class DWAObstController(GenericController):
    FUTURE_STEPS = 30
    V_SAMPLES = 10
    W_SAMPLES = 17
    GAUSSIAN_W = 0.5

    def __init__(self):
        super().__init__()
        self.last_ctrl = (0.0, 0.0)  # Store last control for smoothing

    def generate_ctrl(self):
        if len(self.robots) != 1:
            raise Exception("DWA Controller works only with one bot.")

        self.last_ctrl = self.ctrl[0] if self.ctrl else (0.0, 0.0)

        if self.target is None:
            self.ctrl[0] = (0.0, 0.0)
            return

        bot = self.robots[0]
        pos = bot.position[:2]
        yaw = quaternion_to_euler(bot.orientation)[2]

        vec_to_target = self.target - pos
        dist = np.linalg.norm(vec_to_target)
        angle_to_target = np.arctan2(vec_to_target[1], vec_to_target[0])
        angle_diff = np.arctan2(np.sin(angle_to_target - yaw), np.cos(angle_to_target - yaw))

        if dist < TARGET_STOP_RADIUS:
            self.ctrl[0] = (0.0, 0.0)
            return

        if abs(angle_diff) > ANGLE_THRESHOLD:
            angular = np.clip(angle_diff, -MAX_ANGULAR_TURN, MAX_ANGULAR_TURN)
            # Smooth angular velocity
            angular = 0.7 * self.last_ctrl[1] + 0.3 * angular
            self.ctrl[0] = (0.0, angular)
            return

        dt = 0.1
        v_samples = np.linspace(0.0, MAX_LINEAR_SPEED, DWAObstController.V_SAMPLES)
        w_samples = np.linspace(-MAX_ANGULAR_TURN, MAX_ANGULAR_TURN, DWAObstController.W_SAMPLES)

        # Bias samples towards previous control for stability
        prev_v, prev_w = self.last_ctrl
        v_samples = np.clip(prev_v + np.linspace(-0.2, 0.2, DWAObstController.V_SAMPLES), 0.0, MAX_LINEAR_SPEED)
        w_samples = np.clip(prev_w + np.linspace(-0.3, 0.3, DWAObstController.W_SAMPLES), -MAX_ANGULAR_TURN, MAX_ANGULAR_TURN)
        point_cloud = bot.lidar_point_cloud

        best_score = -np.inf
        best_ctrl = (0.0, 0.0)

        for v in v_samples:
            for w in w_samples:
                x, y, theta = pos[0], pos[1], yaw
                future_positions = []
                for step_idx in range(DWAObstController.FUTURE_STEPS):
                    x += v * np.cos(theta) * dt
                    y += v * np.sin(theta) * dt
                    theta += w * dt
                    if step_idx % 5 == 0:
                        future_positions.append([x, y])
                future_positions = np.array(future_positions)
                dist_to_target = np.linalg.norm(self.target - np.array([x, y]))
                heading_score = -dist_to_target
                velocity_score = v

                # Check for obstacles in the future path
                for pos in future_positions:
                    for point in point_cloud:
                        if np.linalg.norm(pos - point[:2]) < 0.5:
                            heading_score -= 0.1

                # Penalize deviation from previous control for smoothness
                smoothness_penalty = -0.5 * ((v - prev_v) ** 2 + (w - prev_w) ** 2)
                score = heading_score + 0.1 * velocity_score + smoothness_penalty
                if score > best_score:
                    best_score = score
                    best_ctrl = (v, w)

        # Smooth output control to avoid abrupt changes
        smoothed_v = 0.7 * self.last_ctrl[0] + 0.3 * best_ctrl[0]
        smoothed_w = 0.7 * self.last_ctrl[1] + 0.3 * best_ctrl[1]
        self.ctrl[0] = (smoothed_v, smoothed_w)
