import numpy as np
import time
import json
from scipy.spatial.distance import pdist
from utils import quaternion_to_euler
from robots import GenericRobot

""" Generic Controller

This is a base class for all controllers in the PythonRobot framework.
It provides a structure for managing multiple robots, generating control commands,
and recording metrics during the simulation.
"""


class GenericController:
    def __init__(self):
        self.robots: list[GenericRobot] = []
        self.ctrl = []  # linear, angular
        self.target = None
        self._target_reached = None
        self.start_time = time.time()

        self._record_metrics = False
        self.metrics = {
            "timestamp": [],  # timestamps of each step
            "dist_to_target": [],  # distance to target in each step
            "target_reached": [],  # timestamps when targets were reached
            "smoothness": [],  # smoothness of the trajectory
            "effort": [],  # effort of the controller
            "inter-robot_distance": [],  # distance between robots in each step
        }

    def __del__(self):
        """
        Destructor to ensure proper cleanup of the controller.
        """
        self.shutdown()

    def record_metrics(self):
        """
        Enable recording of metrics during the run.
        """
        self._record_metrics = True

    def set_target(self, target: np.ndarray):
        """
        Set the target position for the controller.
        The target should be an array with two elements [x, y].
        """
        self.target = target

    def add_robot(self, bot: GenericRobot):
        """
        Add a robot to the controller.
        """
        self.robots.append(bot)
        self.ctrl.append((0.0, 0.0))

    def step(self):
        """
        Perform a single step of the controller.
        This method generates the control and sets it for each robot.
        """
        self.generate_ctrl()
        self.set_ctrl()
        if self._record_metrics:
            self.update_metrics()

    def generate_ctrl(self):
        """
        The main method to generate control commands for the robots.
        This method should be implemented in subclasses to define specific control logic.
        """
        raise NotImplementedError("generate_ctrl must be implemented")

    def set_ctrl(self):
        """
        Method to set the control commands for each robot.
        """
        bot: GenericRobot
        for bot, ctrl in zip(self.robots, self.ctrl):
            bot.set_cmd_vel(ctrl[0], ctrl[1])

    def get_ctrl(self):
        """
        Getter for the control commands.
        """
        return self.ctrl

    def advance(self, dt: float, obstacles: list):
        """
        Method to advance the simulation by a time step `dt`.
        """
        bot: GenericRobot
        for bot in self.robots:
            if type(bot).__name__ == "DummyTransbot":
                bot_obstacles = obstacles + [temp_bot.obstacle for temp_bot in self.robots if temp_bot != bot]
                bot.advance(dt=dt, obstacles=bot_obstacles)
            elif type(bot).__name__ == "Transbot":
                bot.advance()
    
    def simulate_trajectory(self, bot_id: int, dt: float=0.1, steps: int=10) -> list:
        """
        Method used to simulate the trajectory of a robot based on its current control commands.
        """
        pos = list(self.robots[bot_id].position[:2])
        try:
            yaw = quaternion_to_euler(self.robots[bot_id].orientation)[2]
        except ValueError:
            yaw = 0.0
        linear_speed, angular_turn = self.ctrl[bot_id]
        if abs(linear_speed) < 0.01:
            return []
        traj = []
        for _ in range(steps):
            dx = linear_speed * np.cos(yaw) * dt
            dy = linear_speed * np.sin(yaw) * dt
            dtheta = angular_turn * dt

            pos[0] += dx
            pos[1] += dy
            yaw += dtheta

            traj.append((pos[0], pos[1], yaw))
        return traj

    def target_reached(self):
        """
        Method to be called when the target is reached.
        """
        if self._target_reached is not None and self._target_reached[0] == self.target[0] and self._target_reached[1] == self.target[1]:
            return
        self.metrics["target_reached"].append(time.time() - self.start_time)
        self._target_reached = self.target

    def update_metrics(self):
        """
        This method updates the metrics for the controller.
        It calculates the distance to the target, smoothness, effort, and inter-robot distances.
        """
        if len(self.robots) == 0:
            return

        # Update timestamp
        self.metrics["timestamp"].append(time.time() - self.start_time)

        # Update to target metrics
        cluster_pos = np.mean([robot.position[:2] for robot in self.robots], axis=0)
        dist_to_target = np.linalg.norm(cluster_pos - self.target) if self.target is not None else 0.0
        self.metrics["dist_to_target"].append(dist_to_target)
        if self.target_reached and self.metrics["target_reached"] == -1:
            self.metrics["target_reached"] = self.metrics["timestamp"][-1]

        # Calculate smoothness and effort
        smoothness = np.mean([np.linalg.norm(np.array(self.ctrl[i]) - np.array(self.ctrl[i - 1])) for i in range(1, len(self.ctrl))])
        self.metrics["smoothness"].append(smoothness)
        effort = np.sum([np.linalg.norm(np.array(ctrl)) for ctrl in self.ctrl])
        self.metrics["effort"].append(effort)

        # Update inter-robot distance
        positions = np.array([robot.position[:2] for robot in self.robots])
        if len(positions) > 1:
            dists = pdist(positions)
            self.metrics["inter-robot_distance"].append(np.mean(dists))
        else:
            self.metrics["inter-robot_distance"].append(0.0)

    def save_metrics(self):
        """
        Method used to save the metrics to a JSON file.
        """
        class_name = type(self).__name__
        filename = f"metrics_{class_name}.json"
        with open(filename, 'w', encoding="utf-8") as f:
            json.dump(self.metrics, f, indent=4)

    def shutdown(self):
        """
        This method is called to gracefully clean up the controller
        and all the robots it manages.
        """
        self.set_target(None)
        for i in range(len(self.robots)):
            self.ctrl[i] = (0.0, 0.0)
        self.set_ctrl()

        if self._record_metrics:
            self.save_metrics()
