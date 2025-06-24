import numpy as np
from dummy_topics import *
from .GenericRobot import *

""" DummyTransbot
This class simulates a Transbot robot in a dummy environment.
It provides methods to set command velocities, advance the robot's state,
and retrieve sensor data such as IMU, odometry, and LIDAR.
"""

class DummyTransbot(GenericRobot):
    def __init__(self, env, pos, theta, enable_lidar=True):
        super().__init__(env if env is not None else "dummy_bot")

        self.pos = np.array(pos, dtype=float)
        self.prev_pos = self.pos.copy()
        self.theta = theta
        self.prev_theta = theta
        self.linear_speed = 0.0  # Linear velocity (m/s)
        self.angular_turn = 0.0  # Angular velocity (rad/s)
        self.wheel_base = 0.1
        self.enable_lidar = enable_lidar

        # The robot itself is an obstacle to the others, for collision detection and Lidar simulation
        self.obstacle = Obstacle(pos[0], pos[1], GenericRobot.TRANSBOT_LENGTH, GenericRobot.TRANSBOT_WIDTH, theta)

        self.start()

    def __del__(self):
        self.shutdown()

    def start(self):
        self.imu_sensor = ImuSensor(self.robot_name)
        self.odom_sensor = OdometrySensor(self.robot_name)
        self.lidar_sensor = LidarSensor(self.robot_name)
        self.bot_ctrl = BotCtrl(self.robot_name)

    def set_cmd_vel(self, linear_speed, angular_turn):
        self.linear_speed = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, linear_speed))
        self.angular_turn = max(-MAX_ANGULAR_TURN, min(MAX_ANGULAR_TURN, angular_turn))
        self.bot_ctrl.set_control(self.linear_speed, self.angular_turn)

    def advance(self, dt, obstacles):
        dx = self.linear_speed * np.cos(self.theta) * dt
        dy = self.linear_speed * np.sin(self.theta) * dt
        dtheta = self.angular_turn * dt

        self.prev_pos = self.pos.copy()
        self.prev_theta = self.theta

        self.pos[0] += dx
        self.pos[1] += dy
        self.theta += dtheta

        # Update sensors
        self.imu_sensor.update(self, dt)
        self.odom_sensor.update(self, dt)
        self.obstacle.set_new_pose(self.pos[0], self.pos[1], self.theta)
        if self.enable_lidar:
            self.lidar_sensor.simulate_lidar(
                self.pos, obstacles, self.theta
            )

        # Debug
        # print(self.imu_sensor.get_imu_data())
        # print(self.odom_sensor.get_odom_data())

    @property
    def lidar_point_cloud(self):
        return self.lidar_sensor.point_cloud
