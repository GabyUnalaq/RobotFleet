import rospy
import numpy as np
import threading
from utils import quaternion_to_euler
from dummy_topics import Obstacle
from topics import *
from .GenericRobot import *
from controllers import *

""" Transbot Robot Class
This class represents the Transbot robot, inheriting from GenericRobot.
It initializes various sensors and controllers, manages the robot's state,
and provides methods for controlling the robot's movement and retrieving sensor data.
"""


class Transbot(GenericRobot):
    def __init__(self, env: str = None):
        super().__init__(env)

    def start(self):
        self.imu_sensor = ImuSensor(self.robot_name)
        self.lidar_sensor = LidarSensor(self.robot_name)
        self.odom_sensor = OdometrySensor(self.robot_name)
        self.bot_ctrl = BotCtrl(self.robot_name)
        self.battery = BatteryHandler(self.robot_name)
        self.frames = Frames(self.robot_name)

        self.obstacle = Obstacle(0.0, 0.0, GenericRobot.TRANSBOT_LENGTH, GenericRobot.TRANSBOT_WIDTH)

        self.battery_thread = threading.Thread(target=self._battery_routine)
        self.battery_thread.daemon = True
    
    def _battery_routine(self):
        """
        Routine to handle battery data.
        """
        while not rospy.is_shutdown():
            val = self.battery.check_battery()
            if val != "OK":
                print(f"Battery status: {val}")
            rospy.sleep(5)

    def advance(self):
        """
        Advance the robot's state based on the current linear speed and angular turn.
        """
        pos = self.position[:2]
        if pos.ndim == 2:
            pos = pos[0]
        try:
            yaw = quaternion_to_euler(self.orientation)[2]
        except ValueError:
            yaw = 0.0
        self.obstacle.set_new_pose(pos[0], pos[1], yaw)

    def shutdown(self):
        if self.imu_sensor:
            self.imu_sensor.shutdown()
            self.imu_sensor = None
        if self.bot_ctrl:
            self.bot_ctrl.shutdown()
            self.bot_ctrl = None
        if self.lidar_sensor:
            self.lidar_sensor.shutdown()
            self.lidar_sensor = None
        if self.odom_sensor:
            self.odom_sensor.shutdown()
            self.odom_sensor = None

    def set_cmd_vel(self, linear_speed, angular_turn):
        self.bot_ctrl.set_control(linear_speed, angular_turn)

    def base_link_to_odom(self):
        """
        Get the transformation from base_link to odom.
        """
        return self.frames["odom->base_footprint"].\
            mult(self.frames["base_footprint->base_link"])

    @property
    def lidar_point_cloud(self) -> np.ndarray:
        lidar_points = self.lidar_sensor.point_cloud
        if lidar_points is not None:
            lidar_to_odom_space = self.base_link_to_odom().\
                mult(self.frames["base_link->laser"])
            lidar_to_world = self.odom_to_world.mult(lidar_to_odom_space)
            return lidar_to_world.apply_on_points(lidar_points)
        return np.array([])
