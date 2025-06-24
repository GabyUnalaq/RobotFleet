import rospy

""" Dummy Bot Control
This module simulates a bot control topic that allows setting linear and angular speeds.
It is used for the simulator to control the robot's movement.
"""


class BotCtrl:
    DEFAULT_LINEAR_LIMIT = 0.45
    DEFAULT_ANGULAR_LIMIT = 2.0

    def __init__(self, env: str = None):
        """
        Initialize the Bot Control topic.
        """
        self.linear_limit = BotCtrl.DEFAULT_LINEAR_LIMIT
        self.angular_limit = BotCtrl.DEFAULT_ANGULAR_LIMIT
        self.linear_speed = 0.0
        self.angular_turn = 0.0

    def set_control(self, linear_speed: float, angular_turn: float):
        """
        Set the linear and angular limits.
        """
        linear_speed = max(-self.linear_limit, min(self.linear_limit, linear_speed))
        angular_turn = max(-self.angular_limit, min(self.angular_limit, angular_turn))

        self.linear_speed = linear_speed
        self.angular_turn = angular_turn
