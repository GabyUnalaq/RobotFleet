import rospy
from geometry_msgs.msg import Twist

from .GenericTopic import GenericPublisher

""" Bot Control
This module provides a BotCtrl class that is a publisher for the control of a robot.
It allows setting linear and angular speeds, with limits defined by parameters.
"""


class BotCtrl(GenericPublisher):
    """
    Class to control the movement of the robot.

    Attributes:
        linear_limit: Maximum linear speed limit.
        angular_limit: Maximum angular speed limit.
        linear_speed: Current linear speed.
        angular_turn: Current angular speed.
    """
    TOPIC_NAME = "/cmd_vel"
    DEFAULT_LINEAR_LIMIT = 0.45
    DEFAULT_ANGULAR_LIMIT = 2.0

    def __init__(self, env: str = None):
        """
        Initialize the Bot Control topic.
        """
        super().__init__(Twist, env)
        self.linear_limit = rospy.get_param('~linear_limit', BotCtrl.DEFAULT_LINEAR_LIMIT)
        self.angular_limit = rospy.get_param('~angular_limit', BotCtrl.DEFAULT_ANGULAR_LIMIT)

    def set_control(self, linear_speed: float, angular_turn: float):
        """
        Set the linear and angular limits.
        """
        linear_speed = max(-self.linear_limit, min(self.linear_limit, linear_speed))
        angular_turn = max(-self.angular_limit, min(self.angular_limit, angular_turn))

        with self._lock:
            self._data = Twist()
            self._data.linear.x = linear_speed
            self._data.angular.z = angular_turn
        self.publish()
    
    @property
    def linear_speed(self) -> float:
        """
        Get the current linear speed.
        """
        with self._lock:
            return self._data.linear.x

    @property
    def angular_turn(self) -> float:
        """
        Get the current angular speed.
        """
        with self._lock:
            return self._data.angular.z


def get_key(timeout=0.1):
    import sys
    import tty
    import termios
    import select

    """Read one character from stdin with a timeout (non-blocking)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None


if __name__ == "__main__":
    """
    Example usage of the BotCtrl class.
    This script was adapted from transbot_ws/src/transbot_ctrl/scripts/transbot_keyboard.py.
    """
    import signal

    msg = """
Test BotCtrl:
---------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C or Esc to quit
"""

    def shutdown_handler(signum, frame):
        rospy.signal_shutdown("Shutting down BotCtrl.")
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.init_node('dev_bot_controller', anonymous=True)
    bot_ctrl = BotCtrl("bot02")

    moveBindings = {
        'u': (1, 1), 'U': (1, 1), 'i': (1, 0), 'I': (1, 0), 'o': (1, -1), 'O': (1, -1),
        'j': (0, 1), 'J': (0, 1), 'l': (0, -1), 'L': (0, -1),
        'm': (-1, -1), 'M': (-1, -1), ',': (-1, 0), '.': (-1, 1), ' ': (0, 0),
    } # k is not necessary, as it is handled by the stop command

    speedBindings = {
        'q': (1.1, 1.1), 'Q': (1.1, 1.1), 'z': (.9, .9), 'Z': (.9, .9),
        'w': (1.1, 1), 'W': (1.1, 1), 'x': (.9, 1), 'X': (.9, 1),
        'e': (1, 1.1), 'E': (1, 1.1), 'c': (1, .9), 'C': (1, .9),
    }

    rate = rospy.Rate(20)  # 20 Hz
    print(msg)

    def vels(speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    (speed, turn) = (0.2, 1.0)
    (x, th) = (0, 0)
    status = 0
    count = 0
    print(vels(speed, turn))
    while not rospy.is_shutdown():
        key = get_key()
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            th = moveBindings[key][1]
            count = 0
        elif key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]
            count = 0
            print(vels(speed, turn))
        elif key == '\x1b':
            shutdown_handler(None, None)
        else:
            count = count + 1
            if count > 4: (x, th) = (0, 0)
        bot_ctrl.set_control(speed * x, turn * th)
    print("Done.")
