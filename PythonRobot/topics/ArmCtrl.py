import rospy
from threading import Lock

try:
    from transbot_msgs.msg import Arm, Joint
except ImportError:
    print("transbot_msgs not found. Check if the transbot_ws was sourced and linked correctly.")
    exit(1)


# TODO make this work
class ArmCtrl:
    """
    Class to control the arm of the robot.

    Attributes:
        run_time: Joint movement speed in milliseconds.
        base_angle: Angle of the base joint.
        elbow_angle: Angle of the elbow joint.
        gripper_angle: Angle of the gripper joint.
    """
    ARM_COMMAND_TOPIC = "/TargetAngle"

    def __init__(self, env: str = None):
        self.run_time: int = 500
        self.base_angle: float = 100.0
        self.elbow_angle: float = 100.0
        self.gripper_angle: float = 100.0
        self._lock: Lock = Lock()
        self.pub = None

        if env:
            self.topic_name = f"/{env}{ArmCtrl.ARM_COMMAND_TOPIC}"
        else:
            self.topic_name = ArmCtrl.ARM_COMMAND_TOPIC
        self.start()

    def __del__(self):
        """
        Destructor to clean up the Arm Controller.
        """
        self.shutdown()
    
    def start(self):
        """
        Start the Arm Controller.
        """
        if self.pub:
            print("Arm Controller already started.")
            return
        self.pub = rospy.Publisher(self.topic_name, Arm, queue_size=50)

    def shutdown(self):
        """
        Shutdown the Arm Controller.
        """
        if self.pub:
            self.pub.unregister()
            self.pub = None
        print("Arm Controller shutdown.")

    def set_run_time(self, run_time: int):
        self.run_time = run_time

    def get_arm_command(self) -> Arm:
        arm = Arm()
        joints = []
        with self._lock:
            joints.append(Joint(id=7, run_time=self.run_time, angle=self.base_angle))
            joints.append(Joint(id=8, run_time=self.run_time, angle=self.elbow_angle))
            joints.append(Joint(id=9, run_time=self.run_time, angle=self.gripper_angle))
        arm.joint = joints
        return arm

    def set_arm_command(self, arm: Arm):
        with self._lock:
            self.base_angle = arm.joint[0].angle
            self.elbow_angle = arm.joint[1].angle
            self.gripper_angle = arm.joint[2].angle

    def set_joints(self, base_angle: float, elbow_angle: float, gripper_angle: float):
        base_angle = min(255, max(0, base_angle))
        elbow_angle = min(270, max(30, elbow_angle))
        gripper_angle = min(180, max(60, gripper_angle))

        with self._lock:
            self.base_angle = base_angle
            self.elbow_angle = elbow_angle
            self.gripper_angle = gripper_angle

    def publish(self):
        arm = self.get_arm_command()
        self.pub.publish(arm)


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


def handle_key(key, angles):
    RATE = 1.0
    if key == 'q':
        angles["base"] = min(255, angles["base"] + RATE)
    elif key == 'a':
        angles["base"] = max(0, angles["base"] - RATE)
    elif key == 'w':
        angles["elbow"] = min(270, angles["elbow"] + RATE)
    elif key == 's':
        angles["elbow"] = max(30, angles["elbow"] - RATE)
    elif key == 'e':
        angles["gripper"] = min(180, angles["gripper"] + RATE)
    elif key == 'r':
        angles["gripper"] = max(60, angles["gripper"] - RATE)
    elif key == '\x1b':
        return None
    return angles


if __name__ == "__main__":
    import signal

    def shutdown_handler(signum, frame):
        rospy.signal_shutdown("Shutting down ArmCtrl.")
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.init_node('dev_arm_controller', anonymous=True)
    arm_ctrl = ArmCtrl()

    # Initialize arm joint data
    angles = {
        "base": 100.0,
        "elbow": 100.0,
        "gripper": 100.0,
    }

    rate = rospy.Rate(20)  # 20 Hz
    print("ArmController started. Press ESC or Ctrl+C to exit.")
    print("Controls: q/a (base), w/s (elbow), e/r (gripper)")

    while not rospy.is_shutdown():
        key = get_key()
        if key:
            new_angles = handle_key(key, angles)
            if new_angles is None:
                break
            angles = new_angles
            arm_ctrl.set_joints(angles["base"], angles["elbow"], angles["gripper"])
            arm_ctrl.publish()
            rate.sleep()
    print("Exiting...")
