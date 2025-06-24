import rospy

try:
    from transbot_msgs.srv import RobotArm, RobotArmRequest, RobotArmResponse
except ImportError:
    print("transbot_msgs not found. Check if the transbot_ws was sourced and linked correctly.")
    exit(1)


# TODO make this work
class ArmState:
    """
    Class to handle the Arm state.

    Attributes:
    """
    JOINT_SERVICE = "/CurrentAngle"

    def __init__(self, env: str = None) -> None:
        """
        Initialize the Arm state class.
        """
        self.base_angle: float = 100.0
        self.elbow_angle: float = 100.0
        self.gripper_angle: float = 100.0
        self.client = None

        if env:
            self.srv_name = f"/{env}{ArmState.JOINT_SERVICE}"
        else:
            self.srv_name = ArmState.JOINT_SERVICE

        self.start()

    def __del__(self):
        """
        Destructor to clean up the Arm state.
        """
        self.shutdown()

    def start(self):
        """
        Start the Arm state.
        """
        if self.client:
            print("ArmState already started.")
            return
        self.client = rospy.ServiceProxy(self.srv_name, RobotArm)

    def shutdown(self):
        """
        Shutdown the Arm state.
        """
        if self.sub:
            self.client.close()
            self.client = None
        print("Arm state shutdown.")

    def __str__(self) -> str:
        """
        Return a string representation of the Arm state.
        """
        data = f"Base: {self.base_angle}, Elbow: {self.elbow_angle}, Gripper: {self.gripper_angle}"
        return str(data)

    def get_arm_state(self) -> list:
        """
        Get the latest Arm state.
        """
        self.client.wait_for_service()
        request = RobotArmRequest()
        request.apply = ""
        try:
            response = self.client.call(request)
            if isinstance(response, RobotArmResponse):
                for joint in response.RobotArm.joint:
                    if joint.id == 7:
                        self.base_angle = joint.angle
                    elif joint.id == 8:
                        self.elbow_angle = joint.angle
                    elif joint.id == 9:
                        self.gripper_angle = joint.angle
        except Exception as e:
            rospy.loginfo("ArmState error: %s", e)
            return None
        return [self.base_angle, self.elbow_angle, self.gripper_angle]


if __name__ == "__main__":
    import signal

    def shutdown_handler(signum, frame):
        rospy.signal_shutdown("Shutting down ArmCtrl.")
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.init_node('dev_arm_state_reader', anonymous=True)
    arm_state = ArmState()
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        data = arm_state.get_arm_state()
        if data:
            print(data)
        rate.sleep()

    print("Exiting...")
