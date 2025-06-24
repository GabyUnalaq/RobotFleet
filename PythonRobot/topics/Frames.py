import rospy
from tf2_msgs.msg import TFMessage
import numpy as np
from threading import Lock, Event

from utils import Transform
from .GenericTopic import GenericSubscriber

""" Frames module
This module provides a Frames class that subscribes to the TF messages
and processes the transforms between different frames in a robot environment.

It handles the static and dynamic transforms, allowing for easy access to
the robot's frame transformations.
"""


class Frames(GenericSubscriber):
    """
    Class to handle the transform frames.

    Attributes:
        transforms: Dictionary of transforms.
    """
    _tf_static_data = None
    _tf_static_transforms = None
    _tf_static_lock = Lock()
    TOPIC_NAME = "/tf"
    STATIC_TF_TOPIC = "/tf_static"
    STATIC_TIMEOUT = 5.0  # seconds

    def __init__(self, env: str = None) -> None:
        """
        Initialize the Frames class.
        """
        self.transforms = {
            "odom->base_footprint": None,
            "base_footprint->base_link": None,
            "base_link->imu_link": None,
            "base_link->laser": None,

            # "base_link->camera_link1": None,
            # "camera_link1->camera_link2": None,

            # "base_link->arm": None,
            # "arm->arm_link1": None,
            # "arm_link1->arm_link2": None,
            # "arm_link2->right_link1": None,
            # "right_link1->right_link2": None,
            # "arm_link2->left_link1": None,
            # "left_link1->left_link2": None,
            # "arm_link2->left_link3": None,
            # "arm_link2->right_link3": None,
        }
        super().__init__(TFMessage, None)  # env is not used
        self.env = env
        self.hardcode_static_tf()  # self.handle_static_tf()
        rospy.sleep(0.5)  # Allow time for the TF to be processed

    def hardcode_static_tf(self):
        """
        Hardcode the static transforms for testing purposes.
        """
        if "base_link->arm" in self.transforms:
            self.transforms["base_link->arm"] = Transform(
                translation=np.array([0.12925, 0.0, -0.014083]),
                rotation=np.array([-0.27059864998286565, 0.27059864998154065, -0.6532812339457786, 0.6532812339489777])
            )
        if "base_footprint->base_link" in self.transforms:
            self.transforms["base_footprint->base_link"] = Transform(
                translation=np.array([0.0, 0.0, 0.08]),
                rotation=np.array([0.0, 0.0, 0.0, 1.0])
            )

    def handle_static_tf(self) -> None:
        """
        Method to get the static transforms from the /tf_static topic.
        If the transforms were got already, they are returned directly.
        Because they are static, they cannot be fetched again.

        Sometimes, the fetching fails, an easy fix is to hardcode the values.
        """
        with Frames._tf_static_lock:
            if Frames._tf_static_data is None:
                expected_transforms = [
                    "base_link->arm",
                    "base_footprint->base_link",
                ]
                got_data = Event()
                def tf_static_callback(data):
                    Frames._tf_static_data = data
                    got_data.set()

                # Subscribe to tf static, receive data and unsubscribe
                static_sub = rospy.Subscriber(Frames.STATIC_TF_TOPIC, TFMessage, tf_static_callback, queue_size=1)
                timeout = rospy.Time.now() + rospy.Duration(Frames.STATIC_TIMEOUT)
                rate = rospy.Rate(10)  # 10 Hz
                while not got_data.is_set():
                    if rospy.Time.now() > timeout:
                        static_sub.unregister()
                        raise rospy.ROSException("Timeout waiting for /tf_static")
                    rate.sleep()
                static_sub.unregister()

                # Process data
                Frames._tf_static_transforms = {}
                for elem in Frames._tf_static_data.transforms:
                    name = f"{elem.header.frame_id}->{elem.child_frame_id}"
                    # print(f"Name: {name}, env: {self.env}")
                    if self.env is not None:
                        name = name.replace(f"{self.env}/", "")
                    Frames._tf_static_transforms[name] = Frames._get_transform(elem)
                
                # Check if all expected transforms are present
                for expected in expected_transforms:
                    if expected not in Frames._tf_static_transforms:
                        raise rospy.ROSException(f"Expected static transform '{expected}' not found.")
        if Frames._tf_static_transforms is None:
            raise rospy.ROSException("No static TF found.")
    
        # Extract static transforms
        for key in Frames._tf_static_transforms.keys():
            if key in self.transforms:
                with self._lock:
                    self.transforms[key] = Frames._tf_static_transforms[key]

    def __getitem__(self, key: str) -> np.array:
        """
        Get the transform by key.
        """
        with self._lock:
            return self.transforms[key]

    def _topic_callback(self, data):
        """
        Callback function for the TF data.
        """
        super()._topic_callback(data)
        self._process_tf_data(self._data)

    @staticmethod
    def _get_transform(tr) -> Transform:
        return Transform(
            translation=np.array([tr.transform.translation.x,
                                    tr.transform.translation.y,
                                    tr.transform.translation.z]),
            rotation=np.array([tr.transform.rotation.x,
                                tr.transform.rotation.y,
                                tr.transform.rotation.z,
                                tr.transform.rotation.w])
        )

    def _process_tf_data(self, data) -> dict:
        """
        Process the TF data into a dictionary of transforms.
        """
        for elem in data.transforms:
            name = f"{elem.header.frame_id}->{elem.child_frame_id}"
            if self.env is not None:
                name = name.replace(f"{self.env}/", "")
            if name not in self.transforms.keys():
                continue
            tr = Frames._get_transform(elem)
            with self._lock:
                self.transforms[name] = tr

    def keys(self) -> list:
        """
        Return the keys of the transforms.
        """
        with self._lock:
            return list(self.transforms.keys())


if __name__ == "__main__":
    import signal

    def shutdown_handler(signum, frame):
        rospy.signal_shutdown("Shutting down Frames.")
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.init_node('dev_tf_reader', anonymous=True)
    frames = Frames("bot01")
    rate = rospy.Rate(1)  # 1 Hz

    for key in frames.keys():
        try:
            print(f"{key}:\n{frames[key]}")
        except KeyError:
            continue

    print("Done.")