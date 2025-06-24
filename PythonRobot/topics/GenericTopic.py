import rospy
from typing import Type
from threading import Lock

""" GenericTopic module
This class provides a base implementation for ROS topics, including
GenericSubscriber and GenericPublisher. It allows for easy creation of topics
with basic functionality such as data retrieval, publishing, and subscription.
"""

__all__ = [
    "GenericSubscriber",
    "GenericPublisher",
]


class _GenericTopic:
    """
    Class that implements the base functionality for a ROS topic.
    """
    TOPIC_NAME = "/placeholder"

    def __init__(self, data_type: Type, env: str = None) -> None:
        """
        Initialize the GenericTopic class.
        """
        self._data_type = data_type
        self._data = data_type()
        self._last_data_ts = None
        self._lock: Lock = Lock()
        self.sub = None
        self.pub = None

        if env:
            self.topic_name = f"/{env}{self.TOPIC_NAME}"
        else:
            self.topic_name = self.TOPIC_NAME

    def __del__(self):
        """
        Destructor to clean up the topic.
        """
        self.shutdown()

    def __str__(self) -> str:
        """
        Return a string representation of the data.
        """
        raise NotImplementedError("__str__() must be implemented in the subclass.")

    def start(self):
        """
        Start the subscriber / publisher for the topic.
        """
        raise NotImplementedError("start() must be implemented in the subclass.")

    def shutdown(self):
        """
        Shutdown the topic.
        """
        if self.sub:
            self.sub.unregister()
            self.sub = None
        if self.pub:
            self.pub.unregister()
            self.pub = None

    def is_data_stale(self, threshold: float = 0.5) -> bool:
        """
        Check if the data is stale based on the given threshold in seconds.
        """
        with self._lock:
            if self._last_data_ts is None:
                return True
            return (rospy.Time.now() - self._last_data_ts).to_sec() > threshold

    def get_data(self):
        """
        Get the latest topic data.
        """
        with self._lock:
            return self._data

    def advance(self, dt: float):
        """
        Method used only for dummy topics.
        """
        raise NotImplementedError("advance() must be implemented in the subclass.")


class GenericSubscriber(_GenericTopic):
    """
    Class that implements the base functionality for a ROS subscriber topic.
    """
    def __init__(self, data_type: Type, env: str = None) -> None:
        """
        Initialize the GenericSubscriber class.
        """
        super().__init__(data_type, env)
        self.start()

    def start(self):
        """
        Start the subscriber for the topic.
        """
        if self.sub:
            print(f"{self.__class__.__name__} already started.")
            return
        self.sub = rospy.Subscriber(
            self.topic_name,
            self._data_type,
            self._topic_callback)

    def _topic_callback(self, data):
        """
        Callback function to handle topic data.
        """
        with self._lock:
            self._data = data
            self._last_data_ts = rospy.Time.now()


class GenericPublisher(_GenericTopic):
    """
    Class that implements the base functionality for a ROS publisher topic.
    """
    def __init__(self, data_type: Type, env: str = None) -> None:
        """
        Initialize the GenericPublisher class.
        """
        super().__init__(data_type, env)
        self.start()

    def start(self):
        """
        Start the publisher for the topic.
        """
        if self.pub:
            print(f"{self.__class__.__name__} already started.")
            return
        self.pub = rospy.Publisher(
            self.topic_name,
            self._data_type,
            queue_size=1)

    def set_control(self, **kwargs):
        """
        Set control parameters for the topic.
        """
        raise NotImplementedError("set_control() must be implemented in the subclass.")

    def publish(self):
        """
        Publish data to the topic.
        """
        with self._lock:
            self.pub.publish(self._data)


if __name__ == "__main__":
    raise RuntimeError(
        "This module is not meant to be run directly.")
