import rospy
try:
    from transbot_msgs.msg import Battery
except ImportError:
    print("transbot_msgs not found. Check if the transbot_ws was sourced and linked correctly.")
    exit(1)
from .GenericTopic import GenericSubscriber

""" Battery Handler
This module provides a BatteryHandler class that subscribes to the battery voltage topic
and provides methods to check the battery status based on voltage levels.
"""


class BatteryHandler(GenericSubscriber):
    MAX_VOLTAGE = 11.7
    MIN_VOLTAGE = 9.6
    TOPIC_NAME = "/voltage"

    def __init__(self, env: str = None) -> None:
        """
        Initialize the BatteryHandler class.

        Attributes:
            voltage (float): Battery voltage.
        """
        super().__init__(Battery, env)

    def __str__(self) -> str:
        """
        Return a string representation of the Battery data.
        """
        with self._lock:
            if self._data is None:
                return "No Battery data available."
        return f"Battery Data:\n" \
               f"   Voltage: {self._data.Voltage} V\n" \
               f"   Status: {self.status}\n"

    @property
    def voltage(self) -> float:
        """
        Get the battery voltage.

        Returns:
            float: Battery voltage.
        """
        with self._lock:
            return self._data.Voltage

    @property
    def status(self) -> str:
        """
        Check the battery status.

        Returns:
            str: "OVERCHARGED", "LOW", or "OK" based on the battery voltage.
        """
        voltage = self.voltage
        if voltage > BatteryHandler.MAX_VOLTAGE:
            return "OVERCHARGED"
        elif voltage < BatteryHandler.MIN_VOLTAGE:
            return "LOW"
        else:
            return "OK"


if __name__ == "__main__":
    import time

    rospy.init_node("dev_battery_handler", anonymous=True)
    battery_handler = BatteryHandler("bot01")
    time.sleep(1)  # Allow time for the subscriber to start and receive data
    print(battery_handler)
