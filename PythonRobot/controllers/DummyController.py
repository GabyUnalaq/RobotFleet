import numpy as np
from .GenericController import GenericController

""" Dummy Controller

This controller is a placeholder that simply sets a constant control
for each robot.
"""

class DummyController(GenericController):
    def __init__(self):
        super().__init__()

    def generate_ctrl(self):
        for i, bot in enumerate(self.robots):
            # if bot.robot_name == "bot1":
            #     self.ctrl[i] = (0.1, np.deg2rad(45))
            self.ctrl[i] = (0.5, np.deg2rad(45))
