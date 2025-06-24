import rospy
import numpy as np
from robots.Transbot import Transbot as Robot
from controllers import *
from visualiser import PygameViz

""" Main script for running the PythonRobot simulation.
It uses 3 Transabot robots, places in a triangular formation.
The controller can be adapted to any controller class.
The target is hardcoded but can be modified by the visualiser.
"""


# Create ros node
rospy.init_node(f'main', anonymous=False)

# Constants
FPS = 30
ROBOTS = [
    Robot(env="bot01").initial_pose(x=0.0, y=0.5, theta=0.0),
    Robot(env="bot02").initial_pose(x=0.5, y=0.0, theta=0.0),
    Robot(env="bot04").initial_pose(x=0.0, y=-0.5, theta=0.0),
]
CONTROLLER = FlockController
VIZ = PygameViz(900, 700, FPS)


if __name__ == "__main__":
    ctrl = CONTROLLER()
    # ctrl.record_metrics()
    ctrl.set_target(np.array([6.5, -1.0]))
    for bot in ROBOTS:
        ctrl.add_robot(bot)

    viz = VIZ
    viz.set_zoom(0.8)
    viz.set_ctrl(ctrl)

    viz.start()
    while viz.running:
        viz.step()
    print("Done.")
