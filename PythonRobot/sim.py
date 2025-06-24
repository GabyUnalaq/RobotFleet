import numpy as np

from dummy_topics import Obstacle
from robots.DummyTransbot import DummyTransbot as Robot
from controllers import *
from visualiser import PygameViz

""" Main script for running the PythonRobot simulation.
It uses 7 DummyTransabot robots, placed in a random formation, with
different orientations. Ot contains 5 obstacles, one on each side
and one in the middle of the world.
"""

# Constants
FPS = 10
BOX_WIDTH, BOX_HEIGHT = 9.0, 7.0
OBSTACLE_EDGE_SIZE = 0.1
OBSTACLES = [
    Obstacle(x=0, y=0, w=OBSTACLE_EDGE_SIZE, h=BOX_HEIGHT),  # Left
    Obstacle(x=0, y=0, w=BOX_WIDTH, h=OBSTACLE_EDGE_SIZE),  # Top
    Obstacle(x=0, y=BOX_HEIGHT-OBSTACLE_EDGE_SIZE, w=BOX_WIDTH, h=OBSTACLE_EDGE_SIZE),  # Bottom
    Obstacle(x=BOX_WIDTH-OBSTACLE_EDGE_SIZE, y=0, w=OBSTACLE_EDGE_SIZE, h=BOX_HEIGHT),  # Right
    # Obstacle(x=4.5, y=3.5, w=0.5, h=0.5, angle=np.deg2rad(45))  # Random obstacle
]
ROBOTS = [
    Robot(env="bot1", pos=np.array([1.0, 1.0]), theta=0.0, enable_lidar=False),
    Robot(env="bot2", pos=np.array([1.8, 1.0]), theta=0.0, enable_lidar=False),
    Robot(env="bot3", pos=np.array([1.0, 1.8]), theta=np.pi, enable_lidar=False),
    Robot(env="bot4", pos=np.array([1.8, 1.8]), theta=0.0, enable_lidar=False),
    Robot(env="bot5", pos=np.array([3.0, 2.0]), theta=0.0, enable_lidar=False),
    Robot(env="bot6", pos=np.array([2.0, 3.0]), theta=0.0, enable_lidar=False),
    Robot(env="bot7", pos=np.array([1.0, 4.0]), theta=0.0, enable_lidar=False),
]
TARGET = np.array([7.0, 5.0])  # None / np.array([7.0, 5.0])
CONTROLLER = DWAFlockingController
VIZ = PygameViz(900, 700, FPS)


if __name__ == "__main__":
    ctrl = CONTROLLER()
    # ctrl.record_metrics()
    ctrl.set_target(TARGET)
    for bot in ROBOTS:
        ctrl.add_robot(bot)
    ctrl.set_target(TARGET)

    viz = VIZ
    viz.set_center((BOX_WIDTH / 2, BOX_HEIGHT / 2))
    viz.set_zoom(0.8)
    viz.set_ctrl(ctrl)
    viz.set_obstacles(OBSTACLES)

    viz.start()
    while viz.running:
        viz.step()
    print("Done.")
