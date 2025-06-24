from controllers.GenericController import GenericController

""" Generic Visualiser Class
This class serves as a base for all visualisers in the PythonRobot framework.
It provides methods for starting the visualiser, setting the controller, obstacles,
and zoom level, as well as converting between world and screen coordinates.

It's purpose is to allow creation of various visualisers, that use different libraries,
such as pygame, matplotlib, Qt, etc., while maintaining a consistent interface.
"""


class GenericViz:
    DEFAULT_PX = 100.0  # pixels per meter
    FPS = 60
    BACKGROUND_COLOR = (255, 255, 255)
    TARGET_COLOR = (0, 0, 255)
    OBSTACLE_COLOR = (100, 100, 100)
    LIDAR_COLOR = (255, 0, 0)
    ROBOT_COLOR = (0, 255, 0)
    """
    Generic visualiser class.
    """

    def __init__(self, width, height):
        """
        Initialise the visualiser with the data.
        """
        self.running = False
        self.ctrl: GenericController = None
        self.obstacles = []
        self.zoom = 1.0
        self.flip_y = True
        self.map = None
        self.window_size = (width, height)
        self.center = (0, 0)

    def start(self):
        """
        Start the visualiser.
        """
        self.running = True

    def set_center(self, center):
        """
        Set the center of the visualiser.
        """
        self.center = center

    def set_zoom(self, zoom):
        """
        Set the zoom level of the visualiser.
        """
        self.zoom = zoom

    def set_ctrl(self, ctrl):
        """
        Set the controller for the visualiser.
        """
        self.ctrl = ctrl

    def set_obstacles(self, obstacles):
        """
        Set the obstacles for the visualiser.
        """
        self.obstacles = obstacles

    def step(self):
        """
        Step the controller and draw the robots on the visualiser.
        """
        raise NotImplementedError("step() must be implemented in subclasses")

    def draw(self):
        """
        Draw the robots and obstacles on the visualiser.
        """
        raise NotImplementedError("draw() must be implemented in subclasses")

    def draw_obstacle(self, obstacle, color=None):
        """
        Draw an obstacle on the visualiser.
        """
        raise NotImplementedError("draw_obstacle() must be implemented in subclasses")

    def display_cursor_position(self):
        """
        Display the cursor position on the visualiser.
        """
        raise NotImplementedError("display_cursor_position() must be implemented in subclasses")

    def world_to_screen(self, pos_meters):
        """
        Convert world coordinates to screen coordinates.
        """
        dx = pos_meters[0] - self.center[0]
        dy = pos_meters[1] - self.center[1]

        px_per_meter = self.DEFAULT_PX * self.zoom
        x = self.window_size[0] / 2 + dx * px_per_meter
        y = self.window_size[1] / 2
        if not self.flip_y:
            y += dy * px_per_meter
        else:
            y -= dy * px_per_meter
        return (int(x), int(y))

    def screen_to_world(self, pos_pixels):
        """
        Converts a position in pixels (screen) to meters (world).
        """
        dx = pos_pixels[0] - self.window_size[0] / 2
        if not self.flip_y:
            dy = pos_pixels[1] - self.window_size[1] / 2
        else:
            dy = self.window_size[1] / 2 - pos_pixels[1]

        px_per_meter = self.DEFAULT_PX * self.zoom
        x = self.center[0] + dx / px_per_meter
        y = self.center[1] + dy / px_per_meter
        return (x, y)
