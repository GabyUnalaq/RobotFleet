import pygame
import time
import numpy as np
from utils import quaternion_to_euler
from .GridMap import Map, CHUNK_SIZE, RESOLUTION
from .GenericViz import GenericViz
from robots.GenericRobot import GenericRobot

""" Pygame Visualiser
This visualiser uses Pygame to render the robots, obstacles, and Lidar data in a 2D space.
It supports zooming, panning, and displaying a grid map for Lidar data.
"""


class PygameViz(GenericViz):
    """
    Pygame visualiser class.
    """

    def __init__(self, width, height, fps=60):
        """
        Initialise the visualiser with the data.
        """
        super().__init__(width, height)
        self.screen = None
        self.cursor_pos_font = None
        self.clock = None
        self.fps = fps
        self.map = None
        self.events = {
            "left_click_start_pos": None, "left_click_end_pos": None,
            "left_click_start_time": None, "left_click_end_time": None,
            "right_click_start_pos": None, "right_click_end_pos": None,
            "right_click_start_time": None, "right_click_end_time": None,
            "drag_start_pos": None, "last_mouse_pos": None,
            "dragging": False,
        }

        self.init_pygame()

        self.robot_img = pygame.image.load("robots/car.png").convert_alpha()

    def __del__(self):
        pygame.quit()

    def init_pygame(self):
        pygame.init()
        self.cursor_pos_font = pygame.font.SysFont("Arial", 16)
        self.screen = pygame.display.set_mode(self.window_size)
        pygame.display.set_caption("2D Transbot Viz")
        self.clock = pygame.time.Clock()

    def step(self):
        """
        Draw the robots on the visualiser.
        """
        dt = self.clock.tick(self.FPS) / 1000.0  # in seconds
        self.screen.fill(self.BACKGROUND_COLOR)

        # Events
        self._handle_events()

        # Advance controller
        self.ctrl.step()
        self.ctrl.advance(dt, self.obstacles)

        # Draw
        self.draw()
        if self.map is not None:
            self.map.decay_all()
        self.display_cursor_position()
        pygame.display.flip()

    def draw(self):
        # Draw obstacles
        for obs in self.obstacles:
            self.draw_obstacle(obs)

        # Draw robots
        for bot_id, bot in enumerate(self.ctrl.robots):
            self.draw_robot(bot)

            # Draw Lidar
            lidar_points = bot.lidar_point_cloud
            # world_lidar_points = bot.odom_matrix.apply_on_points(lidar_points)
            for pt in lidar_points:
                if self.map is not None:
                    self.map.update(pt[0], pt[1])
                screen_pt = self.world_to_screen(pt)
                pygame.draw.circle(self.screen, self.LIDAR_COLOR, screen_pt, int(4*self.zoom))
                # pygame.draw.line(self.screen, self.LIDAR_COLOR,
                #                  self.world_to_screen(tuple(bot.pos)), screen_pt, int(self.zoom))

            # Draw control vector
            traj = self.ctrl.simulate_trajectory(bot_id, dt=0.2, steps=30)
            for pt in traj:
                screen_pt = self.world_to_screen(pt[:2])
                pygame.draw.circle(self.screen, self.ROBOT_COLOR, screen_pt, int(3*self.zoom))

        # Draw target
        target = self.ctrl.target
        if target is not None:
            pygame.draw.circle(self.screen, self.TARGET_COLOR, self.world_to_screen(target), int(10*self.zoom))

        # Draw map
        if self.map is not None:
            for (chunk_x, chunk_y), chunk in self.map.chunks.items():
                size = chunk.grid.shape[0]
                for i in range(size):
                    for j in range(size):
                        conf = chunk.grid[i, j]
                        if conf > 0.1:
                            wx = chunk_x * CHUNK_SIZE + i * RESOLUTION - CHUNK_SIZE / 2
                            wy = chunk_y * CHUNK_SIZE + j * RESOLUTION - CHUNK_SIZE / 2
                            sx, sy = self.world_to_screen((wx, wy))
                            gray = 255 - int(255 * conf)
                            pygame.draw.circle(self.screen, (gray, gray, gray), (sx, sy), int(10*self.zoom))

    def draw_robot(self, bot):
        obstacle_color = self.ROBOT_COLOR
        if bot.robot_name == "bot01":
            obstacle_color = (255, 0, 0)  # Red for bot01
        if bot.robot_name == "bot02":
            obstacle_color = (0, 255, 0)  # Green for bot02
        if bot.robot_name == "bot04":
            obstacle_color = (255, 255, 0)  # Yellow for bot04
        # self.draw_obstacle(bot.obstacle, color=obstacle_color)

        # Draw the robot as an image
        pos = bot.position[:2]
        yaw = quaternion_to_euler(bot.orientation)[2]

        px_per_meter = self.DEFAULT_PX * self.zoom
        width_px = int((GenericRobot.TRANSBOT_WIDTH + 0.01) * px_per_meter)
        height_px = int((GenericRobot.TRANSBOT_LENGTH + 0.01) * px_per_meter)

        scaled = pygame.transform.scale(self.robot_img, (width_px, height_px))
        rotated = pygame.transform.rotate(scaled, np.degrees(yaw) - 90)
        center = list(self.world_to_screen(pos))
        # center[1] -= 4*self.zoom
        rect = rotated.get_rect(center=center)
        self.screen.blit(rotated, rect)


    def draw_obstacle(self, obstacle, color=None):
        """
        Draw a single obstacle on the visualiser.
        """
        color = color if color else self.OBSTACLE_COLOR
        corners = obstacle.get_corners()
        screen_corners = [self.world_to_screen(tuple(pt)) for pt in corners]
        pygame.draw.polygon(self.screen, color, screen_corners)

    def display_cursor_position(self):
        """
        Display the cursor position in world coordinates.
        """
        mouse_pos_px = pygame.mouse.get_pos()
        mouse_pos_world = self.screen_to_world(mouse_pos_px)
        coords_text = f"({mouse_pos_world[0]:.2f}, {mouse_pos_world[1]:.2f}) m"
        text_surface = self.cursor_pos_font.render(coords_text, True, (0, 0, 0))
        text_rect = text_surface.get_rect(bottomright=(self.window_size[0] - 10, self.window_size[1] - 10))
        self.screen.blit(text_surface, text_rect)

    def _handle_events(self):
        CLICK_DURATION = 0.2  # seconds
        MOVE_THRESHOLD = 5  # pixels to consider as drag

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == pygame.BUTTON_LEFT:
                    pos = pygame.mouse.get_pos()
                    self.events["left_click_start_pos"] = pos
                    self.events["left_click_start_time"] = time.time()
                    self.events["drag_start_pos"] = pos
                    self.events["last_mouse_pos"] = pos
                    self.events["dragging"] = True

                elif event.button == pygame.BUTTON_RIGHT:
                    pos = pygame.mouse.get_pos()
                    self.events["right_click_start_pos"] = pos
                    self.events["right_click_start_time"] = time.time()

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == pygame.BUTTON_LEFT:
                    self.events["dragging"] = False
                    pos = pygame.mouse.get_pos()
                    self.events["left_click_end_pos"] = pos
                    self.events["left_click_end_time"] = time.time()

                    # Detect if it's a click (not a drag)
                    duration = self.events["left_click_end_time"] - self.events["left_click_start_time"]
                    start_pos = self.events["left_click_start_pos"]
                    distance = pygame.math.Vector2(pos).distance_to(start_pos)
                    if duration < CLICK_DURATION and distance < MOVE_THRESHOLD:
                        pos_world = self.screen_to_world(pos)
                        self.ctrl.set_target(np.array(pos_world))

                elif event.button == pygame.BUTTON_RIGHT:
                    pos = pygame.mouse.get_pos()
                    self.events["right_click_end_pos"] = pos
                    self.events["right_click_end_time"] = time.time()

                    duration = self.events["right_click_end_time"] - self.events["right_click_start_time"]
                    start_pos = self.events["right_click_start_pos"]
                    distance = pygame.math.Vector2(pos).distance_to(start_pos)
                    if duration < CLICK_DURATION and distance < MOVE_THRESHOLD:
                        self.ctrl.set_target(None)

            elif event.type == pygame.MOUSEMOTION:
                if self.events["dragging"]:
                    current_pos = pygame.mouse.get_pos()
                    delta_pixels = pygame.math.Vector2(current_pos) - pygame.math.Vector2(self.events["last_mouse_pos"])
                    self.events["last_mouse_pos"] = current_pos

                    pixels_per_meter = self.DEFAULT_PX * self.zoom
                    dx_m = -delta_pixels.x / pixels_per_meter
                    dy_m = -delta_pixels.y / pixels_per_meter
                    delta_meters = (dx_m, -dy_m if self.flip_y else dy_m)
                    self.center = (self.center[0] + delta_meters[0],
                                self.center[1] + delta_meters[1])
            elif event.type == pygame.MOUSEWHEEL:
                self.zoom *= 1.1 if event.y > 0 else 0.9

                # Optional: Zoom to mouse position
                mouse_px = pygame.mouse.get_pos()
                mouse_world_before = self.screen_to_world(mouse_px)
                mouse_world_after = self.screen_to_world(mouse_px)

                dx = mouse_world_before[0] - mouse_world_after[0]
                dy = mouse_world_before[1] - mouse_world_after[1]

                self.center = (self.center[0] + dx, self.center[1] + dy)
