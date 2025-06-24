import numpy as np
from collections import defaultdict
import time

""" Grid Map Module
This module implements a supposed grid map for Lidar visualisation.
It does not work and was not tested with the real sensors.
"""


CHUNK_SIZE = 50  # meters
RESOLUTION = 0.1  # meters/cell
CONF_DECAY = 0.01
CONF_MAX = 1.0
CONF_MIN = 0.0


class MapChunk:
    def __init__(self):
        size = int(CHUNK_SIZE / RESOLUTION)
        self.grid = np.zeros((size, size), dtype=np.float32)
        self.last_update = time.time()

    def world_to_cell(self, x, y):
        cx = int((x + CHUNK_SIZE / 2) / RESOLUTION)
        cy = int((y + CHUNK_SIZE / 2) / RESOLUTION)
        return cx, cy

    def update_point(self, x, y):
        cx, cy = self.world_to_cell(x, y)
        if 0 <= cx < self.grid.shape[0] and 0 <= cy < self.grid.shape[1]:
            self.grid[cx, cy] = min(CONF_MAX, self.grid[cx, cy] + 0.5)

    def decay(self):
        dt = time.time() - self.last_update
        self.grid = np.maximum(CONF_MIN, self.grid - CONF_DECAY * dt)
        self.last_update = time.time()


class Map:
    def __init__(self):
        self.chunks = defaultdict(MapChunk)

    def get_chunk_key(self, x, y):
        return int(x // CHUNK_SIZE), int(y // CHUNK_SIZE)

    def update(self, x, y):
        key = self.get_chunk_key(x, y)
        self.chunks[key].update_point(x % CHUNK_SIZE, y % CHUNK_SIZE)

    def decay_all(self):
        for chunk in self.chunks.values():
            chunk.decay()
