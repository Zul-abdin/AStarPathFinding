import sys

class Cell:
    def __init__(self, x, y, wall):
        self.wall = wall
        self.x = x
        self.y = y
        self.parent = None
        self.f = sys.maxsize
        self.g = sys.maxsize
        self.h = sys.maxsize