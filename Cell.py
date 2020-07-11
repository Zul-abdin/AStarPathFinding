import sys

class Cell:
    def __init__(self, x, y, wall):
        self.wall = wall
        self.x = x
        self.y = y
        self.parent = None
        self.f = sys.maxsize
        self.g = 0
        self.h = sys.maxsize