from enum import Enum

class Direction(int, Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    
    def __init__(self):
        return self.value
    
MOVE = [(0, 1), (0, -1), (-1, 1), (1, 1)]

ROBOT_W = 19
ROBOT_H = 24

OBSTACLE_W = 10
OBSTACLE_H = 10

FIELD_W = 200
FIELD_H = 200

OBS_D = 15

TURN_RADIUS = 18