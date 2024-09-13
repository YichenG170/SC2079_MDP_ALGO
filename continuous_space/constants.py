# constants.py
from enum import Enum

class Direction(int, Enum):
    UP = 90
    LEFT = 180
    DOWN = 270
    RIGHT = 0

MOVE = [(0, 1), (0, -1), (-1, 1), (1, 1)]

ROBOT_W = 24
ROBOT_H = 19

OBSTACLE_W = 10
OBSTACLE_H = 10

FIELD_W = 200
FIELD_H = 200

OBS_D = 15

TURN_RADIUS = 18

SAMPLE_DISTANCE = 2