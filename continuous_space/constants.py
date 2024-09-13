# constants.py
from enum import Enum

class Direction(int, Enum):
    RIGHT = 0
    UP = 90
    LEFT = 180
    DOWN = 270

# Movement directions and actions
ACTIONS = [
    'GO_FORWARD',
    'GO_BACKWARD',
    'TURN_LEFT_FORWARD',
    'TURN_LEFT_BACKWARD',
    'TURN_RIGHT_FORWARD',
    'TURN_RIGHT_BACKWARD'
]

# Dimensions (units)
ROBOT_W = 24
ROBOT_H = 19

OBSTACLE_W = 10
OBSTACLE_H = 10

FIELD_W = 200
FIELD_H = 200

TURN_RADIUS = 18  # Fixed turning radius for quarter-circle turns

# Sampling for collision detection
SAMPLE_DISTANCE = 2  # Units between samples when checking collision along paths

# Movement step size
MOVE_STEP = 10  # Units for straight movements
