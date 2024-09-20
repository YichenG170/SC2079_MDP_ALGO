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

CELL_SIZE = 10

TURN_RADIUS = 18  # Fixed turning radius for quarter-circle turns

# Sampling for collision detection
SAMPLE_DISTANCE = 2  # Units between samples when checking collision along paths

# Movement step size
MOVE_STEP = 2  # Units for straight movements

# Observation distance
OBSERVATION_DISTANCE = 15 + ROBOT_H / 2

# Rotation cost
ROTATION_COST = 100

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
LIGHT_GREY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
DARK_GREY = (100, 100, 100)