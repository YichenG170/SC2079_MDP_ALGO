# constants.py
from enum import Enum
import multiprocess as mp

BOXSIZE_EP = 5000
IMAGE_BORDER_SIZE = 400
MIN_BOX_SIZE = 13500
BULLSEYE_NEG = -0.15
MIN_CONF = 0.4

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
ROBOT_W = 26
ROBOT_H = 22

INITIAL_X = 12
INITIAL_Y = 13

OBSTACLE_W = 15
OBSTACLE_H = 15

FIELD_W = 210
FIELD_H = 210

CELL_SIZE = 10

TURN_RADIUS = 30  # Fixed turning radius for quarter-circle turns

DEGREE_90 = 00 # a 90 degree irl is DEGREE_90 in the simulation

# Sampling for collision detection
SAMPLE_DISTANCE = 2  # Units between samples when checking collision along paths

# Movement step size
MOVE_STEP = 2  # Units for straight movements
#2
#5

# Threshold for goal detection
GOAL_THRESHOLD = 2
#2
#2

# Minimum allowed distance between the robot and objects
MIN_DISTANCE = 0.5 # Bugs here, leave it 1 or 0.5

# Extra expanded distance for turn projections
EXPAND_DISTANCE = 5

# Observation distance
OBSERVATION_DISTANCE = 25 + ROBOT_H / 2

# Rotation cost
ROTATION_COST = 0

# Core number
CORE_NUM = mp.cpu_count()

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
LIGHT_GREY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
DARK_GREY = (100, 100, 100)