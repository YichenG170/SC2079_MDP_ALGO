from entities import *
from constants import *
from constants import Direction as d

def a_star_search(field: Field, target_pos: list, target_d = Direction):
    robot = field.get_robot()
    start_pos = robot.get_center_pos()
    start_d = robot.get_center_pos()
    
    
def get_rotation_cost(current_d: Direction, next_d: Direction):
    return abs(current_d - next_d) % 2

    # Define a list of target positions and orientations
    targets = []
    
    obstacles = field.get_obstacles()
    for obstacle in obstacles:
        (x, y) = obstacle.get_center_pos()
        d = obstacle.get_theta()
        
        if d == Direction.UP:
            targets.append(((x, y + OBSERVATION_DISTANCE + 1), Direction.DOWN))
        elif d == Direction.DOWN:
            targets.append(((x, y - OBSERVATION_DISTANCE - 1), Direction.UP))
        elif d == Direction.LEFT:
            targets.append(((x - OBSERVATION_DISTANCE - 1, y), Direction.RIGHT))
        elif d == Direction.RIGHT:
            targets.append(((x + OBSERVATION_DISTANCE + 1, y), Direction.LEFT))