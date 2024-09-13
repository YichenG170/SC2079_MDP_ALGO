from entities import *
from constants import *
from constants import Direction as d

def a_star_search(field: Field, target_pos: list, target_d = Direction):
    robot = field.get_robot()
    start_pos = robot.get_center_pos()
    start_d = robot.get_center_pos()
    
    
def get_rotation_cost(current_d: Direction, next_d: Direction):
    return abs(current_d - next_d) % 2