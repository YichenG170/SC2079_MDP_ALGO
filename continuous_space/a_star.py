# a_star.py
import math
import heapq
from entities import *
from constants import FIELD_W, FIELD_H, TURN_RADIUS

class State:
    def __init__(self, x, y, theta, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta % 360  # Orientation in degrees (0-360)
        self.g = g          # Cost from start node
        self.h = h          # Heuristic cost to goal
        self.f = g + h      # Total cost
        self.parent = parent  # Parent state in the path

    def __lt__(self, other):  # For priority queue
        return self.f < other.f

def a_star_search(field: Field, target_pos: list, target_theta: float):
    robot = field.get_robot()
    start_pos = robot.get_center_pos()
    start_theta = robot.get_theta()

    target_x, target_y = target_pos
    target_theta = target_theta % 360

    # Discretization parameters
    STEP_SIZE = 5       # Movement step size
    ANGLE_STEP = 15     # Rotation step size in degrees

    open_list = []
    closed_set = set()

    start_state = State(start_pos[0], start_pos[1], start_theta)
    heapq.heappush(open_list, start_state)

    while open_list:
        current_state = heapq.heappop(open_list)

        # Check if goal is reached
        if is_goal(current_state, target_x, target_y, target_theta):
            # Reconstruct path
            return reconstruct_path(current_state)

        state_id = (round(current_state.x, 1), round(current_state.y, 1), round(current_state.theta, 1) % 360)
        if state_id in closed_set:
            continue
        closed_set.add(state_id)

        # Generate successors
        successors = get_successors(current_state, STEP_SIZE, ANGLE_STEP, field)

        for next_state in successors:
            next_state_id = (round(next_state.x, 1), round(next_state.y, 1), round(next_state.theta, 1) % 360)
            if next_state_id in closed_set:
                continue

            if is_valid(next_state, field):
                # Compute costs
                next_state.g = current_state.g + movement_cost(current_state, next_state)
                next_state.h = heuristic(next_state, target_x, target_y, target_theta)
                next_state.f = next_state.g + next_state.h
                next_state.parent = current_state

                heapq.heappush(open_list, next_state)

    # No path found
    return None

def is_goal(state, target_x, target_y, target_theta):
    # Determine if the current state is the goal
    pos_threshold = 1.0    # Position tolerance
    angle_threshold = 5.0  # Angle tolerance

    distance = math.hypot(state.x - target_x, state.y - target_y)
    angle_diff = abs((state.theta - target_theta + 180) % 360 - 180)

    return distance <= pos_threshold and angle_diff <= angle_threshold

def reconstruct_path(state):
    # Reconstruct the path from start to goal
    path = []
    while state:
        path.append((state.x, state.y, state.theta))
        state = state.parent
    return path[::-1]  # Reverse the path

def get_successors(state, step_size, angle_step, field):
    successors = []

    # Move forward
    new_x = state.x + step_size * math.cos(math.radians(state.theta))
    new_y = state.y + step_size * math.sin(math.radians(state.theta))
    successors.append(State(new_x, new_y, state.theta))

    # Left turn
    left_state = turn(state, angle_step, field.robot.get_width(), left=True)
    successors.append(left_state)

    # Right turn
    right_state = turn(state, angle_step, field.robot.get_width(), left=False)
    successors.append(right_state)

    return successors

def turn(state, delta_theta, robot_width, left=True):
    # Perform a turn around the corresponding back wheel
    theta_rad = math.radians(state.theta)
    delta_rad = math.radians(delta_theta) if left else -math.radians(delta_theta)

    # Calculate pivot point (back wheel)
    dx = (-robot_width / 2) * math.sin(theta_rad)
    dy = (robot_width / 2) * math.cos(theta_rad)
    pivot_x = state.x + dx if left else state.x - dx
    pivot_y = state.y + dy if left else state.y - dy

    # Calculate new position after rotation
    sin_delta = math.sin(delta_rad)
    cos_delta = math.cos(delta_rad)
    x = cos_delta * (state.x - pivot_x) - sin_delta * (state.y - pivot_y) + pivot_x
    y = sin_delta * (state.x - pivot_x) + cos_delta * (state.y - pivot_y) + pivot_y
    new_theta = (state.theta + delta_theta) % 360 if left else (state.theta - delta_theta) % 360

    return State(x, y, new_theta)

def is_valid(state, field):
    # Check for collision with obstacles and field boundaries
    if not (0 <= state.x <= FIELD_W and 0 <= state.y <= FIELD_H):
        return False

    # Update robot position and orientation
    robot = field.get_robot()
    robot.set_center_pos([state.x, state.y])
    robot.set_theta(state.theta)

    # Get robot corners
    robot_corners = robot.get_pos()

    # Check collision with obstacles
    for obstacle in field.get_obstacles():
        obstacle_corners = obstacle.get_pos()
        if polygons_intersect(robot_corners, obstacle_corners):
            return False

    return True

def movement_cost(current_state, next_state):
    # Calculate movement cost including path and rotation cost
    distance = math.hypot(next_state.x - current_state.x, next_state.y - current_state.y)
    angle_diff = abs((next_state.theta - current_state.theta + 180) % 360 - 180)
    rotation_cost = (angle_diff / 360.0) * TURN_RADIUS

    return distance + rotation_cost

def heuristic(state, target_x, target_y, target_theta):
    # Estimate cost to reach the goal
    distance = math.hypot(state.x - target_x, state.y - target_y)
    angle_diff = abs((state.theta - target_theta + 180) % 360 - 180)
    rotation_cost = (angle_diff / 360.0) * TURN_RADIUS

    return distance + rotation_cost

def polygons_intersect(poly1, poly2):
    # Check if two polygons intersect (using Separating Axis Theorem)
    def get_axes(polygon):
        axes = []
        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]
            edge = [p2[0] - p1[0], p2[1] - p1[1]]
            normal = [-edge[1], edge[0]]
            length = math.hypot(normal[0], normal[1])
            axes.append([normal[0] / length, normal[1] / length])
        return axes

    def project_polygon(axis, polygon):
        dots = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
        return [min(dots), max(dots)]

    axes = get_axes(poly1) + get_axes(poly2)
    for axis in axes:
        proj1 = project_polygon(axis, poly1)
        proj2 = project_polygon(axis, poly2)
        if proj1[1] < proj2[0] or proj2[1] < proj1[0]:
            return False
    return True