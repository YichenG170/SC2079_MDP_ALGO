# a_star.py
import math
import heapq
from itertools import permutations
from entities import Robot, Obstacle, Field
from constants import FIELD_W, FIELD_H, TURN_RADIUS, SAMPLE_DISTANCE, ACTIONS, MOVE_STEP

class State:
    def __init__(self, x, y, theta, g=0, h=0, parent=None, action=None):
        self.x = x                  # Current x position
        self.y = y                  # Current y position
        self.theta = theta % 360    # Current orientation in degrees
        self.g = g                  # Cost from start
        self.h = h                  # Heuristic cost to goal
        self.f = g + h              # Total cost
        self.parent = parent        # Parent state
        self.action = action        # Action taken to reach this state

    def __lt__(self, other):
        return self.f < other.f

def a_star_search(field: Field, target_pos: list, target_theta: float):
    robot = field.get_robot()
    start_x, start_y = robot.get_center_pos()
    start_theta = robot.get_theta()

    target_x, target_y = target_pos
    target_theta = target_theta % 360

    open_list = []
    closed_set = set()

    start_state = State(start_x, start_y, start_theta)
    heapq.heappush(open_list, start_state)

    while open_list:
        current_state = heapq.heappop(open_list)

        # Goal check
        if is_goal(current_state, target_x, target_y, target_theta):
            return reconstruct_path(current_state)

        state_id = (round(current_state.x, 1), round(current_state.y, 1), round(current_state.theta, 1) % 360)
        if state_id in closed_set:
            continue
        closed_set.add(state_id)

        # Generate successors
        successors = get_successors(current_state, field)

        for next_state in successors:
            next_state_id = (round(next_state.x, 1), round(next_state.y, 1), round(next_state.theta, 1) % 360)
            if next_state_id in closed_set:
                continue

            if is_valid(next_state, field, current_state):
                # Calculate costs
                next_state.g = current_state.g + movement_cost(current_state, next_state)
                next_state.h = heuristic(next_state, target_x, target_y, target_theta)
                next_state.f = next_state.g + next_state.h
                next_state.parent = current_state

                heapq.heappush(open_list, next_state)

    # No path found
    return None

def is_goal(state, target_x, target_y, target_theta):
    pos_threshold = 1.0    # Position tolerance
    angle_threshold = 5.0  # Orientation tolerance

    distance = math.hypot(state.x - target_x, state.y - target_y)
    angle_diff = abs((state.theta - target_theta + 180) % 360 - 180)

    return distance <= pos_threshold and angle_diff <= angle_threshold

def reconstruct_path(state):
    path = []
    while state:
        path.append((state.x, state.y, state.theta, state.action))
        state = state.parent
    return path[::-1]  # Reverse the path to start from the beginning

def get_successors(state, field):
    successors = []
    robot = field.get_robot()

    for action in ACTIONS:
        if action == 'GO_FORWARD':
            new_x, new_y, new_theta = move_forward(state.x, state.y, state.theta, MOVE_STEP)
            successor = State(new_x, new_y, new_theta, parent=state, action=action)
            successors.append(successor)

        elif action == 'GO_BACKWARD':
            new_x, new_y, new_theta = move_backward(state.x, state.y, state.theta, MOVE_STEP)
            successor = State(new_x, new_y, new_theta, parent=state, action=action)
            successors.append(successor)

        elif action in ['TURN_LEFT_FORWARD', 'TURN_LEFT_BACKWARD',
                       'TURN_RIGHT_FORWARD', 'TURN_RIGHT_BACKWARD']:
            turn_params = parse_turn_action(action)
            new_x, new_y, new_theta = perform_turn(state.x, state.y, state.theta, turn_params['direction'],
                                                  turn_params['movement'])
            successor = State(new_x, new_y, new_theta, parent=state, action=action)
            successors.append(successor)

    return successors

def parse_turn_action(action):
    """
    Parses the turn action to extract turning direction and movement direction.
    """
    parts = action.split('_')
    turn_direction = parts[1].lower()  # 'left' or 'right'
    movement_direction = parts[2].lower()  # 'forward' or 'backward'
    return {'direction': turn_direction, 'movement': movement_direction}

def move_forward(x, y, theta, step):
    theta_rad = math.radians(theta)
    new_x = x + step * math.cos(theta_rad)
    new_y = y + step * math.sin(theta_rad)
    return new_x, new_y, theta

def move_backward(x, y, theta, step):
    theta_rad = math.radians(theta)
    new_x = x - step * math.cos(theta_rad)
    new_y = y - step * math.sin(theta_rad)
    return new_x, new_y, theta

def perform_turn(x, y, theta, turn_direction, movement_direction):
    """
    Performs a quarter-circle turn.
    :param x: Current x position
    :param y: Current y position
    :param theta: Current orientation in degrees
    :param turn_direction: 'left' or 'right'
    :param movement_direction: 'forward' or 'backward'
    :return: New x, y, theta after the turn
    """
    theta_rad = math.radians(theta)
    delta_theta = 90 if turn_direction == 'left' else -90
    if movement_direction == 'backward':
        delta_theta = -delta_theta

    # Determine center of rotation
    if turn_direction == 'left':
        center_x = x - TURN_RADIUS * math.sin(theta_rad)
        center_y = y + TURN_RADIUS * math.cos(theta_rad)
    else:
        center_x = x + TURN_RADIUS * math.sin(theta_rad)
        center_y = y - TURN_RADIUS * math.cos(theta_rad)

    # Calculate new orientation
    new_theta = (theta + delta_theta) % 360

    # Calculate new position after the quarter-circle turn
    angle_start = math.atan2(y - center_y, x - center_x)
    angle_end = angle_start + math.radians(delta_theta)
    new_x = center_x + TURN_RADIUS * math.cos(angle_end)
    new_y = center_y + TURN_RADIUS * math.sin(angle_end)

    return new_x, new_y, new_theta

def is_valid(state, field, parent_state=None):
    """
    Checks if the state is valid (no collision and within boundaries).
    """
    if parent_state is None:
        # Start state; just check current position
        return check_collision(state, field)

    action = state.action
    if action in ['GO_FORWARD', 'GO_BACKWARD']:
        # Straight movement
        return check_straight_path(parent_state, state, field)
    elif action.startswith('TURN_'):
        # Turning movement
        return check_turning_path(parent_state, state, field)
    else:
        return False

def check_straight_path(start_state, end_state, field):
    """
    Checks for collisions along a straight path from start to end.
    """
    distance = math.hypot(end_state.x - start_state.x, end_state.y - start_state.y)
    num_samples = max(1, int(distance / SAMPLE_DISTANCE))
    for i in range(1, num_samples + 1):
        t = i / num_samples
        x = start_state.x + t * (end_state.x - start_state.x)
        y = start_state.y + t * (end_state.y - start_state.y)
        temp_state = State(x, y, start_state.theta)
        if not check_collision(temp_state, field):
            return False
    return True

def check_turning_path(start_state, end_state, field):
    """
    Checks for collisions along a circular turning path from start to end.
    """
    action = end_state.action
    params = parse_turn_action(action)
    turn_direction = params['direction']
    movement_direction = params['movement']

    theta_rad = math.radians(start_state.theta)
    if turn_direction == 'left':
        center_x = start_state.x - TURN_RADIUS * math.sin(theta_rad)
        center_y = start_state.y + TURN_RADIUS * math.cos(theta_rad)
    else:
        center_x = start_state.x + TURN_RADIUS * math.sin(theta_rad)
        center_y = start_state.y - TURN_RADIUS * math.cos(theta_rad)

    # Determine the direction of angle change
    delta_theta = 90 if turn_direction == 'left' else -90
    if movement_direction == 'backward':
        delta_theta = -delta_theta

    angle_start = math.atan2(start_state.y - center_y, start_state.x - center_x)
    angle_end = angle_start + math.radians(delta_theta)

    arc_length = abs(delta_theta) * math.pi / 180 * TURN_RADIUS
    num_samples = max(1, int(arc_length / SAMPLE_DISTANCE))

    for i in range(1, num_samples + 1):
        t = i / num_samples
        current_angle = angle_start + t * math.radians(delta_theta)
        x = center_x + TURN_RADIUS * math.cos(current_angle)
        y = center_y + TURN_RADIUS * math.sin(current_angle)
        theta = (start_state.theta + t * delta_theta) % 360
        temp_state = State(x, y, theta)
        if not check_collision(temp_state, field):
            return False
    return True

def check_collision(state, field):
    """
    Checks if the robot at the given state collides with any obstacles or boundaries.
    """
    # Check boundaries
    if not (0 <= state.x <= FIELD_W and 0 <= state.y <= FIELD_H):
        return False

    robot = field.get_robot()
    original_pos = robot.get_center_pos()
    original_theta = robot.get_theta()

    # Temporarily update robot's position and orientation
    robot.set_center_pos([state.x, state.y])
    robot.set_theta(state.theta)

    # Get robot's corners
    robot_corners = robot.get_pos()

    # Check collision with all obstacles
    for obstacle in field.get_obstacles():
        obstacle_corners = obstacle.get_pos()
        if polygons_intersect(robot_corners, obstacle_corners):
            # Reset robot's position and orientation
            robot.set_center_pos(original_pos)
            robot.set_theta(original_theta)
            return False

    # Reset robot's position and orientation
    robot.set_center_pos(original_pos)
    robot.set_theta(original_theta)

    return True

def movement_cost(current_state, next_state):
    """
    Calculates the movement cost between two states.
    """
    angle_diff = abs((next_state.theta - current_state.theta + 180) % 360 - 180)
    if angle_diff == 0:
        # Straight movement
        distance = math.hypot(next_state.x - current_state.x, next_state.y - current_state.y)
        return distance
    elif angle_diff == 90:
        # Quarter-circle turn
        arc_length = (math.pi * TURN_RADIUS) / 2  # 1/4 of circumference
        return arc_length
    else:
        # Unsupported movement
        return float('inf')

def heuristic(state, target_x, target_y, target_theta):
    """
    Heuristic function estimating the cost from the current state to the goal.
    Uses Euclidean distance plus orientation difference.
    """
    distance = math.hypot(state.x - target_x, state.y - target_y)
    angle_diff = abs((state.theta - target_theta + 180) % 360 - 180)
    rotation_cost = (angle_diff / 360.0) * TURN_RADIUS
    return distance + rotation_cost

def polygons_intersect(poly1, poly2):
    """
    Determines if two convex polygons intersect using the Separating Axis Theorem.
    """
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
