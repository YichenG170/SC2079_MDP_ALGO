# a_star.py
import math
import heapq
from entities import *
from constants import *

def a_star_search(field: Field, target_pos: list, target_theta: float):
    robot = field.get_robot()
    start_pos = robot.get_center_pos()
    start_theta = robot.get_theta()

    target_x, target_y = target_pos
    target_theta = target_theta % 360

    # Discretization parameters

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
        successors = get_successors(current_state, field)

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

def get_successors(state, field):
    successors = []
    TURN_RADIUS = 18  # Turning radius for quarter-circle turns
    STEP_SIZE = 10    # You can adjust this value as needed

    # Action: Go forward x units
    new_x = state.x + STEP_SIZE * math.cos(math.radians(state.theta))
    new_y = state.y + STEP_SIZE * math.sin(math.radians(state.theta))
    successors.append(State(new_x, new_y, state.theta))

    # Action: Go backward x units
    back_x = state.x - STEP_SIZE * math.cos(math.radians(state.theta))
    back_y = state.y - STEP_SIZE * math.sin(math.radians(state.theta))
    successors.append(State(back_x, back_y, state.theta))

    # Action: Go left forward (quarter-circle turn)
    left_forward_state = perform_turn(state, TURN_RADIUS, left=True, forward=True)
    successors.append(left_forward_state)

    # Action: Go left backward (reverse quarter-circle turn)
    left_backward_state = perform_turn(state, TURN_RADIUS, left=True, forward=False)
    successors.append(left_backward_state)

    # Action: Go right forward (quarter-circle turn)
    right_forward_state = perform_turn(state, TURN_RADIUS, left=False, forward=True)
    successors.append(right_forward_state)

    # Action: Go right backward (reverse quarter-circle turn)
    right_backward_state = perform_turn(state, TURN_RADIUS, left=False, forward=False)
    successors.append(right_backward_state)

    return successors

def perform_turn(state, radius, left=True, forward=True):
    # Determine the center of rotation
    theta_rad = math.radians(state.theta)
    dir_multiplier = 1 if forward else -1
    turn_angle = dir_multiplier * 90  # Quarter-circle turn

    if left:
        center_x = state.x - radius * math.sin(theta_rad)
        center_y = state.y + radius * math.cos(theta_rad)
        delta_theta = turn_angle
    else:
        center_x = state.x + radius * math.sin(theta_rad)
        center_y = state.y - radius * math.cos(theta_rad)
        delta_theta = -turn_angle

    # Calculate the new orientation
    new_theta = (state.theta + delta_theta) % 360

    # Calculate the new position after the turn
    angle_start = math.atan2(state.y - center_y, state.x - center_x)
    angle_end = angle_start + math.radians(delta_theta)
    new_x = center_x + radius * math.cos(angle_end)
    new_y = center_y + radius * math.sin(angle_end)

    return State(new_x, new_y, new_theta)

def is_valid(state, field, parent_state=None):
    # If there's no parent state (start state), just check the current position
    if parent_state is None:
        return check_collision(state, field)

    # Determine if the movement is a turn or straight
    angle_diff = abs((state.theta - parent_state.theta + 180) % 360 - 180)
    if angle_diff == 0:
        # Straight movement
        return check_straight_path(parent_state, state, field)
    else:
        # Turning movement
        return check_turning_path(parent_state, state, field)

def check_straight_path(start_state, end_state, field):
    # Number of samples along the path
    distance = math.hypot(end_state.x - start_state.x, end_state.y - start_state.y)
    num_samples = max(1, int(distance / SAMPLE_DISTANCE))

    for i in range(num_samples + 1):
        t = i / num_samples
        x = start_state.x + t * (end_state.x - start_state.x)
        y = start_state.y + t * (end_state.y - start_state.y)
        theta = start_state.theta
        temp_state = State(x, y, theta)
        if not check_collision(temp_state, field):
            return False
    return True

def check_turning_path(start_state, end_state, field):
    # Reconstruct the turn parameters
    radius = TURN_RADIUS
    delta_theta = (end_state.theta - start_state.theta + 360) % 360
    if delta_theta == 0:
        delta_theta = -((start_state.theta - end_state.theta + 360) % 360)
    theta_rad = math.radians(start_state.theta)
    turn_direction = 'left' if delta_theta > 0 else 'right'

    # Determine the center of rotation
    if turn_direction == 'left':
        center_x = start_state.x - radius * math.sin(theta_rad)
        center_y = start_state.y + radius * math.cos(theta_rad)
    else:
        center_x = start_state.x + radius * math.sin(theta_rad)
        center_y = start_state.y - radius * math.cos(theta_rad)

    # Number of samples along the arc
    arc_length = math.radians(abs(delta_theta)) * radius
    num_samples = max(1, int(arc_length / SAMPLE_DISTANCE))

    for i in range(num_samples + 1):
        t = i / num_samples
        angle = math.radians(start_state.theta) + t * math.radians(delta_theta)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        theta = (start_state.theta + t * delta_theta) % 360
        temp_state = State(x, y, theta)
        if not check_collision(temp_state, field):
            return False
    return True

def check_collision(state, field):
    # Check if the robot is within the field boundaries
    if not (0 <= state.x <= FIELD_W and 0 <= state.y <= FIELD_H):
        return False

    # Temporarily update the robot's position and orientation
    robot = field.get_robot()
    original_pos = robot.get_center_pos()
    original_theta = robot.get_theta()

    robot.set_center_pos([state.x, state.y])
    robot.set_theta(state.theta)

    # Get robot corners
    robot_corners = robot.get_pos()

    # Check collision with obstacles
    collision = False
    for obstacle in field.get_obstacles():
        obstacle_corners = obstacle.get_pos()
        if polygons_intersect(robot_corners, obstacle_corners):
            collision = True
            break

    # Reset robot's position and orientation
    robot.set_center_pos(original_pos)
    robot.set_theta(original_theta)

    return not collision

def movement_cost(current_state, next_state):
    angle_diff = abs((next_state.theta - current_state.theta + 180) % 360 - 180)

    if angle_diff == 0:
        # Straight movement
        distance = math.hypot(next_state.x - current_state.x, next_state.y - current_state.y)
        return distance
    elif angle_diff == 90:
        # Quarter-circle turn
        arc_length = (math.pi * TURN_RADIUS) / 2  # Quarter of the circumference
        return arc_length
    else:
        # For backward turns or invalid movements
        return float('inf')  # Penalize invalid or unsupported movements

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