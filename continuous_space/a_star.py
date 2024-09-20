import math
import heapq
from itertools import permutations
from functools import cache  # Use functools.lru_cache for Python versions < 3.9
from entities import Robot, Obstacle, Field
from constants import *

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

    def __hash__(self):
        return hash((round(self.x, 1), round(self.y, 1), round(self.theta, 1)))

    def __eq__(self, other):
        return (round(self.x, 1), round(self.y, 1), round(self.theta, 1)) == \
               (round(other.x, 1), round(other.y, 1), round(other.theta, 1))

def a_star_search(field: Field, target_pos: list, target_theta: float):
    robot = field.get_robot()
    start_x, start_y = robot.get_center_pos()
    start_theta = robot.get_theta()

    target_x, target_y = target_pos
    target_theta = target_theta % 360

    # Precompute obstacle projections
    precompute_obstacle_data(field)

    open_list = []
    closed_set = set()

    start_state = State(start_x, start_y, start_theta)
    heapq.heappush(open_list, start_state)

    while open_list:
        current_state = heapq.heappop(open_list)

        # Goal check
        if is_goal(current_state, target_x, target_y, target_theta):
            return reconstruct_path(current_state)

        if current_state in closed_set:
            continue
        closed_set.add(current_state)

        # Generate successors
        successors = get_successors(current_state, field)

        for next_state in successors:
            if next_state in closed_set:
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
    pos_threshold = GOAL_THRESHOLD   # Position tolerance
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
    robot = field.get_robot()
    original_pos = robot.get_center_pos()
    original_theta = robot.get_theta()

    # Get robot's corners at start position
    robot.set_center_pos([start_state.x, start_state.y])
    robot.set_theta(start_state.theta)
    start_corners = robot.get_pos()

    # Get robot's corners at end position
    robot.set_center_pos([end_state.x, end_state.y])
    robot.set_theta(end_state.theta)
    end_corners = robot.get_pos()

    # Collect all corners
    all_corners = start_corners + end_corners
    all_corners = tuple(map(tuple, all_corners))  # Make hashable

    # Compute convex hull
    swept_area = compute_convex_hull(all_corners)

    # Reset robot's position and orientation
    robot.set_center_pos(original_pos)
    robot.set_theta(original_theta)

    # Build bounding box for the swept area
    min_x = min(point[0] for point in swept_area)
    max_x = max(point[0] for point in swept_area)
    min_y = min(point[1] for point in swept_area)
    max_y = max(point[1] for point in swept_area)

    # Filter obstacles that are within the bounding box
    nearby_obstacles = []
    for obstacle in field.get_obstacles():
        obs_min_x = obstacle.min_x
        obs_max_x = obstacle.max_x
        obs_min_y = obstacle.min_y
        obs_max_y = obstacle.max_y

        if (obs_max_x >= min_x and obs_min_x <= max_x and
            obs_max_y >= min_y and obs_min_y <= max_y):
            nearby_obstacles.append(obstacle)

    # Check collision with nearby obstacles
    for obstacle in nearby_obstacles:
        if polygons_intersect_optimized(tuple(swept_area), obstacle):
            return False

    # Also check boundaries
    for point in swept_area:
        if not (0 <= point[0] <= FIELD_W and 0 <= point[1] <= FIELD_H):
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
    robot_corners = tuple(map(tuple, robot_corners))  # Make hashable

    # Build bounding box for the robot
    min_x = min(point[0] for point in robot_corners)
    max_x = max(point[0] for point in robot_corners)
    min_y = min(point[1] for point in robot_corners)
    max_y = max(point[1] for point in robot_corners)

    # Filter obstacles that are within the bounding box
    nearby_obstacles = []
    for obstacle in field.get_obstacles():
        obs_min_x = obstacle.min_x
        obs_max_x = obstacle.max_x
        obs_min_y = obstacle.min_y
        obs_max_y = obstacle.max_y

        if (obs_max_x >= min_x and obs_min_x <= max_x and
            obs_max_y >= min_y and obs_min_y <= max_y):
            nearby_obstacles.append(obstacle)

    # Check collision with nearby obstacles
    for obstacle in nearby_obstacles:
        if polygons_intersect_optimized(robot_corners, obstacle):
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
        cost = distance
        if next_state.action in ['TURN_LEFT_FORWARD', 'TURN_LEFT_BACKWARD',
                                 'TURN_RIGHT_FORWARD', 'TURN_RIGHT_BACKWARD']:
            # Add rotation cost
            cost += ROTATION_COST
        return cost
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
    num_rotations = angle_diff / 90
    rotation_cost = num_rotations * ROTATION_COST
    return distance + rotation_cost

def precompute_obstacle_data(field):
    """
    Precomputes axes and projections for all obstacles.
    """
    for obstacle in field.get_obstacles():
        obstacle_corners = tuple(map(tuple, obstacle.get_pos()))
        obstacle.axes = get_axes(obstacle_corners)
        obstacle.projections = {}
        for axis in obstacle.axes:
            axis_key = axis_key_func(axis)
            projection = project_polygon(axis, obstacle_corners)
            obstacle.projections[axis_key] = projection

        # Precompute bounding box for obstacle
        xs = [point[0] for point in obstacle_corners]
        ys = [point[1] for point in obstacle_corners]
        obstacle.min_x = min(xs)
        obstacle.max_x = max(xs)
        obstacle.min_y = min(ys)
        obstacle.max_y = max(ys)

def axis_key_func(axis):
    scale_factor = 1e5
    return (int(round(axis[0] * scale_factor)), int(round(axis[1] * scale_factor)))

def polygons_intersect_optimized(poly1, obstacle):
    """
    Optimized polygon intersection using precomputed obstacle data.
    """
    # poly1 is the robot's polygon or swept area (tuple of points)
    axes = get_axes(poly1)

    # Add obstacle's precomputed axes
    axes += obstacle.axes

    for axis in axes:
        key = axis_key_func(axis)
        proj1 = project_polygon(axis, poly1)
        # Use precomputed projection for the obstacle
        proj2 = obstacle.projections.get(key)
        if proj2 is None:
            # Should not happen if precompute_obstacle_data is correct
            obstacle_corners = tuple(map(tuple, obstacle.get_pos()))
            proj2 = project_polygon(axis, obstacle_corners)
            obstacle.projections[key] = proj2  # Cache it

        if proj1[1] < proj2[0] or proj2[1] < proj1[0]:
            return False
    return True

@cache
def get_axes(polygon):
    """
    Computes the axes (normals to edges) for a polygon.
    """
    axes = []
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        edge = [p2[0] - p1[0], p2[1] - p1[1]]
        normal = [-edge[1], edge[0]]
        length = math.hypot(normal[0], normal[1])
        normalized_axis = (normal[0] / length, normal[1] / length)
        axes.append(normalized_axis)
    return tuple(axes)

@cache
def project_polygon(axis, polygon):
    """
    Projects a polygon onto an axis.
    """
    dots = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
    return (min(dots), max(dots))

@cache
def compute_convex_hull(points):
    """
    Computes the convex hull of a set of 2D points using the monotone chain algorithm.
    Returns the convex hull as a tuple of points in counter-clockwise order.
    """
    # Convert points to tuples and eliminate duplicates
    points = sorted(set(points))

    if len(points) <= 1:
        return points

    # Build the lower and upper parts of the convex hull
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenate lower and upper to get full hull
    # Last point of each list is omitted because it is repeated
    convex_hull = lower[:-1] + upper[:-1]

    return tuple(convex_hull)

@cache
def cross(o, a, b):
    """
    2D cross product of OA and OB vectors, i.e., z-component of (OA x OB).
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def tuple_map(points):
    """
    Converts a list of points to a tuple of tuples to make them hashable.
    """
    return tuple(map(tuple, points))
