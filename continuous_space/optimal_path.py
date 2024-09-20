# optimal_path.py

import math
import itertools
from a_star import a_star_search
from entities import Field, Robot, Obstacle
from constants import FIELD_W, FIELD_H, TURN_RADIUS, SAMPLE_DISTANCE, ACTIONS, MOVE_STEP

def optimal_path(field: Field, targets: list):
    """
    Finds the optimal order to visit all targets and returns the combined path.

    :param field: The Field object containing the robot and obstacles.
    :param targets: A list of tuples, each containing a target position [x, y] and orientation theta.
                    Example: [([100, 100], 90), ([150, 150], 0)]
    :return: A list representing the combined path to visit all targets in the optimal order.
             Each element in the list is a tuple: (x, y, theta, action)
    """
    if not targets:
        return []

    # Generate all possible permutations of the target order
    permutations = list(itertools.permutations(targets))
    
    optimal_combined_path = None
    minimal_total_cost = float('inf')

    # Iterate through each permutation to find the one with the minimal total cost
    for perm in permutations:
        combined_path = []
        total_cost = 0.0
        temp_field = clone_field(field)
        temp_robot = temp_field.get_robot()
        temp_robot_pos = temp_robot.get_center_pos()
        temp_robot_theta = temp_robot.get_theta()

        success = True  # Flag to check if all segments are reachable

        for idx, target in enumerate(perm):
            target_pos, target_theta = target
            path_segment = a_star_search(temp_field, target_pos, target_theta)

            if path_segment is None:
                # If any segment is unreachable, skip this permutation
                success = False
                break

            if idx > 0:
                # Skip the first state to avoid duplication of the last state from the previous segment
                path_segment = path_segment[1:]

            # Calculate the cost of this segment
            segment_cost = calculate_path_cost(path_segment)
            total_cost += segment_cost

            # Append the segment to the combined path
            combined_path.extend(path_segment)
            combined_path.extend([(0, 0, 0, 'SNAP')])  # Add a snapshot action between segments

            # Update the robot's position and orientation for the next segment
            last_state = path_segment[-1]
            temp_robot.set_center_pos([last_state[0], last_state[1]])
            temp_robot.set_theta(last_state[2])

        if success and total_cost < minimal_total_cost:
            minimal_total_cost = total_cost
            optimal_combined_path = combined_path

    return optimal_combined_path

def clone_field(field: Field) -> Field:
    """
    Creates a deep copy of the field, including robot and obstacles.

    :param field: The original Field object.
    :return: A new Field object with the same robot and obstacles.
    """
    new_field = Field(r=100, obs=[])
    original_robot = field.get_robot()
    new_robot = Robot(original_robot.get_center_pos().copy(), original_robot.get_theta())
    new_field.set_robot(new_robot)

    for obstacle in field.get_obstacles():
        new_obstacle = Obstacle(obstacle.get_center_pos().copy(), obstacle.get_theta())
        new_field.add_obstacle(new_obstacle)

    return new_field

def calculate_path_cost(path: list) -> float:
    """
    Calculates the total movement cost of a given path.

    :param path: A list of tuples representing the path.
                 Each tuple contains (x, y, theta, action).
    :return: The total cost of the path.
    """
    total_cost = 0.0
    for i in range(1, len(path)):
        prev_state = path[i - 1]
        curr_state = path[i]
        total_cost += movement_cost_custom(prev_state, curr_state)
    return total_cost

def movement_cost_custom(prev_state, curr_state) -> float:
    """
    Custom movement cost calculation between two consecutive states.

    :param prev_state: The previous state tuple (x, y, theta, action).
    :param curr_state: The current state tuple (x, y, theta, action).
    :return: The cost associated with moving from prev_state to curr_state.
    """
    action = curr_state[3]
    if action in ['GO_FORWARD', 'GO_BACKWARD']:
        distance = math.hypot(curr_state[0] - prev_state[0], curr_state[1] - prev_state[1])
        return distance
    elif action.startswith('TURN_'):
        # Quarter-circle turn cost
        return (math.pi * TURN_RADIUS) / 2
    else:
        return float('inf')  # Undefined actions are penalized heavily
