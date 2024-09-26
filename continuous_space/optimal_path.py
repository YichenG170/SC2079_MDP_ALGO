# optimal_path.py

import math
from a_star import a_star_search
from entities import Field, Robot, Obstacle
from constants import TURN_RADIUS

def optimal_path(field: Field, targets: list):
    """
    Finds an efficient path to visit all targets using the Nearest Neighbor heuristic.

    :param field: The Field object containing the robot and obstacles.
    :param targets: A list of tuples, each containing a target position [x, y] and orientation theta.
                    Example: [([100, 100], 90), ([150, 150], 0)]
    :return: A list representing the combined path to visit all targets.
             Each element in the list is a tuple: (x, y, theta, action)
    """
    if not targets:
        return []

    # Include the robot's starting position as a node
    start_pos = field.get_robot().get_center_pos()
    start_theta = field.get_robot().get_theta()
    nodes = [({'pos': start_pos, 'theta': start_theta, 'id': 'start'})]
    for idx, target in enumerate(targets):
        nodes.append({'pos': target[0], 'theta': target[1], 'id': idx})

    # Precompute shortest paths and costs between all pairs of nodes
    pairwise_paths = {}
    pairwise_costs = {}
    for i in range(len(nodes)):
        for j in range(len(nodes)):
            if i != j:
                key = (nodes[i]['id'], nodes[j]['id'])
                # Clone the field to avoid modifying the original
                temp_field = clone_field(field)
                temp_field.get_robot().set_center_pos(nodes[i]['pos'])
                temp_field.get_robot().set_theta(nodes[i]['theta'])
                path = a_star_search(temp_field, nodes[j]['pos'], nodes[j]['theta'])
                if path is not None:
                    cost = calculate_path_cost(path)
                    pairwise_paths[key] = path
                    pairwise_costs[key] = cost
                else:
                    # If no path exists, set cost to infinity
                    pairwise_costs[key] = float('inf')

    # Apply Nearest Neighbor heuristic
    unvisited = set([node['id'] for node in nodes if node['id'] != 'start'])
    current_id = 'start'
    combined_path = []
    total_cost = 0.0

    while unvisited:
        # Find the nearest unvisited node
        nearest_node = None
        min_cost = float('inf')
        for node_id in unvisited:
            cost = pairwise_costs.get((current_id, node_id), float('inf'))
            if cost < min_cost:
                min_cost = cost
                nearest_node = node_id

        if nearest_node is None or min_cost == float('inf'):
            # No reachable unvisited nodes
            print("Some targets are unreachable.")
            return None

        # Append the path to the combined path
        path_segment = pairwise_paths[(current_id, nearest_node)]
        if combined_path:
            # Skip the first state to avoid duplication
            path_segment = path_segment[1:]

        combined_path.extend(path_segment)
        combined_path.append((0, 0, 0, 'SNAP'))

        # Update total cost and current position
        total_cost += min_cost
        current_id = nearest_node
        unvisited.remove(nearest_node)

    print(f"Total path cost: {total_cost}")
    return combined_path

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
