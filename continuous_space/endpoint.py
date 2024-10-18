import cProfile

from optimal_path_2 import *
# import math
# import itertools
from a_star import a_star_search
from entities import Field, Robot, Obstacle
from constants import FIELD_W, FIELD_H, OBSERVATION_DISTANCE, TURN_RADIUS, SAMPLE_DISTANCE, ACTIONS, MOVE_STEP, Direction
import json
from flask import jsonify
from main import visualize_path
import threading

def map_to_inst(json_input: json):
    
    """
    Sample Input:
    {
        "obstacles":
        [
            {
                "x": 0,
                "y": 90, # Make sure to times 10
                "id": 1,
                "d": 2
            },
            ...,
            {
                "x": 190,
                "y": 140,
                "id": 5,
                "d": 6
            }
        ]
    }

    Sample Output:
    {
        "data": {
            "commands": [
                "FR00",
                "FW10",
                "SNAP1",
                "FR00",
                "BW50",
                "FL00",
                "FW60",
                "SNAP2",
                ...,
                "FIN"
            ],
            "distance": 46.0,
            "path": [
                {
                    "d": 0,
                    "s": -1,
                    "x": 1,
                    "y": 1
                },
                {
                    "d": 2,
                    "s": -1,
                    "x": 5,
                    "y": 3
                },
                ...,
                {
                    "d": 2,
                    "s": -1,
                    "x": 6,
                    "y": 9
                },
            ]
        },
        "error": null
    }

    """

    # Parse the JSON input
    data = json_input

    # Extract obstacles
    obstacles = data["obstacles"]

    # Create Field and Robot instances
    field = Field(r=100, obs = [])
    robot = Robot([11, 13], Direction.UP)
    field.set_robot(robot)

    for obstacle in obstacles:
        dir = Direction.UP
        if obstacle["d"] == 0:
            dir = Direction.UP
        elif obstacle["d"] == 2:
            dir = Direction.RIGHT
        elif obstacle["d"] == 4:
            dir = Direction.DOWN
        elif obstacle["d"] == 6:
            dir = Direction.LEFT
        obs = Obstacle([obstacle["x"], obstacle["y"]], dir)
        obs.set_id(obstacle["id"])
        field.add_obstacle(obs)

    targets = []

    obstacles = field.get_obstacles()

    for obstacle in obstacles:
        (x, y) = obstacle.get_center_pos()
        direction = obstacle.get_theta()
        
        if direction == Direction.UP:
            targets.append(((x, y + OBSERVATION_DISTANCE + 1), Direction.DOWN, obstacle.get_id()))
        elif direction == Direction.DOWN:
            targets.append(((x, y - OBSERVATION_DISTANCE - 1), Direction.UP, obstacle.get_id()))
        elif direction == Direction.LEFT:
            targets.append(((x - OBSERVATION_DISTANCE - 1, y), Direction.RIGHT, obstacle.get_id()))
        elif direction == Direction.RIGHT:
            targets.append(((x + OBSERVATION_DISTANCE + 1, y), Direction.LEFT, obstacle.get_id()))

    print(targets)

    # Get the optimal path
    path = optimal_path(field, targets)
    print(path)
    # if path is not None:
    #     path = list(filter(lambda x: x[3] != 'SNAP', path))

    if path:
        print("Optimal path found.\n")
        # Compute and print movement commands
        # Visualize the path using Pygame, passing the list of targets
        # --------------------uncomment here to visualize the path--------------------
        # thread = threading.Thread(target=visualize_path, args=(field, path, targets))
        # thread.start()
        # ----------------------------------------------------------------------------
    else:
        print("No path found.")

    if path == None:
        return jsonify({"error": "path not found"}), 404

    temp_commands = [] # [{"action": action, "distance": distance}]
    path_results = []

    for p in path[1:]:
        x, y = p[0], p[1]
        direction = p[2]
        action = p[3]
        path_results.append({
            "x": x,
            "y": y,
            "d": direction,
            "s": action
        })

        if temp_commands and temp_commands[-1]["action"] == action and action in ["GO_FORWARD", "GO_BACKWARD"]:
            temp_commands[-1]["distance"] += MOVE_STEP
        elif temp_commands and action in ["SNAP"]: 
            temp_commands.append({
                "action": "SNAP",
                "x": x,
                "y": y
            })
        else: 
            distmove = MOVE_STEP if action in ["GO_FORWARD", "GO_BACKWARD"] else DEGREE_90
            temp_commands.append({
            "action": action,
            "distance": distmove
            })

    commands = []
    for command in temp_commands:
        act = ""
        if command["action"] == "GO_FORWARD":
            act = "FW"
        elif command["action"] == "GO_BACKWARD":
            act = "BW"
        elif command["action"] == "TURN_LEFT_FORWARD":
            act = "FL"
            # act = "A0"
        elif command["action"] == "TURN_RIGHT_FORWARD":
            act = "FR"
            # act = "C0"
        elif command["action"] == "TURN_LEFT_BACKWARD":
            act = "BL"
        elif command["action"] == "TURN_RIGHT_BACKWARD":
            act = "BR"
        else:
        # elif command["action"] == "SNAP":
            act = "SNAP"

        if act == "SNAP":
            for target in targets:
                if abs(int(command['x']) - target[0][0]) + abs(int(command['y'] - target[0][1])) < 5:
                    target_id = str(target[2])
                    
            text = "SNAP" + target_id
            commands.append(text)
        else:
            commands.append(f"{act}{str(command['distance']).zfill(3)}")
            
    for element in path:
        new_path = []
        new_path.append(element[0])
        new_path.append(element[1])
        if element[2] == 90:
            new_path.append(0)
        elif element[2] == 0:
            new_path.append(2)
        elif element[2] == 270:
            new_path.append(4)
        elif element[2] == 180:
            new_path.append(6)

    # Create the output dictionary
    output = {
        "data": {
            "commands": commands,
            # "distance": total_distance,
            "path": new_path
        },
        "error": None
    }

    print("before send: ")
    print(output)

    return jsonify(output), 200


# # Add obstacles to the field
# obstacle1 = Obstacle([100, 100], Direction.UP)
# field.add_obstacle(obstacle1)

# obstacle2 = Obstacle([150, 50], Direction.DOWN)
# field.add_obstacle(obstacle2)

# obstacle3 = Obstacle([60, 60], Direction.RIGHT)
# field.add_obstacle(obstacle3)

# obstacle4 = Obstacle([10, 100], Direction.RIGHT)



if __name__ == "__main__":
    # Sample input
    # json_input = obs_3_input

    json_input = """
    {
        "obstacles":
        [
            {
                "x": 100,
                "y": 100,
                "id": 1,
                "d": 0
            },
            {
                "x": 180,
                "y": 180,
                "id": 2,
                "d": 4
            }
        ]
    }
    """

    input = json.loads(json_input)

    if 1:
        

        print("=== input === \n", input)

        output = map_to_inst(input)
        output = json.loads(output)
        # Call the function and print the result
        print("=== output ===\n", output)

    if 0:
        cProfile.run('map_to_inst(input)')
