import cProfile

from optimal_path import *
# import math
# import itertools
from a_star import a_star_search
from entities import Field, Robot, Obstacle
from constants import FIELD_W, FIELD_H, OBSERVATION_DISTANCE, TURN_RADIUS, SAMPLE_DISTANCE, ACTIONS, MOVE_STEP, Direction
import json

def map_to_inst(json_input):
    
    """
    Sample Input:
    {
        "obstacles":
        [
            {
                "x": 0,
                "y": 9,
                "id": 1,
                "d": 2
            },
            ...,
            {
                "x": 19,
                "y": 14,
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
    robot = Robot([10, 10], Direction.UP)
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
        field.add_obstacle(Obstacle([obstacle["x"], obstacle["y"]], dir))

    targets = []

    obstacles = field.get_obstacles()

    for obstacle in obstacles:
        (x, y) = obstacle.get_center_pos()
        direction = obstacle.get_theta()
        
        if direction == Direction.UP:
            targets.append(((x, y + OBSERVATION_DISTANCE + 1), Direction.DOWN))
        elif direction == Direction.DOWN:
            targets.append(((x, y - OBSERVATION_DISTANCE - 1), Direction.UP))
        elif direction == Direction.LEFT:
            targets.append(((x - OBSERVATION_DISTANCE - 1, y), Direction.RIGHT))
        elif direction == Direction.RIGHT:
            targets.append(((x + OBSERVATION_DISTANCE + 1, y), Direction.LEFT))

    # Get the optimal path
    path = optimal_path(field, targets)
    print("=== path ===\n", path)

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

        if temp_commands and temp_commands[-1]["action"] == action:
            temp_commands[-1]["distance"] += MOVE_STEP
        else:
            distmove = MOVE_STEP if action in ["GO_FORWARD""GO_BACKWARD"] else 0
            temp_commands.append({
                "action": action,
                "distance": MOVE_STEP
            })

    commands = []
    for command in temp_commands:
        act = ""
        match command["action"]:
            case "GO_FORWARD": act = "FW"
            case "GO_BACKWARD": act = "BW"
            case "TURN_LEFT_FORWARD": act = "FL"
            case "TURN_RIGHT_FORWARD": act = "FR"
            case "TURN_LEFT_BACKWARD": act = "BL"
            case "TURN_RIGHT_BACKWARD": act = "BR"
            case "SNAP": act = "SNAP"

        if act == "SNAP":
            commands.append("SNAP")
        else:
            commands.append(f"{act}{str(command['distance']).zfill(2)}")



    # Create the output dictionary
    output = {
        "data": {
            "commands": commands,
            # "distance": total_distance,
            "path": path
        },
        "error": None
    }

    return json.dumps(output)


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
                "x": 150,
                "y": 50,
                "id": 2,
                "d": 4
            },
            {
                "x": 60,
                "y": 60,
                "id": 3,
                "d": 2
            },
            {
                "x": 100,
                "y": 100,
                "id": 4,
                "d": 2
            }
        ]
    }
    """

    # json_input = """
    # {
    #     "obstacles":
    #     [
    #         {
    #             "x": 100,
    #             "y": 100,
    #             "id": 1,
    #             "d": 0
    #         },
    #         {
    #             "x": 150,
    #             "y": 50,
    #             "id": 2,
    #             "d": 4
    #         }
    #     ]
    # }
    # """

    input = json.loads(json_input)

    if 0:
        

        print("=== input === \n", input)

        output = map_to_inst(input)
        output = json.loads(output)
        # Call the function and print the result
        print("=== output ===\n", output)

    if 1:
        cProfile.run('map_to_inst(input)')
