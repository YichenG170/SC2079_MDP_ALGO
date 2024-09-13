# main.py

import math
import pygame
from optimal_path import optimal_path
from entities import Field, Robot, Obstacle
from constants import *

def main():
    # Initialize the field with a radius and no initial obstacles
    field = Field(r=100, obs=[])

    # Initialize the robot at position (50, 50) facing UP (90 degrees)
    robot = Robot([50, 50], Direction.UP)
    field.set_robot(robot)

    # Add obstacles to the field
    obstacle1 = Obstacle([100, 100], Direction.UP)
    field.add_obstacle(obstacle1)

    obstacle2 = Obstacle([150, 50], Direction.UP)
    field.add_obstacle(obstacle2)

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

    # Find the optimal path covering all targets
    path = optimal_path(field, targets)

    if path:
        print("Optimal path found.\n")
        # Compute and print movement commands
        compute_and_print_commands(path)
        # Visualize the path using Pygame, passing the list of targets
        visualize_path(field, path, targets)
    else:
        print("No path found.")

def compute_and_print_commands(path):
    print("Commands:")
    for i in range(1, len(path)):
        prev_state = path[i - 1]
        curr_state = path[i]

        action = curr_state[3]  # The action taken to reach the current state

        if action == 'GO_FORWARD':
            distance = math.hypot(curr_state[0] - prev_state[0], curr_state[1] - prev_state[1])
            print(f"MOVE: {distance:.2f} units forward")
        elif action == 'GO_BACKWARD':
            distance = math.hypot(curr_state[0] - prev_state[0], curr_state[1] - prev_state[1])
            print(f"MOVE: {distance:.2f} units backward")
        elif action.startswith('TURN_'):
            parts = action.split('_')
            turn_direction = parts[1].lower()
            movement_direction = parts[2].lower()
            print(f"TURN: Quarter-circle {turn_direction} {movement_direction}")
        else:
            print(f"UNKNOWN ACTION: {action}")
    print("\n")

def visualize_path(field, path, targets):
    # Initialize Pygame
    pygame.init()
    scale_factor = 3  # Enlarge the field by this factor
    window_size = (int(FIELD_W * scale_factor), int(FIELD_H * scale_factor))
    screen = pygame.display.set_mode(window_size)
    pygame.display.set_caption("Robot Path Visualization")
    # Reduced frame rate to slow down the animation
    clock = pygame.time.Clock()

    # Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    BLUE = (0, 0, 255)
    LIGHT_GREY = (200, 200, 200)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)

    # Load obstacles
    obstacles = field.get_obstacles()

    # Main loop variables
    running = True
    index = 0
    path_length = len(path)
    path_positions = []    # Store positions to draw the path
    robot_positions = []   # Positions for robot animation

    # Target tracking
    remaining_targets = targets.copy()
    reached_targets = []

    # Direction arrow tracking
    accumulated_distance = 0.0
    ARROW_DISTANCE = 30.0  # Increased from 15.0 to 30.0 units

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if index < path_length - 1:
            # Interpolate between the current and next state
            current_state = path[index]
            next_state = path[index + 1]

            interpolated_positions = interpolate_states(current_state, next_state)
            path_positions.extend(interpolated_positions)
            robot_positions.extend(interpolated_positions)
            index += 1

        # Clear screen
        screen.fill(WHITE)

        # Draw obstacles
        for obstacle in obstacles:
            draw_entity(screen, obstacle, BLACK, scale_factor)

        # Draw targets as green points initially
        for target in targets:
            target_pos, _ = target
            draw_target(screen, target_pos, GREEN, scale_factor)

        # Draw path
        if len(path_positions) >= 2:
            scaled_path = [((x * scale_factor), (FIELD_H - y) * scale_factor) for x, y, _ in path_positions]
            pygame.draw.lines(screen, LIGHT_GREY, False, scaled_path, 2)

        # Draw direction arrows every ARROW_DISTANCE units
        if len(path_positions) >= 2:
            for i in range(1, len(path_positions)):
                prev = path_positions[i - 1]
                curr = path_positions[i]
                segment_distance = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
                accumulated_distance += segment_distance

                if accumulated_distance >= ARROW_DISTANCE:
                    # Draw arrow at current position indicating direction
                    x, y, theta = curr
                    draw_direction_arrow(screen, x, y, theta, scale_factor)
                    accumulated_distance = 0.0  # Reset accumulator

        # Draw robot
        if robot_positions:
            current_robot_pos = robot_positions.pop(0)
            robot_x, robot_y, robot_theta = current_robot_pos[:3]
            robot = field.get_robot()
            robot.set_center_pos([robot_x, robot_y])
            robot.set_theta(robot_theta)
            draw_entity(screen, robot, BLUE, scale_factor)

            # Check if a target is reached
            for target in remaining_targets.copy():
                target_pos, target_theta = target
                distance = math.hypot(robot_x - target_pos[0], robot_y - target_pos[1])
                angle_diff = abs((robot_theta - target_theta + 180) % 360 - 180)
                if distance <= 1.0 and angle_diff <= 5.0:
                    # Target reached
                    reached_targets.append(target)
                    remaining_targets.remove(target)
                    print(f"Reached target at {target_pos} with orientation {target_theta} degrees.")

        # Draw reached targets as red points
        for target in reached_targets:
            target_pos, _ = target
            draw_target(screen, target_pos, RED, scale_factor)

        # Update display
        pygame.display.flip()
        # Set frame rate to 15 FPS to slow down the animation
        clock.tick(15)  # Reduced from 60 to 15

    pygame.quit()

def draw_entity(screen, entity, color, scale_factor):
    # Get entity corners
    corners = entity.get_pos()
    scaled_corners = [((x * scale_factor), (FIELD_H - y) * scale_factor) for x, y in corners]  # Adjust y-axis

    # Draw entity as a polygon
    pygame.draw.polygon(screen, color, scaled_corners)

    # Draw direction arrow
    draw_direction_arrow_entity(screen, entity, color=(255, 255, 255), scale_factor=scale_factor)

def draw_direction_arrow_entity(screen, entity, color, scale_factor):
    # Draw an arrow inside the entity to indicate direction
    x, y = entity.get_center_pos()
    x *= scale_factor
    y = (FIELD_H - y) * scale_factor  # Adjust y-axis

    # Arrow parameters
    entity_width = entity.get_width() * scale_factor
    entity_height = entity.get_height() * scale_factor
    arrow_length = min(entity_width, entity_height) * 0.4
    arrow_width = arrow_length * 0.2

    # Calculate arrow endpoint based on orientation
    theta_rad = math.radians(entity.get_theta())
    end_x = x + arrow_length * math.cos(-theta_rad)
    end_y = y + arrow_length * math.sin(-theta_rad)

    # Draw arrow shaft
    pygame.draw.line(screen, color, (x, y), (end_x, end_y), max(int(2 * scale_factor), 1))

    # Calculate arrowhead points
    angle = math.atan2(end_y - y, end_x - x)
    left_wing_angle = angle + math.radians(150)
    right_wing_angle = angle - math.radians(150)
    left_wing = (
        end_x + arrow_width * math.cos(left_wing_angle),
        end_y + arrow_width * math.sin(left_wing_angle),
    )
    right_wing = (
        end_x + arrow_width * math.cos(right_wing_angle),
        end_y + arrow_width * math.sin(right_wing_angle),
    )

    # Draw arrowhead
    pygame.draw.polygon(screen, color, [left_wing, (end_x, end_y), right_wing])

def draw_target(screen, target_pos, color, scale_factor):
    """
    Draws a point at the target position.
    :param screen: Pygame screen.
    :param target_pos: [x, y] position of the target.
    :param color: Color of the point.
    :param scale_factor: Scaling factor for visualization.
    """
    x, y = target_pos
    scaled_x = x * scale_factor
    scaled_y = (FIELD_H - y) * scale_factor  # Adjust y-axis
    pygame.draw.circle(screen, color, (int(scaled_x), int(scaled_y)), 5)

def draw_direction_arrow(screen, x, y, theta, scale_factor):
    """
    Draws a direction arrow at the specified position and orientation.
    :param screen: Pygame screen.
    :param x: x-coordinate.
    :param y: y-coordinate.
    :param theta: Orientation in degrees.
    :param scale_factor: Scaling factor for visualization.
    """
    # Arrow parameters
    arrow_length = 20  # Length in pixels
    arrow_width = 10   # Width of the arrowhead

    theta_rad = math.radians(theta)
    end_x = x * scale_factor + arrow_length * math.cos(-theta_rad)
    end_y = (FIELD_H - y) * scale_factor + arrow_length * math.sin(-theta_rad)

    # Draw arrow shaft
    pygame.draw.line(screen, GREEN, (x * scale_factor, (FIELD_H - y) * scale_factor), (end_x, end_y), 2)

    # Calculate arrowhead points
    angle = math.atan2(end_y - (FIELD_H - y) * scale_factor, end_x - x * scale_factor)
    left_wing_angle = angle + math.radians(150)
    right_wing_angle = angle - math.radians(150)
    left_wing = (
        end_x + arrow_width * math.cos(left_wing_angle),
        end_y + arrow_width * math.sin(left_wing_angle),
    )
    right_wing = (
        end_x + arrow_width * math.cos(right_wing_angle),
        end_y + arrow_width * math.sin(right_wing_angle),
    )

    # Draw arrowhead
    pygame.draw.polygon(screen, GREEN, [left_wing, (end_x, end_y), right_wing])

def interpolate_states(start_state, end_state):
    """
    Interpolates between two states to generate intermediate positions for smooth animation.
    """
    positions = []
    action = end_state[3]

    if action in ['GO_FORWARD', 'GO_BACKWARD']:
        # Straight movement
        distance = math.hypot(end_state[0] - start_state[0], end_state[1] - start_state[1])
        num_samples = max(1, int(distance / SAMPLE_DISTANCE))
        for i in range(1, num_samples + 1):
            t = i / num_samples
            x = start_state[0] + t * (end_state[0] - start_state[0])
            y = start_state[1] + t * (end_state[1] - start_state[1])
            theta = start_state[2]
            positions.append((x, y, theta))
    elif action.startswith('TURN_'):
        # Turning movement (quarter-circle)
        parts = action.split('_')
        turn_direction = parts[1].lower()
        movement_direction = parts[2].lower()

        theta_rad = math.radians(start_state[2])

        # Determine center of rotation
        if turn_direction == 'left':
            center_x = start_state[0] - TURN_RADIUS * math.sin(theta_rad)
            center_y = start_state[1] + TURN_RADIUS * math.cos(theta_rad)
            delta_theta = 90
        else:
            center_x = start_state[0] + TURN_RADIUS * math.sin(theta_rad)
            center_y = start_state[1] - TURN_RADIUS * math.cos(theta_rad)
            delta_theta = -90

        if movement_direction == 'backward':
            delta_theta = -delta_theta

        # Calculate new orientation
        new_theta = (start_state[2] + delta_theta) % 360

        # Determine number of samples based on arc length
        arc_length = abs(math.radians(delta_theta)) * TURN_RADIUS
        num_samples = max(1, int(arc_length / SAMPLE_DISTANCE))

        for i in range(1, num_samples + 1):
            t = i / num_samples
            current_angle = math.atan2(start_state[1] - center_y, start_state[0] - center_x) + t * math.radians(delta_theta)
            x = center_x + TURN_RADIUS * math.cos(current_angle)
            y = center_y + TURN_RADIUS * math.sin(current_angle)
            theta = (start_state[2] + t * delta_theta) % 360
            positions.append((x, y, theta))

    return positions

if __name__ == "__main__":
    main()
