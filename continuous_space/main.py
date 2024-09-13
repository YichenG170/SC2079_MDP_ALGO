import math
import pygame
from a_star import a_star_search
from entities import *
from constants import *

def main():
    # Initialize the field
    field = Field(r=100, obs=[])
    robot = Robot([50, 50], Direction.UP)  # Start at (50,50) facing 90 degrees (up)
    field.set_robot(robot)

    # Add obstacles
    obstacle1 = Obstacle([100, 100], Direction.UP)  # Facing 135 degrees
    field.add_obstacle(obstacle1)

    obstacle2 = Obstacle([150, 50], Direction.UP)  # Facing 225 degrees
    field.add_obstacle(obstacle2)

    # Set target position and orientation
    target_position = [150, 150]
    target_theta = 0  # Facing right

    # Run A* search
    path = a_star_search(field, target_position, target_theta)
    if path:
        print("Optimal path found.\n")
        # Compute and print commands
        compute_and_print_commands(path)
        # Visualize the path using Pygame
        visualize_path(field, path)
    else:
        print("No path found.")

def compute_and_print_commands(path):
    print("Commands:")
    for i in range(1, len(path)):
        prev_x, prev_y, prev_theta = path[i - 1]
        curr_x, curr_y, curr_theta = path[i]

        dx = curr_x - prev_x
        dy = curr_y - prev_y
        distance = math.hypot(dx, dy)
        angle_diff = (curr_theta - prev_theta + 360) % 360

        if angle_diff == 0 or angle_diff == 180:
            # Straight movement
            direction = 'forward' if angle_diff == 0 else 'backward'
            print(f"MOVE: {distance:.2f} units {direction}")
        elif abs(angle_diff) == 90:
            # Quarter-circle turn
            turn_direction = 'left' if (angle_diff == 90) else 'right'
            movement_direction = 'forward' if angle_diff == 90 else 'backward'
            print(f"TURN: Quarter-circle {turn_direction} {movement_direction}")
        else:
            # Handle other cases if needed
            pass
    print("\n")

def visualize_path(field, path):
    # Initialize Pygame
    pygame.init()
    scale_factor = 3  # Enlarge the field by this factor
    window_size = (int(FIELD_W * scale_factor), int(FIELD_H * scale_factor))
    screen = pygame.display.set_mode(window_size)
    pygame.display.set_caption("Robot Path Visualization")
    clock = pygame.time.Clock()

    # Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    BLUE = (0, 0, 255)

    # Load obstacles
    obstacles = field.get_obstacles()

    # Main loop variables
    running = True
    index = 0
    path_length = len(path)
    path_positions = []  # Store positions to draw the path

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if index < path_length - 1:
            # Interpolate between the current and next state
            curr_state = State(*path[index])
            next_state = State(*path[index + 1])

            interpolated_positions = interpolate_states(curr_state, next_state)
            path_positions.extend(interpolated_positions)
            index += 1
        else:
            # Pause at the end of the path
            pass

        # Clear screen
        screen.fill(WHITE)

        # Draw path
        draw_path(screen, path_positions, scale_factor)

        # Draw obstacles
        for obstacle in obstacles:
            draw_entity(screen, obstacle, BLACK, scale_factor)

        # Draw robot
        draw_entity(screen, field.robot, BLUE, scale_factor)

        # Update display
        pygame.display.flip()
        clock.tick(10)  # Control the speed of the animation (adjust as needed)

    pygame.quit()
    
def interpolate_states(start_state, end_state):
    positions = []

    angle_diff = (end_state.theta - start_state.theta + 360) % 360
    if angle_diff == 0 or angle_diff == 180:
        # Straight movement
        distance = math.hypot(end_state.x - start_state.x, end_state.y - start_state.y)
        num_samples = max(1, int(distance / SAMPLE_DISTANCE))
        for i in range(1, num_samples + 1):
            t = i / num_samples
            x = start_state.x + t * (end_state.x - start_state.x)
            y = start_state.y + t * (end_state.y - start_state.y)
            theta = start_state.theta
            positions.append((x, y, theta))
    elif abs(angle_diff) == 90:
        # Turning movement
        radius = TURN_RADIUS
        delta_theta = angle_diff if angle_diff == 90 else -90
        turn_direction = 'left' if delta_theta > 0 else 'right'
        theta_rad = math.radians(start_state.theta)

        # Determine center of rotation
        if turn_direction == 'left':
            center_x = start_state.x - radius * math.sin(theta_rad)
            center_y = start_state.y + radius * math.cos(theta_rad)
        else:
            center_x = start_state.x + radius * math.sin(theta_rad)
            center_y = start_state.y - radius * math.cos(theta_rad)

        arc_length = math.radians(abs(delta_theta)) * radius
        num_samples = max(1, int(arc_length / SAMPLE_DISTANCE))
        for i in range(1, num_samples + 1):
            t = i / num_samples
            angle = math.radians(start_state.theta) + t * math.radians(delta_theta)
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            theta = (start_state.theta + t * delta_theta) % 360
            positions.append((x, y, theta))
    else:
        # Handle other cases if necessary
        pass

    return positions

def draw_path(screen, path_positions, scale_factor):
    # Draw the path as light grey lines
    LIGHT_GREY = (200, 200, 200)
    if len(path_positions) >= 2:
        # Convert positions to screen coordinates
        scaled_positions = [((x * scale_factor), (FIELD_H - y) * scale_factor) for x, y, _ in path_positions]
        pygame.draw.lines(screen, LIGHT_GREY, False, scaled_positions, 2)  # Adjust line width as needed

    # Draw arrows every 25 units
    draw_arrows_on_path(screen, path_positions, scale_factor)

def draw_arrows_on_path(screen, path_positions, scale_factor):
    # Draw arrows every 25 units along the path
    LIGHT_GREY = (200, 200, 200)
    arrow_interval = 25  # Units
    total_distance = 0
    next_arrow_distance = arrow_interval

    for i in range(1, len(path_positions)):
        x1, y1, theta1 = path_positions[i - 1]
        x2, y2, theta2 = path_positions[i]
        dx = x2 - x1
        dy = y2 - y1
        segment_distance = math.hypot(dx, dy)

        while total_distance + segment_distance >= next_arrow_distance:
            fraction = (next_arrow_distance - total_distance) / segment_distance
            arrow_x = x1 + fraction * dx
            arrow_y = y1 + fraction * dy
            # Optionally interpolate theta
            arrow_theta = theta1 + fraction * (theta2 - theta1)
            x_screen = arrow_x * scale_factor
            y_screen = (FIELD_H - arrow_y) * scale_factor  # Adjust y-axis
            draw_small_arrow(screen, x_screen, y_screen, arrow_theta, scale_factor, LIGHT_GREY)
            next_arrow_distance += arrow_interval

        total_distance += segment_distance

def draw_small_arrow(screen, x, y, theta, scale_factor, color):
    # Draw a small arrow at (x, y) pointing in direction theta
    arrow_length = 10  # Adjust size as needed
    arrow_width = 5  # Adjust width as needed

    theta_rad = math.radians(theta)
    end_x = x + arrow_length * math.cos(-theta_rad)
    end_y = y + arrow_length * math.sin(-theta_rad)

    # Draw arrow shaft
    pygame.draw.line(screen, color, (x, y), (end_x, end_y), 2)

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

def draw_entity(screen, entity, color, scale_factor):
    # Get entity corners
    corners = entity.get_pos()
    scaled_corners = [((x * scale_factor), (FIELD_H - y) * scale_factor) for x, y in corners]  # Adjust y-axis

    # Draw entity as a polygon
    pygame.draw.polygon(screen, color, scaled_corners)

    # Draw direction arrow
    draw_direction_arrow(screen, entity, color=(255, 255, 255), scale_factor=scale_factor)

def draw_direction_arrow(screen, entity, color, scale_factor):
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

if __name__ == "__main__":
    main()
