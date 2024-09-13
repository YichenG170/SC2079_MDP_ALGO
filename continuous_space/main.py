# main.py
import math
import pygame
from a_star import a_star_search
from entities import Field, Robot, Obstacle
from constants import FIELD_W, FIELD_H, ROBOT_W, ROBOT_H

def main():
    # Initialize the field
    field = Field(r=100, obs=[])
    robot = Robot([50, 50], 90)  # Start at (50,50) facing 90 degrees (up)
    field.set_robot(robot)

    # Add obstacles
    obstacle1 = Obstacle([100, 100], 135)  # Facing 135 degrees
    field.add_obstacle(obstacle1)

    obstacle2 = Obstacle([150, 50], 225)  # Facing 225 degrees
    field.add_obstacle(obstacle2)

    # Set target position and orientation
    target_position = [150, 150]
    target_theta = 0  # Facing right

    # Run A* search
    path = a_star_search(field, target_position, target_theta)
    if path:
        print("Optimal path found.")
        # Visualize the path using Pygame
        visualize_path(field, path)
    else:
        print("No path found.")

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

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if index < path_length:
            # Update robot state
            x, y, theta = path[index]
            field.robot.set_center_pos([x, y])
            field.robot.set_theta(theta)
            index += 1
        else:
            # Pause at the end of the path
            pass

        # Clear screen
        screen.fill(WHITE)

        # Draw obstacles
        for obstacle in obstacles:
            draw_entity(screen, obstacle, BLACK, scale_factor)

        # Draw robot
        draw_entity(screen, field.robot, BLUE, scale_factor)

        # Update display
        pygame.display.flip()
        clock.tick(10)  # Control the speed of the animation (adjust as needed)

    pygame.quit()

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
    left_wing = (end_x + arrow_width * math.cos(left_wing_angle), end_y + arrow_width * math.sin(left_wing_angle))
    right_wing = (end_x + arrow_width * math.cos(right_wing_angle), end_y + arrow_width * math.sin(right_wing_angle))

    # Draw arrowhead
    pygame.draw.polygon(screen, color, [left_wing, (end_x, end_y), right_wing])

if __name__ == "__main__":
    main()
