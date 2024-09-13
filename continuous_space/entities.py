# entities.py
import math
from constants import ROBOT_W, ROBOT_H, OBSTACLE_W, OBSTACLE_H

class Robot:
    def __init__(self, pos: list, theta: float) -> None:
        self.center_pos = pos
        self.theta = theta  # Orientation in degrees
        self.width = float(ROBOT_W)
        self.height = float(ROBOT_H)

    def get_center_pos(self) -> list:
        return self.center_pos

    def set_center_pos(self, pos) -> None:
        self.center_pos = pos

    def get_theta(self) -> float:
        return self.theta

    def set_theta(self, theta: float) -> None:
        self.theta = theta % 360

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_pos(self) -> list:
        x, y = self.get_center_pos()
        w = self.get_width()
        h = self.get_height()
        theta_rad = math.radians(self.theta)

        # Corner offsets before rotation (relative to center)
        corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]

        # Rotate corners
        rotated_corners = []
        for dx, dy in corners:
            rotated_x = x + dx * math.cos(theta_rad) - dy * math.sin(theta_rad)
            rotated_y = y + dx * math.sin(theta_rad) + dy * math.cos(theta_rad)
            rotated_corners.append([rotated_x, rotated_y])

        return rotated_corners

class Obstacle:
    def __init__(self, pos: list, theta: float) -> None:
        self.center_pos = pos
        self.theta = theta  # Orientation in degrees
        self.width = float(OBSTACLE_W)
        self.height = float(OBSTACLE_H)

    def get_center_pos(self) -> list:
        return self.center_pos

    def set_center_pos(self, pos) -> None:
        self.center_pos = pos

    def get_theta(self) -> float:
        return self.theta

    def set_theta(self, theta: float) -> None:
        self.theta = theta % 360

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_pos(self) -> list:
        x, y = self.get_center_pos()
        w = self.get_width()
        h = self.get_height()
        theta_rad = math.radians(self.theta)

        # Corner offsets before rotation (relative to center)
        corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]

        # Rotate corners
        rotated_corners = []
        for dx, dy in corners:
            rotated_x = x + dx * math.cos(theta_rad) - dy * math.sin(theta_rad)
            rotated_y = y + dx * math.sin(theta_rad) + dy * math.cos(theta_rad)
            rotated_corners.append([rotated_x, rotated_y])

        return rotated_corners

class Field:
    def __init__(self, r, obs):
        self.robot = Robot([0, 0], 90)  # Default orientation at 90 degrees (up)
        self.obstacles = []
        self.field = [[0] * (2 * r + 1) for _ in range(2 * r + 1)]

    def get_robot(self):
        return self.robot

    def set_robot(self, robot):
        self.robot = robot

    def get_obstacles(self):
        return self.obstacles

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
        
class State:
    def __init__(self, x, y, theta, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta % 360  # Orientation in degrees (0-360)
        self.g = g          # Cost from start node
        self.h = h          # Heuristic cost to goal
        self.f = g + h      # Total cost
        self.parent = parent  # Parent state in the path

    def __lt__(self, other):  # For priority queue
        return self.f < other.f