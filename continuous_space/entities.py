# entities.py
import math
from constants import ROBOT_W, ROBOT_H, OBSTACLE_W, OBSTACLE_H

class Robot:
    def __init__(self, pos: list, theta: float) -> None:
        self.center_pos = [pos[0]+5, pos[1]+5]  # [x, y]
        self.theta = theta      # Orientation in degrees
        self.width = float(ROBOT_W)
        self.height = float(ROBOT_H)

    def __repr__(self):
        return f"Robot(Position: {self.center_pos}, Orientation: {self.theta})"

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

        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        w_2 = w / 2
        h_2 = h / 2

        # Define corners relative to the center
        corners = [(-w_2, -h_2), (w_2, -h_2), (w_2, h_2), (-w_2, h_2)]

        # Precompute the rotation matrix
        rotation_matrix = [
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ]

        # Rotate corners based on current orientation
        rotated_corners = []
        for dx, dy in corners:
            rotated_x = x + dx * rotation_matrix[0][0] + dy * rotation_matrix[0][1]
            rotated_y = y + dx * rotation_matrix[1][0] + dy * rotation_matrix[1][1]
            rotated_corners.append((rotated_x, rotated_y))

        return rotated_corners

class Obstacle:
    def __init__(self, pos: list, theta: float) -> None:
        self.center_pos = pos  # [x, y]
        self.theta = theta      # Orientation in degrees
        self.width = float(OBSTACLE_W)
        self.height = float(OBSTACLE_H)

    def __repr__(self):
        return f"Obstacle(Position: {self.center_pos}, Orientation: {self.theta})"

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

        # Define corners relative to the center
        corners = ((-w / 2, -h / 2), (w / 2, -h / 2),
                   (w / 2, h / 2), (-w / 2, h / 2))
        
        # Rotate corners based on current orientation
        rotated_corners = []
        for dx, dy in corners:
            rotated_x = x + dx * math.cos(theta_rad) - dy * math.sin(theta_rad)
            rotated_y = y + dx * math.sin(theta_rad) + dy * math.cos(theta_rad)
            rotated_corners.append((rotated_x, rotated_y))

        return tuple(rotated_corners)

class Field:
    def __init__(self, r, obs):
        self.robot = Robot([0, 0], 90)  # Default position and orientation
        self.obstacles = []
        self.field = [[0] * (2 * r + 1) for _ in range(2 * r + 1)]
        for obstacle in obs:
            self.add_obstacle(obstacle)

    def __repr__(self):
        return f"Field(Robot: {self.robot}, Obstacles: {self.obstacles})"

    def get_robot(self):
        return self.robot

    def set_robot(self, robot):
        self.robot = robot

    def get_obstacles(self):
        return self.obstacles

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
