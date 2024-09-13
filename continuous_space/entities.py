from constants import *

class Robot: 
    def __init__(self, pos: list, d: Direction) -> None:
        self.center_pos = pos
        self.direction = d
        self.width = float(ROBOT_W)
        self.height = float(ROBOT_H)
    
    def get_center_pos(self) -> list:
        return self.center_pos
    
    def set_center_pos(self, pos) -> None:
        self.center_pos = pos
    
    def get_direction(self) -> Direction:
        return self.direction
    
    def set_direction(self, d) -> Direction:
        self.direction = d
        
    def get_width(self):
        return self.width
    
    def get_height(self):
        return self.height
    
    def get_pos(self) -> list:
        x, y = self.get_center_pos()
        w = self.get_width()
        h = self.get_height()
        
        if self.direction == Direction.UP:
            bottom_left = [x - w / 2, y - h / 2]
            top_right = [x + w / 2, y + h / 2]
        
        elif self.direction == Direction.DOWN:
            bottom_left = [x - w / 2, y - h / 2]
            top_right = [x + w / 2, y + h / 2]
        
        elif self.direction == Direction.LEFT:
            bottom_left = [x - h / 2, y - w / 2]
            top_right = [x + h / 2, y + w / 2]
        
        elif self.direction == Direction.RIGHT:
            bottom_left = [x - h / 2, y - w / 2]
            top_right = [x + h / 2, y + w / 2]
            
        return [bottom_left, top_right]
    
class Obstacle: 
    def __init__(self, pos: list, d: Direction) -> None:
        self.center_pos = pos
        self.direction = d
        self.width = float(OBSTACLE_W)
        self.height = float(OBSTACLE_H)
    
    def get_center_pos(self) -> list:
        return self.center_pos
    
    def set_center_pos(self, pos) -> None:
        self.center_pos = pos
    
    def get_direction(self) -> Direction:
        return self.direction
    
    def set_direction(self, d) -> Direction:
        self.direction = d
        
    def get_width(self):
        return self.width
    
    def get_height(self):
        return self.height
    
    def get_pos(self) -> list:
        x, y = self.get_center_pos()
        w = self.get_width()
        h = self.get_height()
        
        if self.direction == Direction.UP:
            bottom_left = [x - w / 2, y - h / 2]
            top_right = [x + w / 2, y + h / 2]
        
        elif self.direction == Direction.DOWN:
            bottom_left = [x - w / 2, y - h / 2]
            top_right = [x + w / 2, y + h / 2]
        
        elif self.direction == Direction.LEFT:
            bottom_left = [x - h / 2, y - w / 2]
            top_right = [x + h / 2, y + w / 2]
        
        elif self.direction == Direction.RIGHT:
            bottom_left = [x - h / 2, y - w / 2]
            top_right = [x + h / 2, y + w / 2]
            
        return [bottom_left, top_right]
    
class Field:
    def __init__(self, r, obs):
        self.robot = Robot([0, 0], Direction.UP)
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
        
        