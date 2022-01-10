import math
import numpy as np

class Map:
    def __init__(self, start, goal, mins, maxs, resolution):
        """
            Initialize a map object.
            obstacle_map: a list of lists of 0s and 1s. 1 means obstacle.
            start: a tuple of (x, y)
            goal: a tuple of (x, y)
            resolution: the resolution of the map
        """
        self.start = start # in graph coordinates
        self.goal = goal # in graph coordinates
        self.min_x = mins[0] # in graph coordinates
        self.min_y = mins[1] # in graph coordinates
        self.max_x = maxs[0] # in graph coordinates
        self.max_y = maxs[1] # in graph coordinates
        self.resolution = resolution 
        self.grid_y = self.convert_to_grid_coord(self.max_y, self.min_y) # in grid coordinates
        self.grid_x = self.convert_to_grid_coord(self.max_x, self.min_x) # in grid coordinates
        self.obstacles = np.zeros((self.grid_x, self.grid_y)) # in grid coordinates
        self.obstacle_dims = self.obstacles.shape # in grid coordinates
        self.ox = []
        self.oy = []
    
    def convert_to_grid_coord(self, i, min_i):
        return round((i - min_i) / self.resolution)

    def addObstacleX(self, xt: tuple, y):
        """
            Add an obstacle to the map.
            start: a tuple of (x, y)
            end: a tuple of (x, y)
        """
        self.obstacles[self.convert_to_grid_coord(xt[0], self.min_x):self.convert_to_grid_coord(xt[1], self.min_x), self.convert_to_grid_coord(y, self.min_y)-1] = 1
        self.obstacle_dims = self.obstacles.shape
        for i in range(xt[0], xt[1]):
            self.ox.append(i)
            self.oy.append(y)


    def addObstacleY(self, x, yt:tuple):
        """
            Add an obstacle to the map.
            start: a tuple of (x, y)
            end: a tuple of (x, y)
        """
        self.obstacles[self.convert_to_grid_coord(x, self.min_x)-1, self.convert_to_grid_coord(yt[0], self.min_y):self.convert_to_grid_coord(yt[1], self.min_y)] = 1
        self.obstacle_dims = self.obstacles.shape
        for i in range(yt[0], yt[1]):
            self.ox.append(x)
            self.oy.append(i)

class Node:
    def __init__(self, x, y, cost, parent):
        """
            Initialize a node object.
            x: x coordinate
            y: y coordinate
            cost: cost to reach this node
            parent: the parent node
        """
        self.x = x
        self.y = y
        self.loc = (x, y)
        self.parent = parent
        self.g = cost # g(n) = cost to reach this node
        self.h = 0 # h(n) = heuristic cost to reach the goal
        self.f = self.g + self.h # f(n) = g(n) + h(n)
    
    def update(self, g, h):
        """
            Update the g and h values of the node.
            g: new g value
            h: new h value
        """
        self.g = g
        self.h = h
        self.f = self.g + self.h
    
    def __hash__(self):
        return hash((self.x, self.y))