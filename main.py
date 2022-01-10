from typing import Dict, List
import numpy as np
import math
from classes import Map, Node
import matplotlib.pyplot as plt

show_animation = True

class AStar:
    def __init__(self, map: Map):
        """
            Initialize grid map
            map: a Map object
        """
        self.map = map

    def convert_to_grid_coord(self, i, min_i):
        return round((i - min_i) / self.map.resolution)

    def convert_to_graph_coord(self, i, min_i):
        return i*self.map.resolution + min_i

    @staticmethod
    def calculate_heuristic(node1: Node, node2: Node):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2) 

    def get_neighbours(self, node: Node) -> List[Node]:
        """
            Get all the neighbours of a node.
            input:
                node: a Node object
            output:
                neighbours: a list of Node objects
        """
        neighbours = []
        for i in range(node.x - 1, node.x + 2):
            for j in range(node.y - 1, node.y + 2):
                if i == node.x and j == node.y:
                    continue
                if i < 0 or i >= self.map.grid_x or j < 0 or j >= self.map.grid_y:
                    continue
                if self.map.obstacles[i, j] == 1:
                    continue
                neighbours.append(Node(i, j, node.g + 1, node))
        return neighbours
    
    def reconstruct_path(self, node: Node):
        """
            Reconstruct the path from a node.
            input:
                node: a Node object
            output:
                rx: a list of x positions
                ry: a list of y positions
        """
        rx, ry = [], []
        current = node
        while current is not None:
            rx.append(self.convert_to_graph_coord(current.x, self.map.min_x))
            ry.append(self.convert_to_grid_coord(current.y, self.map.min_y))
            current = current.parent
        return rx[::-1], ry[::-1]

    def plan(self, sx, sy, gx, gy):
        """
            A Star Path Planning

            input:
                sx: start x position
                sy: start y position
                gx: goal x position
                gy: goal y position
            output:
                rx: final path x positions list
                ry: final path y positions list
        """

        start_node = Node(self.convert_to_grid_coord(sx, self.map.min_x), self.convert_to_grid_coord(sy, self.map.min_x), 0, None)
        goal_node = Node(self.convert_to_grid_coord(gx, self.map.min_y), self.convert_to_grid_coord(gy, self.map.min_y), 0, None)

        open_set: Dict[int, Node] = dict()
        closed_set: Dict[int, Node] = dict()
        open_set[hash(start_node)] = start_node

        while len(open_set) > 0:
            current: Node = min(open_set.values(), key=lambda x:x.f)
            if current.x == goal_node.x and current.y == goal_node.y:
                rx, ry = self.reconstruct_path(current)
                return rx, ry
            
            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.convert_to_graph_coord(current.x, self.map.min_x),
                         self.convert_to_graph_coord(current.y, self.map.min_x), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            open_set.pop(hash(current))
            closed_set[hash(current)] = current

            for node in self.get_neighbours(current):
                if hash(node) in closed_set:
                    continue
                if hash(node) not in open_set:
                    open_set[hash(node)] = node
                else:
                    if open_set[hash(node)].g > node.g:
                        open_set[hash(node)].update(node.g, node.h)
                        open_set[hash(node)].parent = current

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    resolution = 2.0  # [m]

    map = Map((sx, sy), (gx, gy), (-10, -10), (60, 60), resolution)

    # set obstacle positions
    map.addObstacleX((-10, 60), -10)
    map.addObstacleY(60, (-10, 60))
    map.addObstacleX((-10, 60), 60)
    map.addObstacleY(-10, (-10, -60))
    map.addObstacleY(20, (-10, 40))

    # show graph

    if show_animation:  # pragma: no cover
        plt.plot(map.ox, map.oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStar(map)
    rx, ry = a_star.plan(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()