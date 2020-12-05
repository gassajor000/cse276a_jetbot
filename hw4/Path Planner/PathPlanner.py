"""
    created by Jordan Gassaway, 12/5/2020
    PathPlanner: Module for planning paths from point a to point b
"""
from typing import List

from hw4.VoronoiMap import VoronoiMap, Node


class PathPlanner:
    def __init__(self, boundary_points: List[Node], obstacles: List[List[Node]]):
        """
        Initialize Path Planner
        :param boundary_points: list of (x, y) points enclosing the workspace of the robot.
        :param obstacles: list of obstacles in the space (represented by a list of points outlining the obstacle)
        """
        self.map = VoronoiMap(boundary_points, obstacles)

    def getPath(self, position: Node, destination: Node) -> List[Node]:
        """
        Plan a path from current position to destination while avoiding obstacles
        :param position: current (x,y) position
        :param destination: destination point (x,y)
        :return: List of points comprising a path from position to destination
        """