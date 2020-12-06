"""
    created by Jordan Gassaway, 12/5/2020
    PathPlanner: Module for planning paths from point a to point b
"""
from typing import List

from .VoronoiMap import VoronoiMap
from . import Point


class PathPlanner:
    def __init__(self, boundary_points: List[Point], obstacles: List[List[Point]]):
        """
        Initialize Path Planner
        :param boundary_points: list of (x, y) points enclosing the workspace of the robot.
        :param obstacles: list of obstacles in the space (represented by a list of points outlining the obstacle)
        """
        self.map = VoronoiMap(boundary_points, obstacles)

    def getPath(self, position: Point, destination: Point) -> List[Point]:
        """
        Plan a path from current position to destination while avoiding obstacles
        :param position: current (x,y) position
        :param destination: destination point (x,y)
        :return: List of points comprising a path from position to destination
        """