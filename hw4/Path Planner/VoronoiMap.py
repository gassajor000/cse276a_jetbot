"""
    created by Jordan Gassaway, 12/5/2020
    VoronoiMap: Creates a map of paths around the configuration space based
    on a Voronoi representation of the space.
"""
from typing import List


class VoronoiMap:
    def __init__(self, boundary_points: List[Point], obstacles: List[List[Point]]):
        self.obstacles = obstacles
        self.boundary_points = boundary_points
        self.nodes = []
        self.graph = []

    def add_obstacle(self, obstacle: List[Point]):
        """
        Add an obstacle to the map
        :param obstacle: list of points outlining an obstacle
        """
        pass

    def add_locus_point(self, locus_point: Point):
        """
        Add a locus point (boundary point or obstacle point) around which a cell is defined
        :param locus_point:
        :return:
        """
