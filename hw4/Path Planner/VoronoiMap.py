"""
    created by Jordan Gassaway, 12/5/2020
    VoronoiMap: Creates a map of paths around the configuration space based
    on a Voronoi representation of the space.
"""
from typing import List
from scipy.spatial import Voronoi

from . import Point


class VoronoiMap:
    FILL_DENSITY = 8

    def __init__(self, boundaries: List[Point], obstacles: List[List[Point]]):
        self.obstacles = obstacles
        self.boundaries = boundaries

        def fill_edge_with_points(p1, p2):
            dx = (p2[0] - p1[0]) / self.FILL_DENSITY
            dy = (p2[1] - p1[1]) / self.FILL_DENSITY
            new_points = [p1, p2]
            for j in range(8):
                new_points.append((p1[0] + dx * j, p1[1] + dy * j))
            return new_points

        def fill_polygon(poly_points):
            fill_points = []
            for i in range(len(poly_points)):
                b1 = obstacles[i]
                b2 = obstacles[i + 1] if i != len(poly_points) - 1 else poly_points[0]
                fill_points += fill_edge_with_points(b1, b2)
            return fill_points

        obstacle_points = [fill_polygon(obstacle) for obstacle in obstacles]
        boundary_points = fill_polygon(boundaries)

        all_points = boundary_points
        for obstacle in obstacle_points:
            all_points += obstacle

        self.graph = Voronoi(all_points)

    def add_obstacle(self, obstacle: List[Point]):
        """
        Add an obstacle to the map
        :param obstacle: list of points outlining an obstacle
        """
        pass
