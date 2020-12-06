"""
    created by Jordan Gassaway, 12/5/2020
    VoronoiMap: Creates a map of paths around the configuration space based
    on a Voronoi representation of the space.
"""
import math
from typing import List
from scipy.spatial import Voronoi

from . import Point


class VoronoiMap:
    vertices = []  # type: List[Point]
    ridge_vertices = []  # type: List[tuple[Point]]
    FILL_DENSITY = 8

    def __init__(self, boundaries: List[Point], obstacles: List[List[Point]]):
        self.obstacles = obstacles
        self.boundaries = boundaries

        obstacle_points = [self._fill_polygon(obstacle) for obstacle in obstacles]
        boundary_points = self._fill_polygon(boundaries)

        all_points = boundary_points
        for obstacle in obstacle_points:
            all_points += obstacle

        self.graph = Voronoi(all_points, incremental=True)
        self._prune_blocked_paths()

    def _fill_edge_with_points(self, p1, p2):
        dx = (p2[0] - p1[0]) / self.FILL_DENSITY
        dy = (p2[1] - p1[1]) / self.FILL_DENSITY
        new_points = [p1, p2]
        for j in range(8):
            new_points.append((p1[0] + dx * j, p1[1] + dy * j))
        return new_points

    def _fill_polygon(self, poly_points):
        fill_points = []
        for i in range(len(poly_points)):
            b1 = poly_points[i]
            b2 = poly_points[i + 1] if i != len(poly_points) - 1 else poly_points[0]
            fill_points += self._fill_edge_with_points(b1, b2)
        return fill_points

    def _prune_blocked_paths(self):
        self.vertices = list(map(lambda v: Point(v[0], v[1]),self.graph.vertices))
        self.ridge_vertices = []
        invalid_vertices = []      # vertices inside the obstacles

        def angle_btw_points(origin: Point, p1: Point, p2: Point):
            """returns angle between two points and the origin (center) point"""
            theta1 = math.atan2(p1.y - origin.y, p1.x - origin.x)
            theta2 = math.atan2(p2.y - origin.y, p2.x - origin.x)
            d_theta = abs(theta2 - theta1)
            return d_theta % (math.pi * 2)

        def inside_polygon(pol: List[Point], p: Point):
            """returns true if p is inside the polygon.
            Sums angles between p and each pair of points in pol. Sum is equal to 0 if false, Pi if true."""
            angle_sum = 0.0
            n = len(pol)
            for i in range(n):
                angle_sum += angle_btw_points(p, pol[i], pol[(i+1) % n])

            return angle_sum >= math.pi

        # find invalid vertices (inside an obstacle)
        for i in range(len(self.vertices)):
            for obstacle in self.obstacles:
                v = self.vertices[i]
                if inside_polygon(obstacle, v):
                    invalid_vertices.append(i)      # mark as invalid
                    break

        # remove edges that connect to an invalid vertex
        for edge in self.graph.ridge_vertices:
            # finite-valid vertices only
            if -1 not in edge and edge[0] not in invalid_vertices or edge[1] not in invalid_vertices:
                self.ridge_vertices.append((self.vertices[edge[0]], self.vertices[edge[1]]))

        # remove invalid vertices
        tmp_vertices = list(map(lambda i: self.vertices[i], invalid_vertices))
        for v in tmp_vertices:
            self.vertices.remove(v)

    def add_obstacle(self, obstacle: List[Point]):
        """
        Add an obstacle to the map
        :param obstacle: list of points outlining an obstacle
        """
        obstacle_points = self._fill_polygon(obstacle)
        self.graph.add_points(obstacle_points)
        self._prune_blocked_paths()

    def plot(self):
        """plot using pyplot"""
        def _adjust_bounds(ax, points):
            margin = 0.1 * points.ptp(axis=0)
            xy_min = points.min(axis=0) - margin
            xy_max = points.max(axis=0) + margin
            ax.set_xlim(xy_min[0], xy_max[0])
            ax.set_ylim(xy_min[1], xy_max[1])
        from matplotlib.collections import LineCollection
        import matplotlib.pyplot as plt
        import numpy as np

        fig = plt.figure()
        ax = fig.gca()

        vertices = np.zeros((len(self.vertices), 2), dtype=float)
        for i, v in enumerate(self.vertices):
            vertices[i][0], vertices[i][1] = v.x, v.y

        ax.plot(self.graph.points[:, 0], self.graph.points[:, 1], '.', markersize=None)
        ax.plot(vertices[:, 0], vertices[:, 1], 'o')

        line_colors = 'k'
        line_width = 1.0
        line_alpha = 1.0

        finite_segments = [[(edge[0].x, edge[0].y), (edge[1].x, edge[1].y)] for edge in self.ridge_vertices]

        ax.add_collection(LineCollection(finite_segments,
                                         colors=line_colors,
                                         lw=line_width,
                                         alpha=line_alpha,
                                         linestyle='solid'))

        _adjust_bounds(ax, self.graph.points)
