"""
    created by Jordan Gassaway, 12/5/2020
    VoronoiMap: Creates a map of paths around the configuration space based
    on a Voronoi representation of the space.
"""
import collections
import heapq
import math
from typing import List, Tuple, Dict, Union
from scipy.spatial import Voronoi

from . import Point

PI_2 = 2 * math.pi


def dist(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

class VoronoiMap:
    vertices = []  # type: List[Point]
    ridge_vertices = []  # type: List[Tuple[Point, Point]]
    graph = {}          # type: Dict[Point: List[tuple[Point, float]]]
    FILL_DENSITY = 8

    def __init__(self, boundaries: List[Point], obstacles: List[List[Point]]):
        self.obstacles = obstacles
        self.boundaries = boundaries

        obstacle_points = [self._fill_polygon(obstacle) for obstacle in obstacles]
        boundary_points = self._fill_polygon(boundaries)

        all_points = boundary_points
        for obstacle in obstacle_points:
            all_points += obstacle

        self.voronoi = Voronoi(all_points, incremental=True)
        self._make_graph()

    def _fill_edge_with_points(self, p1, p2):
        dx = (p2[0] - p1[0]) / self.FILL_DENSITY
        dy = (p2[1] - p1[1]) / self.FILL_DENSITY
        new_points = [p1, p2]
        for j in range(self.FILL_DENSITY):
            new_points.append((p1[0] + dx * j, p1[1] + dy * j))
        return new_points

    def _fill_polygon(self, poly_points):
        fill_points = []
        for i in range(len(poly_points)):
            b1 = poly_points[i]
            b2 = poly_points[i + 1] if i != len(poly_points) - 1 else poly_points[0]
            fill_points += self._fill_edge_with_points(b1, b2)
        return fill_points

    def _make_graph(self):
        self.vertices = list(map(lambda v: Point(v[0], v[1]), self.voronoi.vertices))
        self.ridge_vertices = []
        invalid_vertices = []      # vertices inside the obstacles

        def angle_btw_points(origin: Point, p1: Point, p2: Point):
            """returns angle between two points and the origin (center) point"""
            theta1 = math.atan2(p1.y - origin.y, p1.x - origin.x)
            theta2 = math.atan2(p2.y - origin.y, p2.x - origin.x)
            d_theta = theta2 - theta1
            if d_theta < -math.pi:
                d_theta += PI_2
            elif d_theta > math.pi:
                d_theta -= PI_2

            return d_theta

        def inside_polygon(pol: List[Point], p: Point):
            """returns true if p is inside the polygon.
            Sums angles between p and each pair of points in pol. Sum is equal to 0 if false, Pi if true."""
            angle_sum = 0.0
            n = len(pol)
            for i in range(n):
                angle_sum += angle_btw_points(p, pol[i], pol[(i+1) % n])

            return abs(angle_sum) >= math.pi

        # find invalid vertices (inside an obstacle)
        for i in range(len(self.vertices)):
            for obstacle in self.obstacles:
                v = self.vertices[i]
                if inside_polygon(obstacle, v):
                    invalid_vertices.append(i)      # mark as invalid
                    break

        # remove edges that connect to an invalid vertex
        for edge in self.voronoi.ridge_vertices:
            # finite-valid vertices only
            if -1 not in edge and edge[0] not in invalid_vertices and edge[1] not in invalid_vertices:
                self.ridge_vertices.append((self.vertices[edge[0]], self.vertices[edge[1]]))

        # remove invalid vertices
        tmp_vertices = list(map(lambda i: self.vertices[i], invalid_vertices))
        for v in tmp_vertices:
            self.vertices.remove(v)

        # Assemble all the edges into a graph
        self.graph = {p: [] for p in self.vertices}
        for edge in self.ridge_vertices:
            d = dist(edge[0], edge[1])
            self.graph[edge[0]].append((edge[1], d))
            self.graph[edge[1]].append((edge[0], d))

    def add_obstacle(self, obstacle: List[Point]):
        """
        Add an obstacle to the map
        :param obstacle: list of points outlining an obstacle
        """
        obstacle_points = self._fill_polygon(obstacle)
        self.voronoi.add_points(obstacle_points)
        self._make_graph()

    def get_closest_vertex(self, p: Point):
        """get the closest vertex to a point"""
        min_d = None
        closest = None
        for vertex in self.vertices:
            d = dist(p, vertex)
            if min_d is None or d < min_d:
                min_d = d
                closest = vertex

        return closest

    PathNode =  collections.namedtuple('PathNode', 'dist vertex prev_node')

    def find_shortest_path_to(self, start: Point, end: Point) -> Union[List[Point], None]:
        """
        Find a path from start to end vertices
        :param visited_nodes: list of nodes already visited
        :param start: starting vertex
        :param end: ending vertex
        :return: path from start to end, None if path does not exist.
        """
        node_mapping = { v: self.PathNode(float('infinity'), v, None) for v in self.vertices }   # type: Dict[Point, VoronoiMap.PathNode]
        node_mapping[start] = self.PathNode(0.0, start, None)
        dijkstra_q = list(node_mapping.values())     # list of nodes to process
        heapq.heapify(dijkstra_q)

        def process_node(n: VoronoiMap.PathNode):
            for next_point, dist_to_next in self.graph[n.vertex]:
                next_node = node_mapping[next_point]
                if next_node not in dijkstra_q:     # already found shortest path to node
                    continue
                dist_through_n = n.dist + dist_to_next
                if next_node.dist > dist_through_n:
                    dijkstra_q.remove(next_node)        # Node needs to be removed and readded since we are changing the priority
                    new_next = self.PathNode(dist_through_n, next_node.vertex, n)
                    node_mapping[next_node.vertex] = new_next
                    heapq.heappush(dijkstra_q, new_next)

        def build_path():
            n = node_mapping[end]
            path = []
            while n.vertex is not start:
                path.insert(0, n.vertex)
                n = n.prev_node

            path.insert(0, n.vertex)    # add start node
            return path

        while dijkstra_q:
            node = heapq.heappop(dijkstra_q)
            process_node(node)
            if node.vertex is end:      # break out once we reach the end
                break

            if node.dist == float('infinity'):  # Node (an all remaining nodes) are unreachable from start. End was not reachable.
                return None

        return build_path()

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

        ax.plot(self.voronoi.points[:, 0], self.voronoi.points[:, 1], '.', markersize=None)
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

        _adjust_bounds(ax, self.voronoi.points)
        return ax

    def plotPath(self, path: List[Point]):
        """plot a path over the map"""
        from matplotlib.collections import LineCollection
        ax = self.plot()

        segments = []
        for i in range(len(path)):
            if i != len(path) - 1:
                segments.append([(path[i].x, path[i].y), (path[i+1].x, path[i+1].y)])

        ax.add_collection(LineCollection(segments,
                                         colors='red',
                                         lw=2.0,
                                         alpha=1.0,
                                         linestyle='solid'))

        return ax
