"""
    created by Jordan Gassaway, 12/5/2020
    VoronoiGraph: Create a voronoi graph using the supplied
"""
import collections
from typing import List, Dict

Point = collections.namedtuple('Point', 'x y')
Node = collections.namedtuple('Node', 'id data')
Edge = collections.namedtuple('Edge', 'node1 node2')

class Graph():
    edges = ...  # type: Dict[int, List[int]]
    nodes = ...  # type: Dict[int, Node]

    def __init__(self):
        self.edges = {}
        self.nodes = {}
        self.next_node = 0

    def add_node(self, data, edges: List[Node]=None):
        """Add a new node to the graph"""
        node = Node(self.next_node, data)
        self.nodes[node.id] = node
        self.edges[node.id] = []

        if edges:
            self.edges[node.id] = edges
            for end_node in edges:
                self.edges[end_node.id].append(node.id)

    def remove_node(self, id: int):
        """Remove a node from the graph (returns the node object)"""
        n = self.nodes.pop(id)
        edges = self.edges[id]

        for n_id in edges:
            self.edges[n_id].remove(id)

        self.edges.pop(id)
        return n

    def add_edge(self, edge: Edge):
        """Add an edge connecting two nodes"""
        self.edges[edge.node1].append(edge.node2)
        self.edges[edge.node2].append(edge.node1)

    def remove_edge(self, edge: Edge):
        """Add an edge connecting two nodes"""
        self.edges[edge.node1].remove(edge.node2)
        self.edges[edge.node2].remove(edge.node1)

    def find_path(self, start: int, end: int) -> List[int]:
        """
        Find a path from node1 to node2
        :param start: starting node id
        :param end: ending node id
        :return: List of node ids comprising path from start to finish. None if no path exists.
        """

class VoronoiGraph:
    def __init__(self, loci: List[Point]):
        self.loci = loci


    def add_locus(self, locus: Point):
        """Add a locus point to the graph"""
