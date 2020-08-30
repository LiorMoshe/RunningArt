import logging
from decimal import Decimal
from segments.utils import get_mid_point

minus_id = -1

class NodesManager(object):
    '''
    This class holds all the information related to the intersection nodes that are used by the algorithm.
    It receives queries from the algorithm regarding ids and locations of given nodes in cartesian and geometric
    coordinates
    '''

    def __init__(self, nodes_of_intersections):
        # Map each node id to its location in geo as it is saved in the graph.
        self.nodes_id_to_location = {}

        # Map each node id to the ids of nodes that are connected to it according to its ways.
        self.nodes_ways = {}

        # For debugging, location to id.
        self.location_to_id = {}

        self.intersection_nodes = []

        self.intersections_nodes_idx = nodes_of_intersections


    def get_nodes(self):
        return list(self.nodes_id_to_location.values())

    def get_nodes_ways(self):
        return self.nodes_ways

    def get_location_to_id_map(self):
        return self.location_to_id

    def initialize_ways_graph(self, ways):
        for way in ways:
            way_nodes = way.nodes
            for idx, node in enumerate(way_nodes):
                if node.id in self.intersections_nodes_idx:
                    if node.id not in self.nodes_id_to_location:
                        self.nodes_id_to_location[node.id] = (Decimal(node.lat), Decimal(node.lon))
                        self.location_to_id[(float(node.lat), float(node.lon))] = node.id
                        if node.id not in self.nodes_ways.keys():
                            self.nodes_ways[node.id] = []
                    if idx < len(way_nodes) - 1:
                        next_node = way_nodes[idx + 1]
                        if next_node.id in self.intersections_nodes_idx:
                            self.nodes_ways[node.id].append(next_node.id)
                            if next_node.id not in self.nodes_ways.keys():
                                self.nodes_ways[next_node.id] = []
                            self.nodes_ways[next_node.id].append(node.id)
                        else:
                            tmp_idx = idx + 1
                            found = False
                            while tmp_idx < len(way_nodes) - 1 and found == False:
                                next_node = way_nodes[tmp_idx + 1]
                                if next_node.id in self.intersections_nodes_idx:
                                    found = True
                                    self.nodes_ways[node.id].append(next_node.id)
                                    if next_node.id not in self.nodes_ways.keys():
                                        self.nodes_ways[next_node.id] = []
                                    self.nodes_ways[next_node.id].append(node.id)
                                tmp_idx = tmp_idx + 1
        self.intersection_nodes = self.get_mid_nodes(self.intersections_nodes_idx)

    def get_intersection_nodes(self):
        return self.intersection_nodes

    def get_nodes_map(self):
        return self.nodes_id_to_location

    def get_mid_nodes(self, intersections_nodes):
        global minus_id
        initial_id = minus_id
        curr_id = minus_id

        new_ids = {}
        mid_location_to_id = {}

        for node_id, neighbors_ids in self.nodes_ways.items():
            if node_id in intersections_nodes:
                node_location = self.nodes_id_to_location[node_id]
                for neighbor_id in neighbors_ids:
                    if neighbor_id in intersections_nodes:
                        other_node_loc = self.nodes_id_to_location[neighbor_id]
                        mid_node = get_mid_point(node_location[0], node_location[1], other_node_loc[0],
                                                 other_node_loc[1])
                        min_node_decimal = (Decimal(mid_node[0]), Decimal(mid_node[1]))
                        if min_node_decimal not in mid_location_to_id:
                            logging.info("Mid Node Id: {0}, Location: {1}".format(curr_id, min_node_decimal))
                            mid_location_to_id[min_node_decimal] = curr_id
                            new_ids[curr_id] = [node_id, neighbor_id]
                            curr_id -= 1
                            # nodes_id_to_location[curr_id] = (Decimal(mid_node[0]), Decimal(mid_node[1]))
        for loc, id in mid_location_to_id.items():
            self.nodes_id_to_location[id] = loc
            self.location_to_id[(loc[0], loc[1])] = id
        for i in range(initial_id, curr_id, -1):
            self.nodes_ways[i] = new_ids[i]
            intersections_nodes.append(i)

            self.nodes_ways[new_ids[i][0]].append(i)
            if new_ids[i][1] in self.nodes_ways[new_ids[i][0]]:
                self.nodes_ways[new_ids[i][0]].remove(new_ids[i][1])

            self.nodes_ways[new_ids[i][1]].append(i)
            if new_ids[i][0] in self.nodes_ways[new_ids[i][1]]:
                self.nodes_ways[new_ids[i][1]].remove(new_ids[i][0])
        minus_id = curr_id
        return intersections_nodes
