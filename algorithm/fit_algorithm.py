from geopy.distance import geodesic

from segments.utils import *
from algorithm.segment import SegmentHistory
import logging
import networkx as nx
import math
from decimal import Decimal


def setup_logger(name, log_file, level=logging.INFO):
    """To setup as many loggers as you want"""
    logging.basicConfig(
        filename='logger.log',
        level=logging.DEBUG,
        format='%(asctime)s.%(msecs)03d %(levelname)s %(module)s - %(funcName)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )

    handler = logging.FileHandler(log_file,mode='w')
    # handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

logger = setup_logger('first_logger', 'logger.log')
logger = logging.getLogger('first_logger')


class FitAlgorithm(object):

    def __init__(self, node_manager):

        self.node_manager = node_manager

        # The list of segments that we perform fitting to.
        self.segments = []

        # The current segment we approximate.
        self.current_segment = None

        # The graph that we perform our search on in each iteration.
        self.graph = None

        # Cache of previous computations.
        self.cache = {}

        # Map each cartesian segment to its geocentric equivalent. TODO- This is temporarily used.
        self.segments_cartesian_to_geo = {}

    def set_segments(self, segments):
        """
        Set the current set of segments that we must perform fitting to.
        :return:
        """
        print("CurrentSegments: {0}".format(segments))
        self.segments = segments

    def search_in_cache(self, segment_history):
        """
        Searches for the given segment history in the cache and returns its matching path
        if there is one.
        :param segment_history:
        :return:
        """

        matching_path = None
        for other_segment_history, segment_path in self.cache.items():
            if segment_history == other_segment_history:
                is_reverse = segment_history.is_reverse(other_segment_history)

                if (other_segment_history.start_node == segment_history.start_node or
                        (is_reverse and other_segment_history.end_node == segment_history.start_node)):
                    logging.info("Found a repeating segment of value: {0}, using the cache. Reverse: {1}".
                                 format(segment_history, is_reverse))
                    matching_path = segment_path if not is_reverse else segment_path[::-1]
        return matching_path

    def initialize_graph_for_dijkstra(self):
        self.graph = nx.DiGraph()
        start_location = self.current_segment[0]
        end_location = self.current_segment[1]

        seg_length = math.sqrt((start_location[0] - end_location[0]) ** 2 + (start_location[1] - end_location[1]) ** 2)
        nodes_id_to_location = self.node_manager.get_nodes_map()
        intersection_nodes_idx = self.node_manager.get_intersection_nodes()
        nodes_ways = self.node_manager.get_nodes_ways()

        for id, node in nodes_id_to_location.items():
            if id in intersection_nodes_idx:
                cartesian_node = gps_to_ecef_custom(float(node[0]), float(node[1]))
                dist = min(math.sqrt((cartesian_node[0] - start_location[0]) ** 2 + (cartesian_node[1] - start_location[1]) ** 2),
                           math.sqrt((cartesian_node[0] - end_location[0]) ** 2 + (cartesian_node[1] - end_location[1]) ** 2))

                # Heuristic- Choose only the nodes which are closer to one of the end point of the segments.
                if (dist <= seg_length * 4):
                    for other_node_id in nodes_ways[id]:
                        if other_node_id in intersection_nodes_idx:
                            logging.info("Adding edge from {0} to {1}, real: {2} {3}".format(id, other_node_id, node,
                                                                                             nodes_id_to_location[
                                                                                                 other_node_id]))
                            self.graph.add_edge(node, nodes_id_to_location[other_node_id],
                                       weight=self.cost_function(node, nodes_id_to_location[other_node_id], start_location, end_location, 1, 1,
                                                            1))

    def cost_function(self,start_node, end_node, start_location, end_location, alpha, beta, gamma):
        start_node = gps_to_ecef_custom(float(start_node[0]), float(start_node[1]))
        end_node = gps_to_ecef_custom(float(end_node[0]), float(end_node[1]))
        c1 = Decimal(alpha * path_distance_minimization(end_node, end_location))
        # todo- Bug in the metric, completely different nodes received different values.
        c3 = distance_sum_minimization(start_node, end_node, start_location, end_location)

        first_slope = (end_node[1] - start_node[1]) / (end_node[0] - start_node[0])
        second_slope = (end_location[1] - start_location[1]) / (end_location[0] - start_location[0])
        angle = math.atan(abs((second_slope - first_slope) / (1 + second_slope * first_slope))) * 180 / math.pi

        multiplier = 1
        if angle > 35:
            multiplier = angle

        total_cost = Decimal(multiplier) * c3 + Decimal(third_metric_ratio) * c1

        x_sign_node = 1 if end_node[0] - start_node[0] > 0 else 0
        y_sign_node = 1 if end_node[1] - start_node[1] > 0 else 0
        x_sign_seg = 1 if end_location[0] - start_location[0] > 0 else 0
        y_sign_seg = 1 if end_location[1] - start_location[1] > 0 else 0
        return total_cost * max(abs(x_sign_seg - x_sign_node), 1) * max(abs(y_sign_seg - y_sign_node), 1)

    def choose_optimal_target(self, current_location, k=10):
        """
        Go over the k closest nodes to the end of the given segment and return the one
        which results in the path with the lowest cost.
        Expects to get the segment in cartesian coordnates.
        :param current_location:
        :param k:
        :return:
        """
        seg_length = geodesic(self.segments_cartesian_to_geo[self.current_segment[0]],
                              self.segments_cartesian_to_geo[self.current_segment[1]]).meters

        end_location = self.current_segment[1]
        segment_as_geo = self.segments_cartesian_to_geo[end_location]
        current_location_float = (float(current_location[0]), float(current_location[1]))
        valid_nodes = []
        nodes = self.node_manager.get_nodes()
        nodes_id_to_location = self.node_manager.get_nodes_map()
        location_to_id = self.node_manager.get_location_to_id_map()

        # Filter out nodes that are at the exact location as the current location.
        for node in nodes:
            node = (float(node[0]), float(node[1]))
            if abs(current_location_float[0] - node[0]) > 1e-30 and \
                    abs(current_location_float[1] - node[1]) > 1e-30:
                valid_nodes.append(node)

        # Compute distance of available nodes from the end of the current segment.
        distance_array = np.array([geodesic(segment_as_geo, p) for p in valid_nodes])
        distance_array = distance_array[distance_array != 0]
        indices = np.argpartition(distance_array, k + 1)
        logging.info("Length of nodes_cartesian: {0}, Distance array: {1}, Indices: {2}".
                     format(len(nodes), len(distance_array), len(indices)))

        logging.info("Lengths of arrays: Distances: {0}, Nodes: {1}".format(len(distance_array), len(nodes)))
        min_total = float('inf')
        min_path = []
        min_node_id = 0
        min_node = None
        starting_id = location_to_id[current_location_float]
        logging.info("Current starting node: {0}".format(starting_id))
        for i in range(k):
            logging.info("K: {0}, Length of indices: {1}".format(k, len(indices)))
            logging.info("Index: {0}, Number of cartesian nodes: {1}".format(indices[i], len(nodes)))
            if valid_nodes[indices[i]] is not None:
                current_node = valid_nodes[indices[i]]
                curr_dist = distance_array[indices[i]]
                node_id = location_to_id[(float(current_node[0]), float(current_node[1]))]
                current_node = nodes_id_to_location[node_id]

                try:
                    logging.info("Checking node id: {0}, Distance: {1}".format(node_id, curr_dist))
                    current_node = (Decimal(current_node[0]), Decimal(current_node[1]))
                    total, path = nx.single_source_dijkstra(self.graph, source=current_location, target=current_node)
                    logging.info("Path to node {0} has total cost of {1}".format(node_id, total))

                    path_len = compute_path_length(path)

                    # Prioritize paths that are of equal length to the segment.
                    logging.info("SegLength: {0} PathLength: {1}".format(seg_length, path_len))
                    logging.info("Difference in path length: {0}".format(abs(path_len - seg_length)))
                    total = total * Decimal(max(abs(path_len - seg_length), 1))

                    logging.info(
                        "After factoring node {0} has total cost of {1}, min total: {2}".format(node_id, total,
                                                                                                min_total))

                    # print("Distance of path: {0}, Distance of segment: {1}".format(path_len, seg_length))
                    # print("Node compared: ", nodes[indices[i]])
                    # print("Node Id: {0} Index: {1} Comparing, total: {2}, min total: {3}, length of path: {4}".format(node_id,indices[i],total, min_total, len(path)))
                    if total < min_total:
                        logging.info("Found new minimal path, Node Id: {0}, Path cost: {1}".format(node_id, total))
                        min_total = total
                        min_path = path
                        min_node_id = node_id
                        min_node = current_node
                except nx.NetworkXNoPath:
                    logging.info("No path to node. Id: {0}, Location: {1}, Distance: {2}".format(node_id, current_node,
                                                                                                 curr_dist))
                    logging.info("LocFrom {0} LocTo {1}".format(current_location, current_node))

        # Check if there is no path to any node.
        if min_node is None:
            print("PROBLEM NO PATH")
            logging.info("No path to any node.")

        logging.info("Chose minimal node: {0} with cost {1}".format(min_node_id, min_total))
        return min_path, min_node

    def step(self, current_location):
        """
        Perform one step of the algorithm.
        Meaning, take the current segment and find the matching path.
        :return:
        """
        segment_history = SegmentHistory(self.current_segment[0], self.current_segment[1])
        segment_history.set_start_node(current_location)

        # Check if there is a matching path in the cache, if there is not choose a target and execute dijkstra search.
        dijkstra_path = self.search_in_cache(segment_history)
        next_node = None
        if dijkstra_path is not None:
            next_node = dijkstra_path[len(dijkstra_path) - 1]
        else:
            logging.info("No cached path for this segment, compute the optimal target.")
            self.initialize_graph_for_dijkstra()

            dijkstra_path, next_node = self.choose_optimal_target(current_location,k=5)

        # Save the computed path in the cache.
        segment_history.set_end_node(next_node)
        self.cache[segment_history] = dijkstra_path
        return dijkstra_path, next_node


    def get_next_segment(self):
        """
        Return the next segment that will be approximated by the algorithm in its cartesian coordinates.
        :return:
        """
        curr_segment = self.segments.pop(0)
        start_segment_geo  = (float(curr_segment[0][0]), float(curr_segment[0][1]))
        end_segment_geo = (float(curr_segment[1][0]), float(curr_segment[1][1]))

        start_cartesian = gps_to_ecef_custom(start_segment_geo[0], start_segment_geo[1])
        end_cartesian = gps_to_ecef_custom(end_segment_geo[0], end_segment_geo[1])
        self.segments_cartesian_to_geo[start_cartesian] = start_segment_geo
        self.segments_cartesian_to_geo[end_cartesian] = end_segment_geo
        return [start_cartesian, end_cartesian]


    def algorithm(self, current_location):
        current_location = get_closest_node(current_location, self.node_manager.get_nodes())
        location_to_id = self.node_manager.get_location_to_id_map()
        path = [current_location]
        dijkstra_paths = []
        cnt = 0
        while self.segments:
            self.current_segment = self.get_next_segment()
            dijkstra_path, next_node = self.step(current_location)

            node_ids = []
            for loc in dijkstra_path:
                node_ids.append(location_to_id[(float(loc[0]), float(loc[1]))])

            logger.info("Path Num: {0}, Path: {1}, Node Ids: {2}".format(cnt, dijkstra_path, node_ids))
            if cnt == 0:
                dijkstra_paths.append([[float(point[0]), float(point[1])] for point in dijkstra_path])
            else:
                dijkstra_paths.append([[float(point[0]), float(point[1])] for point in dijkstra_path[1:]])

            for idx, point in enumerate(dijkstra_path):
                if idx > 0:
                    path.append(point)

            current_location = next_node
            cnt += 1

        float_path = []
        for point in path:
            float_path.append([float(point[0]), float(point[1])])

        return float_path, dijkstra_paths

    def reset(self):
        self.segments = []
        self.cache = {}



