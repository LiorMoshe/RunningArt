import logging
import math
import os.path
from decimal import Decimal
from scipy import spatial
from geopy.distance import geodesic

# formatter = logging.Formatter('%(message)s')

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

nearest_nodes = {}
minus_id = -1
float_nodeid_to_loc = {}


def get_nodes_map(node_manager):
    nodes_id_to_location = node_manager.get_nodes_map()
    for id, node_loc in nodes_id_to_location.items():
        float_nodeid_to_loc[id] = [float(node_loc[0]), float(node_loc[1])]

    return float_nodeid_to_loc


def cartesian(latitude, longitude, elevation = 0):
    # Convert to radians
    latitude = latitude * (math.pi / 180)
    longitude = longitude * (math.pi / 180)
    R = 6371  # 6378137.0 + elevation  # relative to centre of the earth
    X = R * math.cos(latitude) * math.cos(longitude)
    Y = R * math.cos(latitude) * math.sin(longitude)
    Z = R * math.sin(latitude)
    return (X, Y, Z)


def write_nodes_to_file(current_location=[], filename=".resources/nodes.txt"):
    nodes = get_intersection_nodes(current_location)
    with open(filename, 'w') as nodes_file:
        for node in nodes:
            nodes_file.write(str(node[0]) + "," + str(node[1]) + '\n')

        nodes_file.close()
    return nodes


def get_intersection_nodes_from_file(current_location=[], filename="./resources/nodes.txt"):
    if os.path.isfile(filename):
        try:
            decimal_nodes = []
            places = []
            nodes = []
            with open(filename, 'r') as nodes_file:
                for line in nodes_file.readlines():
                    splitted_line = line.split()
                    cartesian_coord = cartesian(float(splitted_line[0]), float(splitted_line[1]))
                    places.append(cartesian_coord)
                    decimal_nodes.append((Decimal(splitted_line[0]), Decimal(splitted_line[1])))
                    nodes.append([splitted_line[0], splitted_line[1]])
            tree = spatial.KDTree(places)
            return decimal_nodes, nodes, tree
        except Exception:
            pass
    else:
        return write_nodes_to_file(current_location, filename)


def compute_average_distance(nodes_manager):
    """
    Given the available ways from each node, compute the total average distance.
    :return:
    """
    total_roads = 0
    total_dist = 0
    intersection_nodes = nodes_manager.get_intersection_nodes()
    nodes_id_to_location = nodes_manager.get_nodes_map()
    nodes_ways = nodes_manager.get_nodes_ways()
    for node_id, available_ways in nodes_manager.get_nodes_ways().items():
        if node_id in intersection_nodes:
            logging.info("Node Id: {0}, Ways: {1}".format(node_id, len(available_ways)))
            node = nodes_id_to_location[node_id]
            for other_node_id in available_ways:
                if other_node_id in intersection_nodes:
                    logging.info("Way between, Start: {0}, End: {1}".format(node_id, other_node_id))
                    other_node = nodes_id_to_location[other_node_id]
                    total_dist += geodesic(node, other_node).meters
                    total_roads += 1
    logging.info("Intersection Nodes: {0}, Total Roads: {1}, Node Ways: {2}".format(len(intersection_nodes), total_roads, len(nodes_ways)))
    print(total_dist / total_roads)
    return total_dist / total_roads


def append_ids_to_paths(dijkstra_paths, node_manager):
    """
    Append the node ids to the paths returned by our algorithm.
    :param dijkstra_paths:
    :return:
    """
    updated_paths = []
    location_to_id = node_manager.get_location_to_id_map()
    for path in dijkstra_paths:
        updated_path = []
        for point in path:
            updated_path.append({"id": location_to_id[(point[0], point[1])], "loc": point})
        updated_paths.append(updated_path)
    return updated_paths