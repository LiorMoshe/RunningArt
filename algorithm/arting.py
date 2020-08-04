import logging
import math
import os.path
from decimal import Decimal
from operator import itemgetter

import networkx as nx
from scipy import spatial
from geopy.distance import geodesic

from algorithm.segment import SegmentHistory
from osm.bounding_box_calculation import *
from segments.utils import is_straight_path, compute_latlong_angle, deg2rad, mod


intersections_nodes_idx = [286643475, 317214411, 357500272, 357545243, 366653136, 799417137, 286643440, 286643458, 286643460, 366651300, 366652380, 366652746, 366653067, 366653799, 1571764097, 1628430688, 1628430723, 4170418930, 366652962, 540420234, 540420265, 540420291, 366654065, 366654066, 540419840, 2470061832, 406399586, 540419838, 540419855, 1574692678, 2294948482, 540419958, 286643465, 286741983, 549271109, 1574692741, 1574692746, 1574692918, 286542239, 286542525, 286543443, 286754329, 496176171, 1628430716, 1672351167, 4582891013, 496176315, 496176455, 799417353, 366653165, 366653693, 1628430719, 540421284, 540421320, 1628430692, 286643451, 357536696, 366651462, 286643444, 366651463, 357538387, 1672351158, 2108063257, 357538922, 357536485, 366651303, 366651349, 496176172, 540420824, 366652262, 366652516, 496176174, 2139244077, 2470061834, 1628430689, 1628430687, 1628430710, 1628430720, 2470061831, 412522566, 496176177, 2470061851, 2469958099, 286643432, 4833025980, 2139244073, 7052661053, 514357166, 366649858, 384695042, 1995922116, 1995922128, 1995922151, 2470061837, 3999875641]

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

# Maps the way
nodes_ways = {}

minus_id = -1

# Maps node is to location
nodes_id_to_location = {}

segments_cartesian_to_geo = {}

float_nodeid_to_loc = {}

# For debugging, location to id.
location_to_id = {}

third_metric_ratio = 50

def get_intersection_nodes_with_ways(current_location=[]):
    api = overpy.Overpass()
    result = api.query("""
     <osm-script>
  <union into="_">
    <query into="_" type="way">
          <bbox-query e="34.773" n="32.060" s="32.057" w="34.768"/>
      <has-kv k="highway" modv="" v="residential"/>
    </query>
    <query into="_" type="way">
          <bbox-query e="34.773" n="32.060" s="32.057" w="34.768"/>
      <has-kv k="highway" modv="" v="tertiary"/>
    </query>
        <query into="_" type="way">
          <bbox-query e="34.773" n="32.060" s="32.057" w="34.768"/>
      <has-kv k="highway" modv="" v="pedestrian"/>
    </query>
  </union>
  <print e="" from="_" geometry="skeleton" ids="yes" limit="" mode="meta" n="" order="id" s="" w=""/>
  <recurse from="_" into="_" type="down"/>
  <print e="" from="_" geometry="skeleton" ids="yes" limit="" mode="meta" n="" order="quadtile" s="" w=""/>
</osm-script>
            """)
    print("done ways")
    return result.ways, result.nodes

def get_nodes_map():
    for id, node_loc in nodes_id_to_location.items():
        float_nodeid_to_loc[id] = [float(node_loc[0]), float(node_loc[1])]

    return float_nodeid_to_loc

def reset_maps():
    """
    Reset the internal mapping.
    :return:
    """
    global minus_id
    minus_id = -1
    nearest_nodes = {}
    nodes_ways = {}
    nodes_id_to_location = {}
    segments_cartesian_to_geo = {}
    float_nodeid_to_loc = {}
    location_to_id = {}

def initialize_ways_graph(ways):
    for way in ways:
        way_nodes = way.nodes
        # copied_way = deepcopy(way_nodes)
        for idx, node in enumerate(way_nodes):
            # copied_way.remove(node)
            if node.id in intersections_nodes_idx:
                if node.id not in nodes_id_to_location:
                    nodes_id_to_location[node.id] = (Decimal(node.lat), Decimal(node.lon))
                    location_to_id[(float(node.lat), float(node.lon))] = node.id
                    if node.id not in nodes_ways.keys():
                        nodes_ways[node.id] = []
                if idx < len(way_nodes) - 1:
                    next_node = way_nodes[idx + 1]
                    if next_node.id in intersections_nodes_idx:
                        nodes_ways[node.id].append(next_node.id)
                        if next_node.id not in nodes_ways.keys():
                            nodes_ways[next_node.id] = []
                        nodes_ways[next_node.id].append(node.id)
                    else:
                        tmp_idx = idx + 1
                        found = False
                        while tmp_idx < len(way_nodes) - 1 and found == False :
                            next_node = way_nodes[tmp_idx + 1]
                            if next_node.id in intersections_nodes_idx:
                                found=True
                                nodes_ways[node.id].append(next_node.id)
                                if next_node.id not in nodes_ways.keys():
                                    nodes_ways[next_node.id] = []
                                nodes_ways[next_node.id].append(node.id)
                            tmp_idx = tmp_idx + 1
    intersections_nodes = get_mid_nodes(intersections_nodes_idx)
    return intersections_nodes

def cartesian(latitude, longitude, elevation = 0):
    # Convert to radians
    latitude = latitude * (math.pi / 180)
    longitude = longitude * (math.pi / 180)
    R = 6371  # 6378137.0 + elevation  # relative to centre of the earth
    X = R * math.cos(latitude) * math.cos(longitude)
    Y = R * math.cos(latitude) * math.sin(longitude)
    Z = R * math.sin(latitude)
    return (X, Y, Z)


def find_nearest_nodes(lat, lon, tree):
    cartesian_coord = cartesian(lat, lon)
    closest = tree.query([cartesian_coord], k=8, p=2)
    return closest[1][0][1:], closest[0][0][1:]


def average_euclidean_distance(tree, nodes):
    calculated_nodes = {}
    length_sum = 0
    node_idx = 0
    for node in nodes[:-1]:
        counter = 0
        indexes, distances = find_nearest_nodes(float(node[0]), float(node[1]), tree)
        nearest_nodes[node_idx] = indexes
        for idx in indexes:
            if idx in calculated_nodes.keys():
                if calculated_nodes[idx] != node_idx:
                    calculated_nodes[node_idx] = idx
                    length_sum = length_sum + geodesic((node[0], node[1]), (nodes[idx][0], nodes[idx][1])).meters
                    # length_sum = length_sum + geo.get_lat_long_dist(float(node[0]), float(node[1]), float(nodes[idx][0]),float(nodes[idx][1]))
                    break
            else:
                calculated_nodes[node_idx] = idx
                # length_sum = length_sum + distances[counter]
                # same output with above line
                length_sum = length_sum + geodesic((node[0], node[1]), (nodes[idx][0], nodes[idx][1])).meters
                # length_sum = length_sum + geo.get_lat_long_dist(float(node[0]), float(node[1]), float(nodes[idx][0]), float(nodes[idx][1]))
                break
            counter = counter + 1
        node_idx = node_idx + 1
    indexes, distances = find_nearest_nodes(float(nodes[node_idx][0]), float(nodes[node_idx][1]), tree)
    nearest_nodes[node_idx] = indexes
    return length_sum / node_idx


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


'''
Get the nodes out of osm's map. Currently it is fixed.
'''
def get_intersection_nodes(current_location=[]):
    api = overpy.Overpass()
    # TODO: change bounding box per each query based on the current location
    # TODO: Loading this information when the user enters to the app

    result = api.query("""
 <osm-script>
<query type="way" into="hw">
  <has-kv k="highway"/>
  <has-kv k="highway" modv="not" regv="footway|cycleway|path|service|track"/>
<bbox-query e="-73.986" n="40.767" s="40.758" w="-73.997"/> 
</query>

<foreach from="hw" into="w">
  <recurse from="w" type="way-node" into="ns"/>
  <recurse from="ns" type="node-way" into="w2"/>
  <query type="way" into="w2">
    <item set="w2"/>
    <has-kv k="highway"/>
    <has-kv k="highway" modv="not" regv="footway|cycleway|path|service|track"/>
  </query>
  <difference into="wd">
    <item set="w2"/>
    <item set="w"/>
  </difference>
  <recurse from="wd" type="way-node" into="n2"/>
  <recurse from="w"  type="way-node" into="n3"/>
  <query type="node">
    <item set="n2"/>
    <item set="n3"/>
  </query>
  <print/>
</foreach>
  </osm-script>
    """)
    return [(node.lat, node.lon) for node in result.nodes]


def path_distance_minimization(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    # sub = np.subtract(point1, point2)
    # return (sub.item(0) ** 2 + sub.item(1) ** 2) ** Decimal(0.5)


def calculate_upper_k(node2, seg1, k, n):
    return np.add((np.subtract(node2, seg1)) * (Decimal((k / n)) * path_distance_minimization(node2, seg1)), seg1)


def calculate_lower_p(node1, node2, upper_k):
    d = np.subtract(node2, node1)
    return np.dot(d, np.subtract(upper_k, node1))


def calculate_function_r(node1, node2, upper_k, lower_p):
    if lower_p < 0:
        return path_distance_minimization(upper_k, node1)
    if lower_p > 1:
        return path_distance_minimization(upper_k, node2)
    return path_distance_minimization(upper_k, np.add(node1, lower_p * np.subtract(node2, node1)))


def distance_sum_minimization(node1, node2, seg1, seg2, n=third_metric_ratio):
    """
    Compute a sum of the distances between the figure segment and two neighboring nodes of the graph.
    This approach is based on the Riemann sum with the idea of computing the area that lies between
    two straight lines.
    :param node1:
    :param node2:
    :param seg1:
    :param seg2:
    :param n:
    :return:
    """
    node1 = (float(node1[0]), float(node1[1]))
    node2 = (float(node2[0]), float(node2[1]))
    seg1 = (float(seg1[0]), float(seg1[1]))
    seg2 = (float(seg2[0]), float(seg2[1]))
    # print("Node 1: ", node1)
    # print("Seg 1: ", seg1)

    nodes_dist = math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)
    seg_dist = math.sqrt((seg1[0] - seg2[0]) ** 2 + (seg1[1] - seg2[1]) ** 2)

    # print("Nodes Dist: {0}, Segments Dist: {1}".format(nodes_dist, seg_dist))
    # print("Nodes length: ", math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2))
    result = 0
    # print("Seg1: ", seg1)
    # print("Seg2: ", seg2)
    tangent = (seg2[1] - seg1[1]) / (seg2[0] - seg1[0])
    if seg1[0] <= seg2[0]:
        sign = 1
    else:
        sign = -1

    eq = lambda val: tangent * (val - seg1[0]) + seg1[1]

    for i in range(1, n + 1):

        diff = (node2[0] - node1[0], node2[1] - node1[1])
        x_val = seg1[0] + (i / n) * sign
        k = (x_val, eq(x_val))

        # k = (seg1[0] + (i / n) * diff[0], node2[1] + (i/n) * diff[1])
        #
        dot_product = diff[0] * (k[0] - node1[0]) + diff[1] * (k[1] - node1[1])
        if dot_product < 0:
            target = (k[0] - node1[0], k[1] - node1[1])
        elif dot_product <= 1:
            target = (k[0] - (node1[0] + dot_product * diff[0]), k[1] - (node1[1] + dot_product * diff[1]))
        else:
            target = (k[0] - node2[0], k[1] - node2[1])

        result += math.sqrt(target[0] ** 2 + target[1] ** 2)

    # return Decimal((seg_dist / nodes_dist) * result)
    return Decimal(result)


def angle_comparison(node1, node2, seg1, seg2):
    # first_slope = node2[1] - node1[1], node2[0] - node1[0]
    # second_slope = seg2[1] - seg1[1], seg2[0] - seg1[0]
    pass


nodes_length_dict = {}


def cost_function(node1, node2, seg1, seg2, alpha, beta, gamma):
    f_node_id = location_to_id[(float(node1[0]), float(node1[1]))]
    s_node_id = location_to_id[(float(node2[0]), float(node2[1]))]
    node1 = gps_to_ecef_custom(float(node1[0]), float(node1[1]))
    node2 = gps_to_ecef_custom(float(node2[0]), float(node2[1]))
    c1 = Decimal(alpha * path_distance_minimization(node2, seg2))
    # if (node1, node2) in nodes_length_dict.keys():
    #     c2 = nodes_length_dict[(node1,node2)]
    # else:
    #     c2 = Decimal(beta * path_distance_minimization(node1, node2))
    #     nodes_length_dict[(node1, node2)] = c2

    # if (abs(float(seg2[0]) - float(seg1[0])) > 1e-8):
    # print("First Seg: {0}, Second Seg: {1}".format(seg1, seg2))
    # todo- Bug in the metric, completely different nodes received different values.
    c3 = distance_sum_minimization(node1, node2, seg1, seg2)
    # print("Metric Computation: Distance: {0} Area: {1}".format(c1, c3))

    first_slope = (node2[1] - node1[1]) / (node2[0] - node1[0])
    second_slope = (seg2[1] - seg1[1]) / (seg2[0] - seg1[0])
    angle = math.atan(abs((second_slope - first_slope) / (1 + second_slope * first_slope))) * 180 / math.pi

    # if (first_slope * second_slope < 0):
    #     return Decimal('Infinity')

    multiplier = 1
    if angle > 35:
        multiplier = angle

    # print("Computing cost of nodes: {0} and {1}, slope mult: {2} Angle:  {3}, Area: {4}, Total Computed Area Factor: {5}".
    #       format(f_node_id, s_node_id, first_slope * second_slope, angle, float(c3), multiplier * float(c3)))

    # logging.info("Node1: (x: {0}, y: {1}), Node2: (x: {2}, y: {3})".format(node1[0], node1[1], node2[0], node2[1]))
    # logger.info("Computing cost of nodes: {0} and {1}, slope mult: {2} Angle:  {3}, Area: {4}, Total Computed: {5}"
    #             ", C1 Factor: {6}".
    #             format(f_node_id, s_node_id, first_slope * second_slope, angle, float(c3), multiplier * float(c3),
    #                    float(third_metric_ratio) * float(c1)))

    total_cost = Decimal(multiplier) * c3 + Decimal(third_metric_ratio) * c1

    x_sign_node = 1 if node2[0] - node1[0] > 0 else 0
    y_sign_node = 1 if node2[1] - node1[1] > 0 else 0
    x_sign_seg = 1 if seg2[0] - seg1[0] > 0 else 0
    y_sign_seg = 1 if seg2[1] - seg1[1] > 0 else 0
    return total_cost * max(abs(x_sign_seg - x_sign_node), 1) * max(abs(y_sign_seg - y_sign_node), 1)

    # return Decimal(0.25) * c1
    # \
    # + gamma * distance_sum_minimization(node1, node2, seg1, seg2)


def get_starting_node(current_location, nodes):
    return min([[p, path_distance_minimization(current_location, p)] for p in nodes], key=itemgetter(1))[0]


def get_next_segment(segments, leftovers):
    if len(leftovers) == 0:
        global segments_cartesian_to_geo
        seg = segments[0]
        segments.remove(seg)
        # Convert to cartesian form.
        start_segment_geo = (float(seg[0][0]), float(seg[0][1]))
        end_segment_geo = (float(seg[1][0]), float(seg[1][1]))

        start_cartesian = gps_to_ecef_custom(start_segment_geo[0], start_segment_geo[1])
        end_cartesian = gps_to_ecef_custom(end_segment_geo[0], end_segment_geo[1])
        segments_cartesian_to_geo[start_cartesian] = start_segment_geo
        segments_cartesian_to_geo[end_cartesian] = end_segment_geo
        return [start_cartesian, end_cartesian]
    else:
        seg = leftovers[0]
        leftovers.remove(seg)
        return seg


def compute_average_distance(intersections_nodes):
    """
    Given the available ways from each node, compute the total average distance.
    :return:
    """
    total_roads = 0
    total_dist = 0
    for node_id, available_ways in nodes_ways.items():
        if node_id in intersections_nodes:
            logging.info("Node Id: {0}, Ways: {1}".format(node_id, len(available_ways)))
            node = nodes_id_to_location[node_id]
            for other_node_id in available_ways:
                if other_node_id in intersections_nodes:
                    logging.info("Way between, Start: {0}, End: {1}".format(node_id, other_node_id))
                    other_node = nodes_id_to_location[other_node_id]
                    total_dist += geodesic(node, other_node).meters
                    total_roads += 1
    logging.info("Intersection Nodes: {0}, Total Roads: {1}, Node Ways: {2}".format(len(intersections_nodes), total_roads, len(nodes_ways)))
    print(total_dist / total_roads)
    return total_dist / total_roads

def get_segment_nearest_node(segment, nodes):
    return min([[p, path_distance_minimization(segment, p)] for p in nodes], key=itemgetter(1))[0]


def initialize_graph_for_dijkstra(seg1, seg2, intersections_nodes_idx):
    g = nx.DiGraph()
    counter = 0
    seg_length = math.sqrt((seg1[0] - seg2[0]) ** 2 + (seg1[1] - seg2[1]) ** 2)
    for id, node in nodes_id_to_location.items():
        if id in intersections_nodes_idx:
            node_car = gps_to_ecef_custom(float(node[0]), float(node[1]))
            dist = min(math.sqrt((node_car[0] - seg1[0]) ** 2 + (node_car[1] - seg1[1]) ** 2),
                       math.sqrt((node_car[0] - seg2[0]) ** 2 + (node_car[1] - seg2[1]) ** 2))

            # Heuristic- Choose only the nodes which are closer to one of the end point of the segments.
            if (dist <= seg_length * 4):
                for other_node_id in nodes_ways[id]:
                    if other_node_id in intersections_nodes_idx:
                        g.add_edge(node, nodes_id_to_location[other_node_id],
                                   weight=cost_function(node, nodes_id_to_location[other_node_id], seg1, seg2, 1, 1, 1))
        counter += 1
    return g


def run_dijkstra(graph, source, target):
    return nx.dijkstra_path(graph, source, target)


def get_mid_nodes(intersections_nodes):
    global minus_id
    initial_id = minus_id
    curr_id = minus_id

    new_ids = {}
    mid_location_to_id = {}

    for node_id, neighbors_ids in nodes_ways.items():
        if node_id in intersections_nodes:
            node_location = nodes_id_to_location[node_id]
            for neighbor_id in neighbors_ids:
                if neighbor_id in intersections_nodes:
                    other_node_loc = nodes_id_to_location[neighbor_id]
                    mid_node = get_mid_point(node_location[0], node_location[1], other_node_loc[0], other_node_loc[1])
                    min_node_decimal = (Decimal(mid_node[0]), Decimal(mid_node[1]))
                    if min_node_decimal not in mid_location_to_id:
                        logging.info("Mid Node Id: {0}, Location: {1}".format(curr_id, min_node_decimal))
                        mid_location_to_id[min_node_decimal] = curr_id
                        new_ids[curr_id] = [node_id, neighbor_id]
                        curr_id -= 1
                        # nodes_id_to_location[curr_id] = (Decimal(mid_node[0]), Decimal(mid_node[1]))
    for loc, id in mid_location_to_id.items():
        nodes_id_to_location[id] = loc
        location_to_id[(loc[0], loc[1])] = id
    for i in range(initial_id, curr_id, -1):
        nodes_ways[i] = new_ids[i]
        intersections_nodes.append(i)

        nodes_ways[new_ids[i][0]].append(i)
        if new_ids[i][1] in nodes_ways[new_ids[i][0]]:
            nodes_ways[new_ids[i][0]].remove(new_ids[i][1])

        nodes_ways[new_ids[i][1]].append(i)
        if new_ids[i][0] in nodes_ways[new_ids[i][1]]:
            nodes_ways[new_ids[i][1]].remove(new_ids[i][0])
    minus_id = curr_id
    return intersections_nodes

def get_mid_point(lat1, lon1, lat2, lon2):
    lat1 = float(lat1) * math.pi / 180
    lat2 = float(lat2) * math.pi / 180
    lon1 = float(lon1) * math.pi / 180
    lon2 = float(lon2) * math.pi / 180
    Bx = math.cos(lat2) * math.cos(lon2 - lon1)
    By = math.cos(lat2) * math.sin(lon2 - lon1)
    latMid = math.atan2(math.sin(lat1) + math.sin(lat2),
                        math.sqrt((math.cos(lat1) + Bx) * (math.cos(lat1) + Bx) + By * By))
    lonMid = lon1 + math.atan2(By, math.cos(lat1) + Bx)
    return (latMid * 180 / math.pi, lonMid * 180 / math.pi)

def choose_optimal_target(graph, current_location, segment, nodes, k=10, seg_length=0.0):
    """
    Go over the k closest nodes to the end of the given segment and return the one
    which results in the path with the lowest cost.
    Expects to get the segment in cartesian coordnates.
    :param graph:
    :param current_location:
    :param segment:
    :param nodes:
    :param k:
    :return:
    """
    # segment_cartesian = gps_to_ecef_custom(float(segment[0]), float(segment[1]))
    # current_location_cartesian = gps_to_ecef_custom(float(current_location[0]), float(current_location[1]))
    # nodes_cartesian = [gps_to_ecef_custom(float(node[0]), float(node[1])) for node in nodes]
    # for p in nodes_cartesian:
    #     dist = math.sqrt((segment[0] - p[0]) ** 2 + (segment[1] - p[1]) ** 2)
    #     logging.info("Node: {0}, Segment: {1}, Dist: {2}".format(segment, p, dist))

    # distance_array = np.array([path_distance_minimization(segment, p) for p in nodes_cartesian])
    segment_as_geo = segments_cartesian_to_geo[segment]
    current_location_float = (float(current_location[0]), float(current_location[1]))

    # Used for debugging.
    # for p in nodes:
    #     curr_p = (float(p[0]), float(p[1]))
    #     curr_dist = geodesic(segment_as_geo, curr_p)
    #     num_id = location_to_id[curr_p]
    #     logging.info("Num Node: {0}, Location: {1}, Segment: {2} ,Coordinates: {3}".format(num_id, curr_dist,
    #                                                                                        segment_as_geo, curr_p))

    # Filter out locations that are exactly on the current location.
    valid_nodes = []
    for p in nodes:
        curr_p = (float(p[0]), float(p[1]))
        if abs(current_location_float[0] - curr_p[0]) > 1e-30 and \
                abs(current_location_float[1] - curr_p[1]) > 1e-30:
            valid_nodes.append(curr_p)

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

            try:
                logging.info("Checking node id: {0}, Distance: {1}".format(node_id, curr_dist))
                total, path = nx.single_source_dijkstra(graph, source=current_location, target=current_node)
                logging.info("Path to node {0} has total cost of {1}".format(node_id, total))

                path_len = compute_path_length(path)

                # Prioritize paths that are of equal length to the segment.
                total = total * Decimal(max(abs(path_len - seg_length), 1))

                logging.info(
                    "After factoring node {0} has total cost of {1}, min total: {2}".format(node_id, total, min_total))

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
                logging.info("No path to node. Id: {0}, Location: {1}, Distance: {2}".format(node_id, current_node, curr_dist))

    # Check if there is no path to any node.
    if min_node is None:
        print("PROBLEM NO PATH")
        logging.info("No path to any node.")

    logging.info("Chose minimal node: {0} with cost {1}".format(min_node_id, min_total))
    return min_path, min_node

def compute_path_length(path):
    dist = 0
    for idx in range(len(path) - 1):
        dist += geodesic((float(path[idx][0]), float(path[idx][1])), (float(path[idx + 1][0]), float(path[idx + 1][1]))).meters
    return dist

def compute_remaining_segment(segment, length, segment_length):
    """
    Given a segment and the length taken out of this segment, return the leftover.
    :param segment:
    :param length:
    :return:
    """
    if length > segment_length:
        raise ValueError("There is no remainer of the segment, path length of {0} is larger than the segment"
                         "length {1}".format(length, segment_length))

    ratio = float(length / segment_length)
    return [(ratio * segment[0][0] + (1 - ratio) * segment[1][0],
             ratio * segment[0][1] + (1 - ratio) * segment[1][1]), segment[1]]


def append_ids_to_paths(dijkstra_paths):
    """
    Append the node ids to the paths returned by our algorithm.
    :param dijkstra_paths:
    :return:
    """
    updated_paths = []
    for path in dijkstra_paths:
        updated_path = []
        for point in path:
            updated_path.append({"id": location_to_id[(point[0], point[1])], "loc": point})
        updated_paths.append(updated_path)
    return updated_paths



def handle_long_paths(path, segment):
    """
    Compute the length of the path and the segment.
    If the path is a straight line and its way longer. Compute the end point and add
    it to the graph with a matching node.
    This happens in cases where the structure of the graph forces us to take paths that are way longer
    than the lengths of the given segment.
    :param path:
    :param segment:
    :return:
    """
    seg_length = math.sqrt((segment[0][0] - segment[1][0]) ** 2 +
                               (segment[0][1] - segment[1][1]) ** 2)
    path_length = compute_path_length(path)
    print("Path Length: {0}, Segment Length: {1}".format(path_length, seg_length))
    logging.info("Path Length: {0}, Segment Length: {1}".format(path_length, seg_length))

    # Decide on some ratio between them.


def algorithm(current_location, segments, threshold=10):
    '''
    Executes dijkstra's algorithm based on the given nodes.
    Throughout the main loop of the algorithm we assume that the given segment is in cartesian coordinates.
    :param current_location:
    :param segments:
    :param nodes:
    :return:
    '''
    nodes = list(nodes_id_to_location.values())
    print("Receiving starting node")
    current_location = get_starting_node(current_location, nodes)
    path = [current_location]
    cnt = 0
    cache = {}
    dijkstra_paths = []
    leftovers = []
    while segments or leftovers:
        print("Number of segments left: ", len(segments))
        logging.info("Number of segments left: {0}".format(len(segments)))
        next_segment = get_next_segment(segments, leftovers)
        segment = SegmentHistory(next_segment[0], next_segment[1])
        segment.set_start_node(current_location)
        graph = initialize_graph_for_dijkstra(next_segment[0], next_segment[1], intersections_nodes_idx)
        logging.info("Finished initializing graph for dijkstra.")
        seg_length = math.sqrt((next_segment[0][0] - next_segment[1][0]) ** 2 +
                               (next_segment[0][1] - next_segment[1][1]) ** 2)

        # Check if the segment is already in cache.
        dijkstra_path = None
        node_near_segment = None
        for other_segment, segment_path in cache.items():
            if segment == other_segment:
                is_reverse = segment.is_reverse(other_segment)

                if (other_segment.start_node == segment.start_node or (is_reverse and other_segment.end_node == segment.start_node)):
                    logging.info("Found a repeating segment of value: {0}, using the cache. Reverse: {1}".
                                 format(segment, is_reverse))
                    dijkstra_path = segment_path if not is_reverse else segment_path[::-1]
                    node_near_segment = dijkstra_path[len(dijkstra_path) - 1]


        if dijkstra_path is None and node_near_segment is None:
            logging.info("No cached path for this segment, compute the optimal target.")
            dijkstra_path, node_near_segment = choose_optimal_target(graph, current_location, next_segment[1], nodes,
                                                                     k=5, seg_length=seg_length)

        # TODO- Use this in the future when I will recall what it does.
        # is_straight = is_straight_path(dijkstra_path)
        # print("Is Straight Path: {0}".format(is_straight))

        # if is_straight:
        #     rotate_segments_based_on_path(segments, dijkstra_path, current_location, segments_cartesian_to_geo[next_segment[1]])


        # handle_long_paths(dijkstra_path,next_segment)
        # if ((seg_length - length) > threshold):
        #     print("PASSED LENGTH TEST")
        #     leftovers.append(compute_remaining_segment(next_segment, length, seg_length))

        # print("Path Length: {0}, Segment Length: {1}".format(length, seg_length))
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

        segment.set_end_node(node_near_segment)
        cache[segment] = dijkstra_path
        current_location = node_near_segment
        cnt += 1


    float_path = []
    for point in path:
        float_path.append([float(point[0]), float(point[1])])

    return float_path, dijkstra_paths

def find_shortest_angle_diff(source, target):
    return mod((target - source +180), 360) - 180


def rotate_segments_based_on_path(segments, path, start_segment, end_segment):
    path_angle = compute_latlong_angle(deg2rad(float(path[0][0])), deg2rad(float(path[0][1])),
                                        deg2rad(float(path[1][0])), deg2rad(float(path[1][1])))

    seg_angle = compute_latlong_angle(deg2rad(float(start_segment[0])), deg2rad(float(start_segment[1])),
                                         deg2rad(float(end_segment[0])), deg2rad(float(end_segment[1])))

    # Need to align the segments according to the paths angle.
    counter_clock_diff = find_shortest_angle_diff(seg_angle, path_angle)
    print("Source Angle: {0}, Target Angle: {1}, Diff: {2}".format(seg_angle, path_angle, counter_clock_diff))



def gps_to_ecef_custom(lat, lon):
    rad_lat = lat * (math.pi / 180.0)
    rad_lon = lon * (math.pi / 180.0)

    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    v = a / math.sqrt(1 - e2 * math.sin(rad_lat) * math.sin(rad_lat))

    x = (v) * math.cos(rad_lat) * math.cos(rad_lon)
    y = (v) * math.cos(rad_lat) * math.sin(rad_lon)

    return (x, y)


if __name__ == "__main__":
    # x = (32.0595662, 34.7693194)
    # y = (33.0595662, 34.7693194)
    # z = geodesic(x, y)
    # path = [(Decimal('32.0595662'), Decimal('34.7693194')), (
    # Decimal('32.05913935001895964660434401594102382659912109375'),
    # Decimal('34.76938890032428020049337646923959255218505859375'))]
    # length = compute_path_length(path)
    # print("length: ", length)

    # paths = [[32.060333, 34.7696562], [32.059973100007376, 34.76969955017055]]
    # seg_start = [32.060333, 34.7696562]
    # end = [32.062333, 34.7656562]
    print(find_shortest_angle_diff(120, 240))

