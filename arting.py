import numpy as np
from operator import itemgetter
import overpy
import dijkstra as dj
import itertools
from decimal import Decimal
from copy import deepcopy
import os.path
import math
from scipy import spatial
from segments import cv_contours as geo
from geopy.distance import geodesic
import networkx as nx

nearest_nodes = {}

# Maps the way
nodes_ways = {}
intersections_nodes_idx = [286643475, 317214411, 357500272, 357545243, 366653136, 799417137, 286643440, 286643458, 286643460, 366651300, 366652380, 366652746, 366653067, 366653799, 1571764097, 1628430688, 1628430723, 4170418930, 366652962, 540420234, 540420265, 540420291, 366654065, 366654066, 540419840, 2470061832, 406399586, 540419838, 540419855, 1574692678, 2294948482, 540419958, 286643465, 286741983, 549271109, 1574692741, 1574692746, 1574692918, 286542239, 286542525, 286543443, 286754329, 496176171, 1628430716, 1672351167, 4582891013, 496176315, 496176455, 799417353, 366653165, 366653693, 1628430719, 540421284, 540421320, 1628430692, 286643451, 357536696, 366651462, 286643444, 366651463, 357538387, 1672351158, 2108063257, 357538922, 357536485, 366651303, 366651349, 496176172, 540420824, 366652262, 366652516, 496176174, 2139244077, 2470061834, 1628430689, 1628430687, 1628430710, 1628430720, 2470061831, 412522566, 496176177, 2470061851, 2469958099, 286643432, 4833025980, 2139244073, 7052661053, 514357166, 366649858, 384695042, 1995922116, 1995922128, 1995922151, 2470061837, 3999875641]
# Maps node is to location
nodes_id_to_location = {}

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
    return result.ways

def initialize_ways_graph(ways):
    for way in ways:
        way_nodes = way.nodes
        # copied_way = deepcopy(way_nodes)
        for idx, node in enumerate(way_nodes):
            # copied_way.remove(node)
            if node.id in intersections_nodes_idx:
                if node.id not in nodes_id_to_location:
                    nodes_id_to_location[node.id] = (node.lat, node.lon)
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



                # if next_node.id not in nodes_id_to_location:
                #     nodes_id_to_location[next_node.id] = (next_node.lat, next_node.lon)
                #     nodes_ways[next_node.id] = []
                #
                # nodes_ways[next_node.id].append(node.id)
            # for other_node in copied_way:
            #     if other_node.id not in nodes_ways[node.id]:
            #         nodes_ways[node.id].append(other_node.id)
            # copied_way.append(node)

def cartesian(latitude, longitude, elevation = 0):
    # Convert to radians
    latitude = latitude * (math.pi / 180)
    longitude = longitude * (math.pi / 180)
    R = 6371 # 6378137.0 + elevation  # relative to centre of the earth
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
                    length_sum = length_sum + geodesic((node[0],node[1]), (nodes[idx][0],nodes[idx][1])).meters
                    #length_sum = length_sum + geo.get_lat_long_dist(float(node[0]), float(node[1]), float(nodes[idx][0]),float(nodes[idx][1]))
                    break
            else:
                calculated_nodes[node_idx] = idx
                # length_sum = length_sum + distances[counter]
                # same output with above line
                length_sum = length_sum + geodesic((node[0], node[1]), (nodes[idx][0], nodes[idx][1])).meters
                #length_sum = length_sum + geo.get_lat_long_dist(float(node[0]), float(node[1]), float(nodes[idx][0]), float(nodes[idx][1]))
                break
            counter = counter + 1
        node_idx = node_idx + 1
    indexes, distances = find_nearest_nodes(float(nodes[node_idx][0]), float(nodes[node_idx][1]), tree)
    nearest_nodes[node_idx] = indexes
    return length_sum/node_idx


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
            with open(filename,'r') as nodes_file:
                for line in nodes_file.readlines():
                    splitted_line = line.split()
                    cartesian_coord = cartesian(float(splitted_line[0]), float(splitted_line[1]))
                    places.append(cartesian_coord)
                    decimal_nodes.append((Decimal(splitted_line[0]),Decimal(splitted_line[1])))
                    nodes.append([splitted_line[0], splitted_line[1]])
            tree = spatial.KDTree(places)
            return decimal_nodes, nodes, tree
        except Exception:
            pass
    else:
        return write_nodes_to_file(current_location,filename)


'''
Get the nodes out of osm's map. Currently it is fixed.
'''
def get_intersection_nodes(current_location=[]):
    api = overpy.Overpass()
    #TODO: change bounding box per each query based on the current location
    #TODO: Loading this information when the user enters to the app

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
    sub = np.subtract(point1, point2)
    return (sub.item(0) ** 2 + sub.item(1) ** 2) ** Decimal(0.5)


def calculate_upper_k(node2, seg1, k, n):
    return np.add((np.subtract(node2, seg1)) * (Decimal((k/n)) * path_distance_minimization(node2, seg1)), seg1)


def calculate_lower_p(node1, node2, upper_k):
    d = np.subtract(node2, node1)
    return np.dot(d, np.subtract(upper_k, node1))


def calculate_function_r(node1, node2, upper_k, lower_p):
    if lower_p < 0:
        return path_distance_minimization(upper_k, node1)
    if lower_p > 1:
        return path_distance_minimization(upper_k, node2)
    return path_distance_minimization(upper_k, np.add(node1, lower_p * np.subtract(node2, node1)))


def distance_sum_minimization(node1, node2, seg1, seg2, n=1):
    result = 0
    for k in range(n+1):
        upper_k = calculate_upper_k(node2, seg1, k, n)
        lower_p = calculate_lower_p(node1, node2, upper_k)
        result += calculate_function_r(node1, node2, upper_k, lower_p)
    return result

nodes_length_dict ={}
def cost_function(node1, node2, seg1, seg2, alpha, beta, gamma):
    c1 = alpha * path_distance_minimization(node2, seg2)
    if (node1, node2) in nodes_length_dict.keys():
        c2 = nodes_length_dict[(node1,node2)]
    else:
        c2 = beta * path_distance_minimization(node1, node2)
        nodes_length_dict[(node1, node2)] = c2

    return c1 + c2
    # \
        # + gamma * distance_sum_minimization(node1, node2, seg1, seg2)


def get_starting_node(current_location, nodes):
    return min([[p, path_distance_minimization(current_location, p)] for p in nodes], key=itemgetter(1))[0]


def get_next_segment(segments):
    seg = segments[0]
    segments.remove(seg)
    return seg

def compute_average_distance():
    """
    Given the available ways from each node, compute the total average distance.
    :return:
    """
    total_roads = 0
    total_dist = 0
    for node_id, available_ways in nodes_ways.items():
        if node_id in intersections_nodes_idx:
            node = nodes_id_to_location[node_id]
            for other_node_id in available_ways:
                if other_node_id in intersections_nodes_idx:
                    other_node = nodes_id_to_location[other_node_id]
                    total_dist += geodesic(node, other_node).meters
                    total_roads += 1
    print(total_dist / total_roads)
    return total_dist / total_roads

def get_segment_nearest_node(segment, nodes):
    return min([[p, path_distance_minimization(segment, p)] for p in nodes], key=itemgetter(1))[0]


def initialize_graph_for_dijkstra(seg1, seg2):
    g = nx.DiGraph()
    counter = 0
    for id, node in nodes_id_to_location.items():
        if id in intersections_nodes_idx:
            for other_node_id in nodes_ways[id]:
                if other_node_id in intersections_nodes_idx:
                    g.add_edge(node, nodes_id_to_location[other_node_id],
                               weight=cost_function(node, nodes_id_to_location[other_node_id], seg1, seg2, 1, 1, 1))
        # for idx in nearest_nodes[counter][:5]:
        #     g.add_edge(node, nodes[idx], weight=cost_function(node, nodes[idx], seg1, seg2, 1, 1, 1))
        counter += 1

    # for x in list(itertools.combinations(nodes, 2)):
    #     g.add_edge(x[0], x[1], weight=cost_function(x[0], x[1], seg1, seg2, 1, 1, 1))
    #     g.add_edge(x[1], x[0], weight=cost_function(x[1], x[0], seg1, seg2, 1, 1, 1))
    return g


def run_dijkstra(graph, source, target):
    return nx.dijkstra_path(graph, source, target)


def algorithm(current_location, segments):
    '''
    Executes dijkstra's algorithm based on the given nodes.
    :param current_location:
    :param segments:
    :param nodes:
    :return:
    '''
    nodes = list(nodes_id_to_location.values())
    current_location = get_starting_node(current_location, nodes)
    path = []
    copied_nodes = deepcopy(nodes)
    while segments:
        print("Number of segments left: ", len(segments))
        next_segment = get_next_segment(segments)
        graph = initialize_graph_for_dijkstra(next_segment[0], next_segment[1])
        copied_nodes.remove(current_location)
        node_near_segment = get_segment_nearest_node(next_segment[1], copied_nodes)
        copied_nodes.append(current_location)
        path.append(run_dijkstra(graph, current_location, node_near_segment))
        current_location = node_near_segment

    print(path)

    float_path = []
    for point in path:
        float_path.append([float(point[0][0]), float(point[0][1])])
    return float_path



if __name__=="__main__":
    print()
    # algorithm()
