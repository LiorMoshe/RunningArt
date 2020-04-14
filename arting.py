import numpy as np
from operator import itemgetter
import overpy
import itertools
from decimal import Decimal
from copy import deepcopy
import os.path
import networkx as nx
from datetime import datetime
import sys

nodes_length_dict = {}
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
            nodes = []
            with open(filename,'r') as nodes_file:
                for line in nodes_file.readlines():
                    splitted_line = line.replace('\n','').split(',')
                    nodes.append((Decimal(splitted_line[0]),Decimal(splitted_line[1])))

            return nodes
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


def get_segment_nearest_node(segment, nodes):
    return min([[p, path_distance_minimization(segment, p)] for p in nodes], key=itemgetter(1))[0]


def initialize_graph_for_dijkstra(nodes, seg1, seg2):
    g = nx.DiGraph()
    for x in list(itertools.combinations(nodes, 2)):
        g.add_edge(x[0], x[1], weight=cost_function(x[0], x[1], seg1, seg2, 1, 1, 1))
        g.add_edge(x[1], x[0], weight=cost_function(x[1], x[0], seg1, seg2, 1, 1, 1))
    return g


def run_dijkstra(graph, source, target):
    return nx.dijkstra_path(graph, source, target)


def algorithm(current_location, segments, nodes):
    '''
    Executes dijkstra's algorithm based on the given nodes.
    :param current_location:
    :param segments:
    :param nodes:
    :return:
    '''
    current_location = get_starting_node(current_location, nodes)
    path = []

    copied_nodes = deepcopy(nodes)
    while segments:
        print("Number of segments left: ", len(segments))
        next_segment = get_next_segment(segments)
        graph = initialize_graph_for_dijkstra(nodes, next_segment[0], next_segment[1])
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

