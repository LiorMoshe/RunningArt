import numpy as np
from operator import itemgetter
import collections
import dijkstra as dj
import itertools


def path_distance_minimization(point1, point2):
    sub = np.subtract(point1, point2)
    return (sub.item(0) ** 2 + sub.item(1) ** 2) ** 0.5


def distance_sum_minimization(node1, node2, seg1, seg2):
    pass


def cost_function(node1, node2, seg1, seg2):
    return path_distance_minimization(node1, node2)


def get_starting_node(current_location, nodes):
    return min([[p, path_distance_minimization(current_location, p)] for p in nodes], key=itemgetter(1))[0]


def get_next_segment(segments):
    #TODO: return correct segment
    #TODO: delete the next segment from the segments list
    seg = segments[0]
    segments.remove(seg)
    return seg


def get_segment_nearest_node(segment, nodes):
    return min([[p, path_distance_minimization(segment, p)] for p in nodes], key=itemgetter(1))[0]


def initialize_graph_for_dijkstra(nodes, seg1, seg2):
    g = dj.Graph()
    for x in list(itertools.combinations(nodes, 2)):
        g.add_edge(x[0], x[1], cost_function(x[0], x[1], seg1, seg2))
        g.add_edge(x[1], x[0], cost_function(x[1], x[0], seg1, seg2))
    print('Graph data:')
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print('( %s , %s, %3d)' % (vid, wid, v.get_weight(w)))
    return g


def run_dijkstra(graph, source, target):
    target_vertex = graph.get_vertex(target)
    dj.dijkstra(graph, graph.get_vertex(source), target_vertex)
    path = [target_vertex.get_id()]
    dj.shortest(target_vertex, path)
    print('The shortest path : %s' % (path[::-1]))
    return path[::-1]


def algorithm(current_location=[9,9], nodes=[(8,8), (7,7)], segments=[(1,2),(3,4)]):
    current_location = get_starting_node(current_location, nodes)
    path = []
    while segments:
        next_segment = get_next_segment(segments)
        graph = initialize_graph_for_dijkstra(nodes, next_segment[0], next_segment[1])
        node_near_segment = get_segment_nearest_node(next_segment[1], nodes)
        path.append(run_dijkstra(graph, current_location, node_near_segment))
        current_location = node_near_segment

    print(path)


algorithm()




