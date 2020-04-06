import numpy as np
from operator import itemgetter
import overpy
import dijkstra as dj
import itertools


def get_intersection_nodes(current location = ()):
    api = overpy.Overpass()
    #TODO: change bounding box per each query based on the current location
    result = api.query("""
    <osm-script>
    <query type="way" into="hw">
  <has-kv k="highway"/>
  <has-kv k="highway" modv="not" regv="footway|cycleway|path|service|track"/>
   <bbox-query e="7.157" n="50.748" s="50.746" w="7.154"/> 
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
    nodes = []
    for node in result.nodes:
        nodes.append((node.lat,node.lon))
    return nodes


def path_distance_minimization(point1, point2):
    sub = np.subtract(point1, point2)
    return (sub.item(0) ** 2 + sub.item(1) ** 2) ** 0.5


def calculate_upper_k(node2, seg1, k, n):
    return np.add((np.subtract(node2, seg1)) * ((k/n) * path_distance_minimization(node2, seg1)), seg1)


def calculate_lower_p(node1, node2, upper_k):
    d = np.subtract(node2, node1)
    return np.dot(d, np.subtract(upper_k, node1))


def calculate_function_r(node1, node2, upper_k, lower_p):
    if lower_p < 0:
        return path_distance_minimization(upper_k, node1)
    if lower_p > 1:
        return path_distance_minimization(upper_k, node2)
    return path_distance_minimization(upper_k, np.add(node1, lower_p * np.subtract(node2, node1)))


def distance_sum_minimization(node1, node2, seg1, seg2, n):
    result = 0
    for k in range(n+1):
        upper_k = calculate_upper_k(node2, seg1, k, n)
        lower_p = calculate_lower_p(node1, node2, upper_k)
        result += calculate_function_r(node1, node2, upper_k, lower_p)
    return result


def cost_function(node1, node2, seg1, seg2, alpha, beta, gamma):
    return alpha * path_distance_minimization(node2, seg2) + beta * path_distance_minimization(node1, node2) \
        + gamma * distance_sum_minimization(node1, node2, seg1, seg2)


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
        g.add_edge(x[0], x[1], cost_function(x[0], x[1], seg1, seg2, 1, 1, 1))
        g.add_edge(x[1], x[0], cost_function(x[1], x[0], seg1, seg2, 1, 1, 1))
    # print('Graph data:')
    # for v in g:
    #     for w in v.get_connections():
    #         vid = v.get_id()
    #         wid = w.get_id()
    #         print('( %s , %s, %3d)' % (vid, wid, v.get_weight(w)))
    return g


def run_dijkstra(graph, source, target):
    target_vertex = graph.get_vertex(target)
    dj.dijkstra(graph, graph.get_vertex(source), target_vertex)
    path = [target_vertex.get_id()]
    dj.shortest(target_vertex, path)
    # print('The shortest path : %s' % (path[::-1]))
    return path[::-1]


def algorithm(current_location=[9,9], nodes=get_intersection_nodes(), segments=[[(1,2),(3,4)]]):
    current_location = get_starting_node(current_location, nodes)
    path = []
    while segments:
        next_segment = get_next_segment(segments)
        graph = initialize_graph_for_dijkstra(nodes, next_segment[0], next_segment[1])
        node_near_segment = get_segment_nearest_node(next_segment[1], nodes)
        path.append(run_dijkstra(graph, current_location, node_near_segment))
        current_location = node_near_segment

    print(path)


# algorithm()
