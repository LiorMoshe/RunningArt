import logging
import math
import os.path
from decimal import Decimal

from scipy import spatial

from osm.bounding_box_calculation import *

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

minus_id = -1

float_nodeid_to_loc = {}

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