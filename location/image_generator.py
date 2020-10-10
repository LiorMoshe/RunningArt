import overpy
from geopy.distance import geodesic
import geopy
import math
from PIL import Image
import time
import numpy as np
from algorithm.osm import bbox_calculation
from segments.utils import compute_latlong_angle

# Holds all the pairs of nodes that are next to one another.
neighboring_nodes = []

def intersection_nodes_with_ways(coordinates):
    south_lat, west_long, north_lat, east_long = coordinates
    api = overpy.Overpass(url='https://overpass.kumi.systems/api/interpreter')
    bbox = '<bbox-query e=%s n=%s s=%s w=%s/>' % ('\"'+str(east_long)+'\"','\"'+str(north_lat)+'\"','\"'+str(south_lat)+'\"','\"'+str(west_long)+'\"')
    query = """
     <osm-script>
  <union into="_">
    <query into="_" type="way">
          %s
      <has-kv k="highway" modv="" v="residential"/>
    </query>
    <query into="_" type="way">
          %s
      <has-kv k="highway" modv="" v="tertiary"/>
    </query>
        <query into="_" type="way">
          %s
      <has-kv k="highway" modv="" v="pedestrian"/>
    </query>
  </union>
  <print e="" from="_" geometry="skeleton" ids="yes" limit="" mode="meta" n="" order="id" s="" w=""/>
  <recurse from="_" into="_" type="down"/>
  <print e="" from="_" geometry="skeleton" ids="yes" limit="" mode="meta" n="" order="quadtile" s="" w=""/>
</osm-script>
            """ % (bbox, bbox, bbox)
    print(query)
    result = api.query(query)
    return result.ways, result.nodes

def preprocessing_ways(coordinates):
    """
    Go over all the ways in the bounding box represented by the given coordinates.
    :return: Array of 2 lat-lon locations, first location is southwest point, second is northeast.
    """
    ways, nodes = intersection_nodes_with_ways(coordinates)
    for way in ways:
        way_nodes = way.nodes
        for idx, node in enumerate(way_nodes):
            if idx < len(way_nodes) - 1:
                next_node = way_nodes[idx + 1]
                neighboring_nodes.append((node, next_node))

def get_node_index(starting_pos, node, dimension):
    """
    Given the starting position of our bounding box (southwest location) compute the indices
    of the given node in the binary matrix where the starting position is at (0,0).
    If the indices are out of the given area, return None.
    :param starting_pos:
    :param node:
    :param dimension:
    :return:
    """
    angle = compute_latlong_angle(starting_pos[0], starting_pos[1], float(node[0]), float(node[1]))
    dist = geodesic(starting_pos, (node[0], node[1])).meters
    if angle >= 0 and angle <= 90:
        angle *= math.pi / 180
        row = int(dist * math.cos(angle))
        col = int(dist * math.sin(angle))
        if row >= dimension or col >= dimension:
            return None
        return (row,col)
    else:
        return None

def generate_binary_matrix(coordinates, distance, freq=0.001):
    """
    Initialize the full matrix to be zeroes and go over only the relevant roads.
    Assumes that the relevant ways were already processed and pairs of neighboring nodes
    are saved in neighboring_nodes.

    :return:
    """
    start_time = time.time()
    dimension = int(distance * 1000 * 2)
    southwest = (coordinates[0], coordinates[1])
    binary_data = np.zeros((dimension, dimension))

    # Go over each pair of neighboring nodes.
    for pair in neighboring_nodes:
        first_node, second_node = pair
        first_node = (first_node.lat, first_node.lon)
        second_node = (second_node.lat, second_node.lon)

        first_node_indices = get_node_index(southwest, first_node, dimension)
        second_node_indices = get_node_index(southwest, second_node, dimension)

        # Mark the indices of the nodes as ones that we have roads in them.
        if first_node_indices is not None:
            binary_data[first_node_indices[0], first_node_indices[1]] = 1

        if second_node_indices is not None:
            binary_data[second_node_indices[0], second_node_indices[1]] = 1

        start = None
        end = None
        if first_node_indices is not None and second_node_indices is not None:
            start = first_node
            end = second_node
        elif first_node_indices is None and second_node_indices is not None:
            start = second_node
            end = first_node
        elif first_node_indices is not None and second_node_indices is None:
            start=first_node
            end=second_node

        # Iterate over the linear line connected both nodes and mark it as a part of the road.
        # Size of each jump in the line is determined based on the size of freq.
        if start is not None and end is not None:
            nodes_angle = compute_latlong_angle(start[0], start[1], end[0], end[1])
            nodes_dist = geodesic(start, end).kilometers
            jumps = int(nodes_dist / freq)
            for i in range(1,jumps+1):
                dest = geopy.distance.distance(kilometers=freq * i).destination(
                    start, nodes_angle)
                dest = (dest.latitude, dest.longitude)
                dest_indices = get_node_index(southwest, dest, dimension)
                if dest_indices is None:
                    break

                binary_data[dest_indices[0], dest_indices[1]] = 1

    print('function took {:.3f} ms'.format((time.time() - start_time) * 1000.0))
    return binary_data

if __name__=="__main__":
    location_node = (32.05954608820065, 34.770199096548296)
    distance = 0.2
    coordinates = bbox_calculation(location_node, distance)

    # Process all the relevant ways.
    preprocessing_ways(coordinates)

    # Compute the binary matrix and display it.
    binary_data = generate_binary_matrix(coordinates, distance)
    img = Image.new('1', (binary_data.shape[0], binary_data.shape[1]))
    pixels = img.load()
    for i in range(img.size[0]):
        for j in range(img.size[1]):
            tmp = int(binary_data[i, j].item())
            pixels[i, j] = tmp

    # TODO- Given image is rotated by 90 degrees.
    img = img.transpose(Image.ROTATE_90)
    img.show()
