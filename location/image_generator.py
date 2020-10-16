from decimal import Decimal

import overpy
from geopy.distance import geodesic
import geopy
import math
from PIL import Image
import time
import numpy as np
from algorithm.osm import bbox_calculation
from segments.utils import compute_latlong_angle
from location.convolution import find_starting_location

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

def get_node_index(starting_pos, node, dimension, scale):
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
        row = int(dist * math.cos(angle) / scale)
        col = int(dist * math.sin(angle) / scale)
        if row >= dimension or col >= dimension:
            return None
        return (row,col)
    else:
        return None

def get_position_from_indices(starting_pos, indices, scale):
    result = geopy.distance.distance(kilometers=(scale/1000) * indices[0]).destination(
                starting_pos, 0)

    result = geopy.distance.distance(kilometers=(scale / 1000) * indices[1]).destination(
        result, 90)

    return (result.latitude, result.longitude)

def fill_image_based_on_road(start_point, end_point, image_data, image_start_pos, dimension, scale, freq=0.001, nodes_indices=[],
                             save_nodes=False):
    """
    Fill the data of a given image based on a road between two points.
    :param start_point:
    :param end_point:
    :param image_data:
    :param image_start_pos:
    :param dimension:
    :param scale:
    :return:
    """
    start_indices = get_node_index(image_start_pos, start_point, dimension, scale)
    end_indices = get_node_index(image_start_pos, end_point, dimension, scale)

    # Mark the indices of the nodes as ones that we have roads in them.
    if start_indices is not None:
        if save_nodes:
            nodes_indices.append(start_indices)
        image_data[start_indices[0], start_indices[1]] = 1

    if end_indices is not None:
        if save_nodes:
            nodes_indices.append(end_indices)
        image_data[end_indices[0], end_indices[1]] = 1

    start = None
    end = None
    if start_indices is not None and end_indices is not None:
        start = start_point
        end = end_point
    elif start_indices is None and end_indices is not None:
        start = end_point
        end = start_point
    elif start_indices is not None and end_indices is None:
        start = start_point
        end = end_point

    # Iterate over the linear line connected both nodes and mark it as a part of the road.
    # Size of each jump in the line is determined based on the size of freq.
    if start is not None and end is not None:
        nodes_angle = compute_latlong_angle(start[0], start[1], end[0], end[1])
        nodes_dist = geodesic(start, end).kilometers
        jumps = int(nodes_dist / freq)
        for i in range(1, jumps + 1):
            dest = geopy.distance.distance(kilometers=freq * i).destination(
                start, nodes_angle)
            dest = (dest.latitude, dest.longitude)
            dest_indices = get_node_index(image_start_pos, dest, dimension, scale)
            if dest_indices is None:
                break

            image_data[dest_indices[0], dest_indices[1]] = 1


def generate_segments_image(segments, coordinates, dimension, scale=1, freq=0.001):
    """
    Given the current segments, generate the matching image starting from the southwest corner.
    We must create an image of a similar scale to the scale of the image of the total area.

    TODO- Return initial segment location in the future, not all routes start at (0,0).

    :param segments: Ordered points of segments.
    :return: Image and location
    """
    image_data = np.zeros((dimension, dimension))
    southwest = (coordinates[0], coordinates[1])

    if len(segments) == 0:
        return image_data

    point = segments[0]
    for i in range(1,len(segments)):
        next_point = segments[i]
        fill_image_based_on_road(point, next_point, image_data, southwest, dimension, scale, freq)
        point = next_point

    # Take the same image and substract it to a smaller matrix.
    min_row = float('inf')
    max_row = float('-inf')
    min_col = float('inf')
    max_col = float('-inf')

    for i in range(image_data.shape[0]):
        for j in range(image_data.shape[1]):
            if image_data[i][j]:
                if i < min_row:
                    min_row = i

                if i > max_row:
                    max_row = i

                if j < min_col:
                    min_col = j

                if j > max_col:
                    max_col = j


    if min_row == max_row:
        max_row += 1

    if min_col == max_col:
        max_col += 1

    return image_data[min_row:max_row, min_col:max_col]

def generate_binary_matrix(coordinates, distance, freq=0.001, scale=1):
    """
    Initialize the full matrix to be zeroes and go over only the relevant roads.
    Assumes that the relevant ways were already processed and pairs of neighboring nodes
    are saved in neighboring_nodes.
    :return:
    """
    start_time = time.time()
    dimension = int(distance * 1000 * 2 / scale)
    southwest = (coordinates[0], coordinates[1])
    binary_data = np.zeros((dimension, dimension))

    # Holds all the indices of the nodes in the created image.
    nodes_indices = []


    # Go over each pair of neighboring nodes.
    for pair in neighboring_nodes:
        first_node, second_node = pair
        first_node = (first_node.lat, first_node.lon)
        second_node = (second_node.lat, second_node.lon)

        fill_image_based_on_road(first_node, second_node, binary_data, southwest, dimension, scale, freq, nodes_indices, True)

    print('function took {:.3f} ms'.format((time.time() - start_time) * 1000.0))
    return binary_data, nodes_indices

segments_data = [((Decimal('32.05955764855389844569799606688320636749267578125'), Decimal('34.76934540354852032351118396036326885223388671875')), (Decimal('32.05899271950649875861927284859120845794677734375'), Decimal('34.76934540354852032351118396036326885223388671875'))), ((Decimal('32.05899271950649875861927284859120845794677734375'), Decimal('34.76934540354852032351118396036326885223388671875')), (Decimal('32.05842779045909907154054963029921054840087890625'), Decimal('34.76934540354852032351118396036326885223388671875'))), ((Decimal('32.05842779045909907154054963029921054840087890625'), Decimal('34.76934540354852032351118396036326885223388671875')), (Decimal('32.0580012376790222106137662194669246673583984375'), Decimal('34.7694162528002408407701295800507068634033203125'))), ((Decimal('32.0580012376790222106137662194669246673583984375'), Decimal('34.7694162528002408407701295800507068634033203125')), (Decimal('32.0579521097425867992569692432880401611328125'), Decimal('34.76999991393653743898539687506854534149169921875'))), ((Decimal('32.0579521097425867992569692432880401611328125'), Decimal('34.76999991393653743898539687506854534149169921875')), (Decimal('32.0579521097425867992569692432880401611328125'), Decimal('34.7705151785426238575382740236818790435791015625')))]

if __name__=="__main__":
    #preprocess_segments.
    curr_segments = []
    for seg in segments_data:
        print(seg[0])
        curr_segments.append((float(seg[0][0]), float(seg[0][1])))
    curr_segments.append((float(segments_data[len(segments_data)-1][1][0]),float(segments_data[len(segments_data)-1][1][1])))


    location_node = (32.05954608820065, 34.770199096548296)
    distance = 0.2
    coordinates = bbox_calculation(location_node, distance)
    print("Coordinates ", coordinates)
    segments_image = generate_segments_image(curr_segments, coordinates, int(distance * 1000 * 2), 1)


    # Process all the relevant ways.
    preprocessing_ways(coordinates)

    # Compute the binary matrix and display it.
    binary_data, indices = generate_binary_matrix(coordinates, distance, scale=1)
    opt = find_starting_location(binary_data, segments_image, indices, (0,0))
    print(get_position_from_indices((coordinates[0], coordinates[1]), opt, 1))
    # img = Image.new('1', (binary_data.shape[0], binary_data.shape[1]))
    # img = Image.new('1', (segments_image.shape[0], segments_image.shape[1]))
    # pixels = img.load()
    # for i in range(img.size[0]):
    #     for j in range(img.size[1]):
    #         tmp = int(segments_image[i, j].item())
    #         pixels[i, j] = tmp
    #
    # # TODO- Given image is rotated by 90 degrees.
    # img = img.transpose(Image.ROTATE_90)
    # img.show()
