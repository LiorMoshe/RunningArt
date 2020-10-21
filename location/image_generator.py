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
from location.spread_matrix import nearestOne
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

def generate_binary_matrix(coordinates, distance, nodes_manager,freq=0.001, scale=1):
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

    neighboring_nodes = nodes_manager.get_neighboring_nodes()

    # Go over each pair of neighboring nodes.
    for pair in neighboring_nodes:
        first_node, second_node = pair
        first_node = (first_node.lat, first_node.lon)
        second_node = (second_node.lat, second_node.lon)

        fill_image_based_on_road(first_node, second_node, binary_data, southwest, dimension, scale, freq, nodes_indices, True)

    print('function took {:.3f} ms'.format((time.time() - start_time) * 1000.0))
    return binary_data, nodes_indices


def save_image(image_name, image_data):
    img = Image.new('1', (image_data.shape[0], image_data.shape[1]))
    pixels = img.load()
    for i in range(img.size[0]):
        for j in range(img.size[1]):
            tmp = int(image_data[i, j].item())
            pixels[i, j] = tmp

    # TODO- Given image is rotated by 90 degrees.
    img = img.transpose(Image.ROTATE_90)
    img.save(image_name)
segments_data = [((Decimal('32.05949153225217429508120403625071048736572265625'), Decimal('34.7689903147888088597028399817645549774169921875')), (Decimal('32.05897201385528916262046550400555133819580078125'), Decimal('34.7689764726551260309861390851438045501708984375'))), ((Decimal('32.05897201385528916262046550400555133819580078125'), Decimal('34.7689764726551260309861390851438045501708984375')), (Decimal('32.0584154316375844473441247828304767608642578125'), Decimal('34.76899031471020151684570009820163249969482421875'))), ((Decimal('32.0584154316375844473441247828304767608642578125'), Decimal('34.76899031471020151684570009820163249969482421875')), (Decimal('32.058358296501097584041417576372623443603515625'), Decimal('34.76949840190848561860548215918242931365966796875'))), ((Decimal('32.058358296501097584041417576372623443603515625'), Decimal('34.76949840190848561860548215918242931365966796875')), (Decimal('32.058358296501097584041417576372623443603515625'), Decimal('34.7699479656932197713103960268199443817138671875'))), ((Decimal('32.058358296501097584041417576372623443603515625'), Decimal('34.7699479656932197713103960268199443817138671875')), (Decimal('32.058358296501097584041417576372623443603515625'), Decimal('34.76949840190848561860548215918242931365966796875'))), ((Decimal('32.058358296501097584041417576372623443603515625'), Decimal('34.76949840190848561860548215918242931365966796875')), (Decimal('32.0584154316375844473441247828304767608642578125'), Decimal('34.76899031471020151684570009820163249969482421875'))), ((Decimal('32.0584154316375844473441247828304767608642578125'), Decimal('34.76899031471020151684570009820163249969482421875')), (Decimal('32.05897201385528916262046550400555133819580078125'), Decimal('34.7689764726551260309861390851438045501708984375'))), ((Decimal('32.05897201385528916262046550400555133819580078125'), Decimal('34.7689764726551260309861390851438045501708984375')), (Decimal('32.05897377235773859638356952928006649017333984375'), Decimal('34.76959622351788681271500536240637302398681640625'))), ((Decimal('32.05897377235773859638356952928006649017333984375'), Decimal('34.76959622351788681271500536240637302398681640625')), (Decimal('32.05897201385528916262046550400555133819580078125'), Decimal('34.7689764726551260309861390851438045501708984375'))), ((Decimal('32.05897201385528916262046550400555133819580078125'), Decimal('34.7689764726551260309861390851438045501708984375')), (Decimal('32.05949153225217429508120403625071048736572265625'), Decimal('34.7689903147888088597028399817645549774169921875'))), ((Decimal('32.05949153225217429508120403625071048736572265625'), Decimal('34.7689903147888088597028399817645549774169921875')), (Decimal('32.0595452856531863972122664563357830047607421875'), Decimal('34.76954978937900619939682655967772006988525390625'))), ((Decimal('32.0595452856531863972122664563357830047607421875'), Decimal('34.76954978937900619939682655967772006988525390625')), (Decimal('32.0595452856531863972122664563357830047607421875'), Decimal('34.76989610413392028931411914527416229248046875')))]

if __name__=="__main__":
    preprocessing_ways(coordinates=(32.05774, 34.76808,32.06134, 34.77231))
    # segments = [(32.059545907506724, 34.769338084235166), (32.058984780936086, 34.769338084235166),
    #  (32.058563936008106, 34.769338084235166), (32.058283372722784, 34.769338084235166),
    #  (32.05800280943746, 34.769338084235166), (32.05772224615214, 34.769338084235166),
    #  (32.057441682866816, 34.769338084235166), (32.057161119581494, 34.769338084235166),
    #  (32.05688055629617, 34.769338084235166), (32.05659999301085, 34.769338084235166),
    #  (32.056325246224254, 34.76946403594408), (32.056192647003066, 34.77014379439682),
    #  (32.056192647003066, 34.770805868793545), (32.056192647003066, 34.771302424591084),
    #  (32.056192647003066, 34.7716708595883)]
    #
    # seg_img = generate_segments_image(segments, (32.05412772649942, 34.76298168819518, 32.06494947098929, 34.775689240965605),
    #                         int(0.6*1000*2))
    #
    # save_image("mycoolimg.png", seg_img)