import math
from decimal import Decimal

from PIL import ImageFont, Image, ImageDraw
from operator import itemgetter
import numpy as np
from geopy.distance import geodesic


def get_lat_long_dist(lat1, lon1, lat2, lon2):
    deg2rad = lambda deg: deg * math.pi / 180
    R = 6371
    dLat = deg2rad(lat2 - lat1)
    dLon = deg2rad(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * \
        math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c
    return d

def convert_coordinates(polyline, initial_pos):
    '''
    Given the polylines given in the output of our algorithm and an initial geocentric location
    on the map. Return a new set of polylines in geocentric location.
    Apply scaling if necessary.
    We use the fact that 111111 meters in the y direction is equivalent to 1 degree of latitude and
    111111 * cos(latitude) meters in the x direction is 1 degree of longitude.
    :param polylines The set of polylines given by the algorithm.
    :param initial_pos Initial geo poistion in which we start our drawing.
    '''
    if len(polyline) == 0:
        raise ValueError("Received an empty polyline.")

    current_xy = polyline[0]
    current_geo = initial_pos

    print("Input Polyline: {0}".format(polyline))
    print("Start Pos: {0}".format(initial_pos))

    previous_map = {}
    geo_polyline = []
    for point in polyline:
        if point in previous_map:
            geo_polyline.append(previous_map[point])
            current_geo = previous_map[point]
        else:
            x_diff = point[0] - current_xy[0]
            y_diff = -(point[1] - current_xy[1])
            lat_diff = 1 / 111111 * y_diff
            long_diff = 1 / (111111 * math.cos(current_geo[0] * math.pi / 180)) * x_diff
            geo_pos = (current_geo[0] + lat_diff, current_geo[1] + long_diff)
            geo_polyline.append(geo_pos)
            current_geo = geo_pos
            previous_map[point] = geo_pos
        current_xy = point

    print("Output GeoPolyline: {0}".format(geo_polyline))
    return geo_polyline

'''
Converts osm's nodes to a serializable format.
'''
def convert_nodes_to_float(nodes):
    converted_nodes = []
    for node in nodes:
        converted_nodes.append([float(node.lat),float(node.lon)])

    return converted_nodes


def connect_polylines(polylines):
    """
    Connect the list of polylines to a single polyline which will define our run.
    TODO - This is a naive solution, when we get to phrases it will probably change.
    In practice we can't just connect one end of a polyline to another because it does not
    make any sense based on the run but this is a temporary solution.
    :param polylines:
    :return:
    """
    if len(polylines) == 1:
        return polylines[0]

    connected_polyline = []

    for idx, polyline in enumerate(polylines):
        for point in polyline:
            connected_polyline.append(point)
    return connected_polyline

def text_to_image(text):
    """
    Convert the given text to a PIL image.
    :param text:
    :return:
    """
    img = Image.new('RGB', (1450, 1450), color='white')

    fnt = ImageFont.truetype('/Library/Fonts/Arial.ttf', 128)
    d = ImageDraw.Draw(img)
    d.text((10, 10), text, font=fnt, fill=(0, 0, 0))
    return img


def convert_polyline_to_decimal(polyline):
    decimal_polyline = []
    for point in polyline:
        decimal_polyline.append((Decimal(point[0]), Decimal(point[1])))
    return decimal_polyline

def preprocess_segments(polyline):
    """
    Gives the ordered list of points of the polyline, return them as pairs of segments
    which will be processed by our algorithm.
    :param segments:
    :return:
    """
    connected_segments = []

    for idx in range(len(polyline) - 1):
        # if (idx < len(polyline) - 1):
        connected_segments.append((polyline[idx % len(polyline)], polyline[(idx + 1) % len(polyline)]))
    return connected_segments

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

def deg2rad(deg):
    return deg * math.pi / 180

def rad2deg(rad):
    return rad * 180 / math.pi

def compute_latlong_angle(lat1, long1, lat2, long2):
    d_long = long2-long1
    y = math.sin(d_long) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_long)
    brng = math.atan2(y,x)
    brng = rad2deg(brng)
    # brng = (brng + 360) % 360 # count degrees counter-clockwise - remove to make clockwise
    return brng

def mod(a, n):
    return a - math.floor(a/n) * n

def is_straight_path(path, threshold=45):
    """
    Check if the given path is straight. This is done by checking the angles between each pair
    of consecutive nodes.
    We wish that every pair of nodes may differ in their angle by less than 45 degrees.
    :param path:
    :return:
    """
    angle = None
    for i in range(len(path)-1):
        curr_point = path[i]
        next_point = path[i+1]

        curr_angle = compute_latlong_angle(deg2rad(float(curr_point[0])),deg2rad(float(curr_point[1])),
                                           deg2rad(float(next_point[0])),deg2rad(float(next_point[1])))

        if angle is not None:
            diff = abs(angle - curr_angle)
            if diff > threshold:
                return False

        angle = curr_angle

    return True




def path_distance_minimization(point1, point2):
    return math.sqrt((float(point1[0]) - float(point2[0])) ** 2 + (float(point1[1]) - float(point2[1])) ** 2)

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


third_metric_ratio = 50
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


def get_closest_node(current_location, nodes):
    return min([[p, path_distance_minimization(current_location, p)] for p in nodes], key=itemgetter(1))[0]

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

def compute_path_length(path):
    dist = 0
    for idx in range(len(path) - 1):
        dist += geodesic((float(path[idx][0]), float(path[idx][1])), (float(path[idx + 1][0]), float(path[idx + 1][1]))).meters
    return dist
