import math
from decimal import Decimal

from PIL import ImageFont, Image, ImageDraw


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
    TODO - Move this elsewhere.
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
    return geo_polyline

'''
Converts osm's nodes to a serializable format.
'''
def convert_nodes_to_float(nodes):
    converted_nodes = []
    for node in nodes:
        converted_nodes.append([float(node[0]),float(node[1])])

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
    if len(polylines) == 0:
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

    for idx in range(len(polyline)):
        connected_segments.append((polyline[idx % len(polyline)], polyline[(idx + 1) % len(polyline)]))
    return connected_segments