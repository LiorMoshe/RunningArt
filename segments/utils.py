import math
from decimal import Decimal


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

def convert_coordinates(polylines, initial_pos, scale=1):
    '''
    TODO - Move this elsewhere.
    Given the polylines given in the output of our algorithm and an initial geocentric location
    on the map. Return a new set of polylines in geocentric location.
    Apply scaling if necessary.
    We use the fact that 111111 meters in the y direction is equivalent to 1 degree of latitude and
    111111 * cos(latitude) meters in the x direction is 1 degree of longitude.
    :param polylines The set of polylines given by the algorithm.
    :param initial_pos Initial geo poistion in which we start our drawing.
    :param scale The scale of the edges. TODO- Will be used in the future.
    '''
    if len(polylines) == 0 or len(polylines[0]) == 0:
        raise ValueError("Received empty list of polylines")

    current_xy = polylines[0][0]
    current_geo = initial_pos
    geo_polylines = []

    previous_map = {}
    for polyline in polylines:
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
        geo_polylines.append(geo_polyline)
    return geo_polylines

'''
Converts osm's nodes to a serializable format.
'''
def convert_nodes_to_float(nodes):
    converted_nodes = []
    for node in nodes:
        converted_nodes.append([float(node[0]),float(node[1])])

    return converted_nodes


def segments_as_decimal(polylines):
    updated_polylines = []

    # adjust input
    polylines_fixes = []
    for polyline in polylines:
        for pol in polyline:
            polylines_fixes.append(pol)

    for polyline in polylines_fixes:
        updated_polylines.append((Decimal(polyline[0]), Decimal(polyline[1])))
    return updated_polylines

def preprocess_segments(segments):
    connected_segments = []

    for idx in range(len(segments)):
        connected_segments.append((segments[idx % len(segments)], segments[(idx + 1) % len(segments)]))
    return connected_segments