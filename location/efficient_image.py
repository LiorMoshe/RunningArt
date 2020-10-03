import navpy
import overpy
from geopy.distance import geodesic
import geopy
import math
from PIL import Image
import time
from numpy import random
import numpy as np
import cv2

coordinates = 32.05882463852773, 34.76935192629364, 32.060267537791354, 34.771046266802955
location_node = (32.05954608820065, 34.770199096548296)
distance = 0.08
south_west_ecef = navpy.lla2ecef(float(coordinates[0]), float(coordinates[1]), 0)
north_east_ecef = navpy.lla2ecef(float(coordinates[2]), float(coordinates[3]), 0)
bounding_box_segments = []
bounding_box_segments.append([(south_west_ecef[0], south_west_ecef[1]), (north_east_ecef[0], south_west_ecef[1])])
bounding_box_segments.append([(south_west_ecef[0], north_east_ecef[1]), (north_east_ecef[0], north_east_ecef[1])])
bounding_box_segments.append([(south_west_ecef[0], south_west_ecef[1]), (south_west_ecef[0], north_east_ecef[1])])
bounding_box_segments.append([(north_east_ecef[0], south_west_ecef[1]), (north_east_ecef[0], north_east_ecef[1])])

def calculate_distance_between_2points(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def is_between(a,c,b):
    return abs(calculate_distance_between_2points(a,c) + calculate_distance_between_2points(b,c) - calculate_distance_between_2points(a,b)) < 0.01

def intersect(a1, a2, b1, b2):
    """
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return False
    print(x/z,y/z)
    return is_between(b1,(x/z,y/z),b2)

# """
# Check whether two line segments intersect.
# """
# def ccw(A, B, C):
#     return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
#
# """
# Check whether two line segments intersect.
# """
# def intersect(A, B, C, D):
#     return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def inside_bounding_box(node, next_node):
    ecef_node = navpy.lla2ecef(float(node[0]), float(node[1]), 0)
    ecef_node = (ecef_node[0], ecef_node[1])
    ecef_next_node = navpy.lla2ecef(float(next_node[0]), float(next_node[1]), 0)
    ecef_next_node = (ecef_next_node[0], ecef_next_node[1])
    return not intersect(ecef_node, ecef_next_node, bounding_box_segments[0][0], bounding_box_segments[0][1]) and not \
        intersect(ecef_node, ecef_next_node, bounding_box_segments[1][0], bounding_box_segments[1][1]) and not \
        intersect(ecef_node, ecef_next_node, bounding_box_segments[2][0], bounding_box_segments[2][1]) and not \
        intersect(ecef_node, ecef_next_node, bounding_box_segments[3][0], bounding_box_segments[3][1])

def intersection_nodes_with_ways(location_node, km_distance):
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
    result = api.query(query)
    return result.ways, result.nodes


def preprocessing_ways(location_node):
    ways, nodes = intersection_nodes_with_ways(location_node, distance)
    for way in ways:
        way_nodes = way.nodes
        for idx, node in enumerate(way_nodes):
            if idx < len(way_nodes) - 1:
                next_node = way_nodes[idx + 1]
                if inside_bounding_box(node, next_node):
                    pass
                else:
                    tmp_idx = idx + 1
                    found = False
                    while tmp_idx < len(way_nodes) - 1 and not found:
                        next_node = way_nodes[tmp_idx + 1]
                        if inside_bounding_box(node, next_node):
                            found = True
                        tmp_idx = tmp_idx + 1


#preprocessing_ways(location_node)
a = 32.05859, 34.76974
b = 32.05921, 34.77003

print(inside_bounding_box(a,b))