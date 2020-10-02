import navpy
import overpy
from geopy.distance import geodesic
import geopy
import math
from PIL import Image
import time
from numpy import random
import cv2

coordinates = 32.05882463852773, 34.76935192629364, 32.060267537791354, 34.771046266802955
location_node = (32.05954608820065, 34.770199096548296)
distance = 0.08


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


def inside_bounding_box(node):
    pass


def preprocessing_ways(location_node):
    ways, nodes = intersection_nodes_with_ways(location_node, distance)
    for way in ways:
        way_nodes = way.nodes
        for idx, node in enumerate(way_nodes):
            if inside_bounding_box(node):
                if idx < len(way_nodes) - 1:
                    next_node = way_nodes[idx + 1]
                    if inside_bounding_box(next_node):
                        pass
                    else:
                        tmp_idx = idx + 1
                        found = False
                        while tmp_idx < len(way_nodes) - 1 and not found:
                            next_node = way_nodes[tmp_idx + 1]
                            if inside_bounding_box(next_node):
                                found = True
                            tmp_idx = tmp_idx + 1


preprocessing_ways(location_node)


