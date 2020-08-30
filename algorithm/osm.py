from geopy.distance import geodesic
import geopy
import overpy
import os
import time
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter


def bbox_calculation(location_node, km_distance):
    km_distance = km_distance
    destination = geopy.distance.distance(kilometers=km_distance).destination((location_node[0],location_node[1]), 180)
    south_lat, south_long = destination.latitude, destination.longitude
    destination = geopy.distance.distance(kilometers=km_distance).destination((location_node[0],location_node[1]), 0)
    north_lat, north_long = destination.latitude, destination.longitude
    destination = geopy.distance.distance(kilometers=km_distance).destination((location_node[0],location_node[1]), 90)
    east_lat, east_long = destination.latitude, destination.longitude
    destination = geopy.distance.distance(kilometers=km_distance).destination((location_node[0],location_node[1]), 270)
    west_lat, west_long = destination.latitude, destination.longitude
    return south_lat, west_long, north_lat, east_long


def get_intersection_nodes_idx(location_node, km_distance):
    south_lat, west_long, north_lat, east_long = bbox_calculation(location_node, km_distance)
    api = overpy.Overpass(url='https://overpass.kumi.systems/api/interpreter')
    bbox = '<bbox-query e=%s n=%s s=%s w=%s/>' % ('\"'+str(east_long)+'\"','\"'+str(north_lat)+'\"','\"'+str(south_lat)+'\"','\"'+str(west_long)+'\"')
    query = '''
    <osm-script>
<query type="way" into="hw">
  <has-kv k="highway"/>
  <has-kv k="highway" modv="not" regv="footway|cycleway|path|service|track"/>
   %s 
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
    ''' % bbox
    result = api.query(query)
    return [node.id for node in result.nodes]


def intersection_nodes_with_ways(location_node, km_distance):
    south_lat, west_long, north_lat, east_long = bbox_calculation(location_node,km_distance)
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
