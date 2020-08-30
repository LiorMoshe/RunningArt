from geopy.distance import geodesic
import geopy
import overpy
import os
import time
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter

total = 0
for i in range(10):
    start_time = time.time()
    #url='https://overpass.kumi.systems/api/interpreter'
    api = overpy.Overpass(url='http://52.87.194.67/api/interpreter')
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
    print("--- %s seconds ---" % (time.time() - start_time))
    total += time.time() - start_time
    # print( [(node.lat, node.lon) for node in result.nodes])
print(total, total/10)

start_time = time.time()
api = overpy.Overpass(url='http://34.239.152.172/api/interpreter')
query = """
     <osm-script>
  <union into="_">
    <query into="_" type="way">
          <bbox-query e="-73.986" n="40.767" s="40.758" w="-73.997"/>
      <has-kv k="highway" modv="" v="residential"/>
    </query>
    <query into="_" type="way">
          <bbox-query e="-73.986" n="40.767" s="40.758" w="-73.997"/>
      <has-kv k="highway" modv="" v="tertiary"/>
    </query>
        <query into="_" type="way">
          <bbox-query e="-73.986" n="40.767" s="40.758" w="-73.997"/>
      <has-kv k="highway" modv="" v="pedestrian"/>
    </query>
  </union>
  <print e="" from="_" geometry="skeleton" ids="yes" limit=""  n="" order="id" s="" w=""/>
  <recurse from="_" into="_" type="down"/>
  <print e="" from="_" geometry="skeleton" ids="yes" limit=""  n="" order="quadtile" s="" w=""/>
</osm-script>
            """
print(1)
result = api.query(query)
print(result.ways, result.nodes)
print("--- %s seconds ---" % (time.time() - start_time))
