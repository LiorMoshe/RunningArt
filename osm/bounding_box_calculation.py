from geopy.distance import geodesic
import geopy
import overpy
import os
import time
from osmread import parse_file, Way, Relation
from sklearn.neighbors import NearestNeighbors
import numpy as np
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter

dist_cache = {}
def distance(p1, p2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    lon1, lat1 = p1
    lon2, lat2 = p2
    try:
        return dist_cache[(lon1, lat1, lon2, lat2)]
    except KeyError:
        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])
        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        km = 6367 * c
        dist_cache[(lon1, lat1, lon2, lat2)] = km
        return km


class ArtingNode:
  def __init__(self, id, lat, lon):
    self.id = id
    self.lat = lat
    self.lon = lon

class ArtingWay:
  def __init__(self, nodes):
    self.nodes = nodes



def get_boundng_box_intersection_nodes(location_node, km_distance):
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
    south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node,km_distance)
    api = overpy.Overpass()
    bbox = '<bbox-query e=%s n=%s s=%s w=%s/>' % ('\"'+str(east_long)+'\"','\"'+str(north_lat)+'\"','\"'+str(south_lat)+'\"','\"'+str(west_long)+'\"')
    query = '''
    <query type="way" into="hw">
      <has-kv k="highway"/>
      <has-kv k="highway" modv="not" regv="footway|cycleway|path|service|track"/>
              %s
    </query>


<foreach from="hw" into="w">
  <recurse from="w"  type="way-node" into="n3"/>
  <query type="node">
    <item set="n3"/>
  </query>
  <print/>
</foreach>
    <print/>
    ''' % bbox
    query = '''
    <osm-script>
<query type="way" into="hw">
  <has-kv k="highway"/>
  <has-kv k="highway" modv="not" regv="footway|cycleway|path|service|track"/>
   <bbox-query e="34.773" n="32.060" s="32.057" w="34.768"/> 
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
    '''
    result = api.query(query)
    return [node.id for node in result.nodes]


def intersection_nodes_with_ways(location_node, km_distance):
    south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node,km_distance)
    api = overpy.Overpass()
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


def local_convert(location_node,km_distance):
    south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node, km_distance)
    box = '{0},{1},{2},{3}'.format(west_long, south_lat, east_long, north_lat)
    print(box)
    print(os.getcwd())
    cmd = "osmconvert israel-and-palestine-latest.osm.pbf -b={0} --drop-author --drop-version  --complete-ways -o=edry.o5m".format(box)
    print(cmd)
    os.system(cmd)
    cmd = "osmfilter edry.o5m --keep=\"highway=pedestrian =tertiary =residential  \" --drop-relations --drop-author --drop-version  -o=edry2.o5m"
    os.system(cmd)
    cmd = "osmconvert edry2.o5m -o=edry.pbf"
    os.system(cmd)
    ways = []
    nodes_info = {}
    distances_nodes = []
    timee = time.time()
    for entity in parse_file('edry.pbf'):
        print("Parsing entity")
        if isinstance(entity, Way):
            nodes = []
            for n in entity.nodes:
                if n in nodes_info.keys():
                    nodes.append(nodes_info[n])
            ways.append(ArtingWay(nodes))
        elif isinstance(entity, Relation):
            pass
        else:
            if len(nodes_info.keys())<2:
                nodes_info[entity.id] = ArtingNode(entity.id, entity.lat, entity.lon)
                distances_nodes.append([entity.lat, entity.lon])
            else:
                points = distances_nodes
                nbrs = NearestNeighbors(n_neighbors=1, metric=distance).fit(points)
                userlocation = [entity.lat, entity.lon]

                userlocation = np.array([[userlocation[0], userlocation[1]]])
                distances, indices = nbrs.kneighbors(userlocation)
                if(distances[0][0]>0.02494899):
                    nodes_info[entity.id] = ArtingNode(entity.id, entity.lat, entity.lon)
                    distances_nodes.append([entity.lat, entity.lon])

    print('-----------------')
    print(time.time() - timee)
    return ways,nodes_info.keys()


def get_location_country(position):
    locator = Nominatim(user_agent="my-application")
    coordinates = (53.480837, -2.244914)
    location = locator.reverse(coordinates)
    print(location.raw['address']['country'])


def get_interections(ways):
    nodes = []
    for way in ways:
        for node in way.nodes:
            if node.id not in nodes:
                nodes.append(node.id)
    return nodes

def intersection_nodes(location_node,km_distance):
    south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node, km_distance)
    box = '{0},{1},{2},{3}'.format(west_long, south_lat, east_long, north_lat)
    cmd = 'cmd /c "osmconvert64-0.8.8p.exe ../israel-and-palestine-latest.osm.pbf ' \
          ' -b={0} --complete-ways -o=intersections.o5m"'.format(box)
    os.system(cmd)
    cmd = 'cmd /c "osmfilter intersections.o5m --keep="highway=" --drop-ways="highway=footway =cycleway =path =service =track" -o=good_ways.osm"'
    os.system(cmd)
    cmd = 'cmd /c "osmconvert64-0.8.8p.exe good_ways.osm -o=good_ways.pbf"'
    os.system(cmd)
    nodes = []
    for entity in parse_file('good_ways.pbf'):
        if isinstance(entity, Way):
            for node in entity.nodes:
                # if node==2470035536:
                #     print("hide")
                aaa = 1
            pass
        elif isinstance(entity, Relation):
            print("asdasd")
        else:
            nodes.append(entity.id)
    return nodes

# aaa= [286643475, 317214411, 357500272, 357545243, 366653136, 799417137, 286643440, 286643458, 286643460, 366651300, 366652380, 366652746, 366653067, 366653799, 1571764097, 1628430688, 1628430723, 4170418930, 366652962, 540420234, 540420265, 540420291, 366654065, 366654066, 540419840, 2470061832, 406399586, 540419838, 540419855, 1574692678, 2294948482, 540419958, 286643465, 286741983, 549271109, 1574692741, 1574692746, 1574692918, 286542239, 286542525, 286543443, 286754329, 496176171, 1628430716, 1672351167, 4582891013, 496176315, 496176455, 799417353, 366653165, 366653693, 1628430719, 540421284, 540421320, 1628430692, 286643451, 357536696, 366651462, 286643444, 366651463, 357538387, 1672351158, 2108063257, 357538922, 357536485, 366651303, 366651349, 496176172, 540420824, 366652262, 366652516, 496176174, 2139244077, 2470061834, 1628430689, 1628430687, 1628430710, 1628430720, 2470061831, 412522566, 496176177, 2470061851, 2469958099, 286643432, 4833025980, 2139244073, 7052661053, 514357166, 366649858, 384695042, 1995922116, 1995922128, 1995922151, 2470061837, 3999875641]
#
# bbb = [286643475, 317214411, 357500272, 357545243, 366653136, 799417137, 286643440, 286643458,
#                            286643460, 366651300, 366652380, 366652746, 366653067, 366653799, 1571764097, 1628430688,
#                            1628430723, 4170418930, 366652962, 540420234, 540420265, 540420291, 366654065, 366654066,
#                            540419840, 2470061832, 406399586, 540419838, 540419855, 1574692678, 2294948482, 540419958,
#                            286643465, 286741983, 549271109, 1574692741, 1574692746, 1574692918, 286542239, 286542525,
#                            286543443, 286754329, 496176171, 1628430716, 1672351167, 4582891013, 496176315, 496176455,
#                            799417353, 366653165, 366653693, 1628430719, 540421284, 540421320, 1628430692, 286643451,
#                            357536696, 366651462, 286643444, 366651463, 357538387, 1672351158, 2108063257, 357538922,
#                            357536485, 366651303, 366651349, 496176172, 540420824, 366652262, 366652516, 496176174,
#                            2139244077, 2470061834, 1628430689, 1628430687, 1628430710, 1628430720, 2470061831,
#                            412522566, 496176177, 2470061851, 2469958099, 286643432, 4833025980, 2139244073, 7052661053,
#                            514357166, 366649858, 384695042, 1995922116, 1995922128, 1995922151, 2470061837, 3999875641]
#
# for xx in bbb:
#     if xx not in aaa:
#         print(xx)