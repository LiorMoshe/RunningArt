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
import platform

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


# def local_convert(location_node,km_distance):
#     south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node, km_distance)
#     box = '{0},{1},{2},{3}'.format(west_long, south_lat, east_long, north_lat)
#     print(box)
#     print(os.getcwd())
#     cmd = "osmconvert israel-and-palestine-latest.osm.pbf -b={0} --drop-author --drop-version  --complete-ways -o=edry.o5m".format(box)
#     print(cmd)
#     os.system(cmd)
#     cmd = "osmfilter edry.o5m --keep=\"highway=pedestrian =tertiary =residential  \" --drop-relations --drop-author --drop-version  -o=edry2.o5m"
#     os.system(cmd)
#     cmd = "osmconvert edry2.o5m -o=edry.pbf"
#     os.system(cmd)
#     ways = []
#     nodes_info = {}
#     distances_nodes = []
#     timee = time.time()
#     for entity in parse_file('edry.pbf'):
#         if isinstance(entity, Way):
#             nodes = []
#             for n in entity.nodes:
#                 if n in nodes_info.keys():
#                     nodes.append(nodes_info[n])
#             ways.append(ArtingWay(nodes))
#         elif isinstance(entity, Relation):
#             pass
#         else:
#             if len(nodes_info.keys())<2:
#                 nodes_info[entity.id] = ArtingNode(entity.id, entity.lat, entity.lon)
#                 distances_nodes.append([entity.lat, entity.lon])
#             else:
#                 points = distances_nodes
#                 nbrs = NearestNeighbors(n_neighbors=1, metric=distance).fit(points)
#                 userlocation = [entity.lat, entity.lon]
#
#                 userlocation = np.array([[userlocation[0], userlocation[1]]])
#                 distances, indices = nbrs.kneighbors(userlocation)
#                 if(distances[0][0]>0.02494899):
#                     nodes_info[entity.id] = ArtingNode(entity.id, entity.lat, entity.lon)
#                     distances_nodes.append([entity.lat, entity.lon])
#
#     print('-----------------')
#     print(time.time() - timee)
#     return ways,nodes_info.keys()



def local_convert(location_node, km_distance):
    south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node, km_distance)
    box = '{0},{1},{2},{3}'.format(west_long, south_lat, east_long, north_lat)
    plt = platform.system()
    if plt == "Darwin":
        if get_location_country(location_node)=='ישראל':
            cmd = "osmconvert {0} -b={1} --drop-author --drop-version  --complete-ways -o=edry.o5m".format('israel-and-palestine-latest.osm.pbf', box)
            if get_location_country(location_node) == 'Hungary':
                cmd = "osmconvert {0} -b={1} --drop-author --drop-version  --complete-ways -o=edry.o5m".format(
                    'hungary-latest.osm.pbf', box)
        timee = time.time()
        os.system(cmd)
        print("first done: ", time.time() - timee)
        timee = time.time()
        cmd = "osmfilter edry.o5m --keep=\"highway=pedestrian =tertiary =residential  \" --drop-relations --drop-author --drop-version  -o=edry2.o5m"
        os.system(cmd)
        print("second done: ", time.time() - timee)
        timee = time.time()
        cmd = "osmconvert edry2.o5m -o=edry.pbf"
        os.system(cmd)
        print("third done: ", time.time() - timee)
    elif plt == "Windows":
        country = get_location_country(location_node)
        if country =='ישראל' :
            cmd = 'cmd /c "osmconvert64-0.8.8p.exe {0} ' \
              ' -b={1} --drop-author --drop-version  --complete-ways -o=edry.o5m"'.format('israel-and-palestine-latest.osm.pbf', box)
        if country == 'Magyarország':
            cmd = 'cmd /c "osmconvert64-0.8.8p.exe {0} ' \
              ' -b={1} --drop-author --drop-version  --complete-ways -o=edry.o5m"'.format('hungary-latest.osm.pbf', box)
        timee = time.time()
        os.system(cmd)
        print("first done: ", time.time() - timee)
        cmd = 'cmd /c "osmfilter edry.o5m --keep="highway=pedestrian =tertiary =residential  " --drop-relations --drop-author --drop-version  -o=edry2.o5m"'
        timee = time.time()
        os.system(cmd)
        print("second done: ", time.time() - timee)
        timee = time.time()
        cmd = 'cmd /c "osmconvert64-0.8.8p.exe edry2.o5m -o=edry.pbf"'
        os.system(cmd)
        print("third done: ", time.time() - timee)

    ways = []
    nodes_info = {}
    distances_nodes = []
    for entity in parse_file('edry.pbf'):
        if isinstance(entity, Way):
            nodes = []
            for n in entity.nodes:
                if n in nodes_info.keys():
                    nodes.append(nodes_info[n])
            ways.append(ArtingWay(nodes))
        elif isinstance(entity, Relation):
            pass
        else:
            nodes_info[entity.id] = ArtingNode(entity.id, entity.lat, entity.lon)
            distances_nodes.append([entity.lat, entity.lon, entity.id])

    nodes_info = {}
    deleted = {}
    timee = time.time()
    points = []
    for node in distances_nodes:
        points.append([node[0], node[1]])
    nbrs = NearestNeighbors(n_neighbors=5, metric=distance).fit(points)
    distances, indices = nbrs.kneighbors(points)
    for i in range(len(distances_nodes)):
        # if distances_nodes[i][2] == 366650750 or distances_nodes[i][2] == 496176177:
        #     print(distances_nodes[i][2], distances_nodes[indices[i][1]][2])
        #     print("dis: ", distance([distances_nodes[i][0], distances_nodes[i][1]],
        #                             [distances_nodes[indices[i][1]][0], distances_nodes[indices[i][1]][1]]))
        if distances_nodes[indices[i][1]][2] not in deleted.keys() and distances_nodes[i][2] not in deleted.keys():
            if distances[i][1] > 0.02494899:
                nodes_info[distances_nodes[i][2]] = ArtingNode(distances_nodes[i][2], distances_nodes[i][0],
                                                               distances_nodes[i][1])
            else:
                for j in range(5):
                    if j != 0:
                        if distances[i][j] > 0.02494899:
                            break;
                        else:
                            deleted[distances_nodes[indices[i][j]][2]] = "deleted"
                nodes_info[distances_nodes[i][2]] = ArtingNode(distances_nodes[i][2], distances_nodes[i][0],
                                                               distances_nodes[i][1])


    # for point in distances_nodes:
    #     userlocation = np.array([[point[0], point[1]]])
    #     distances, indices = nbrs.kneighbors(userlocation)
    #     #0.02494899
    #     if (distances[0][1] > 0.005):
    #         nodes_info[point[2]] = ArtingNode(point[2], point[0], point[1])

    print('-----------------')
    print(time.time() - timee)
    return ways,nodes_info.keys()



def get_location_country(position):
    locator = Nominatim(user_agent="my-application")
    coordinates = (position[0], position[1])
    location = locator.reverse(coordinates)
    print(location.raw['address']['country'])
    return location.raw['address']['country']


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


