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

def distance(p1, p2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    lon1, lat1 = p1
    lon2, lat2 = p2
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])
    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    km = 6367 * c
    return km

points = [[32.058119, 34.7708631], [32.0547353, 34.7708529], [32.0563077, 34.7708089], [32.0598833, 34.7680363], [32.0589019, 34.7712548], [32.0606402, 34.7709973], [32.0580247, 34.7699899], [32.0587653, 34.7698838], [32.0588484, 34.7707568], [32.05815, 34.771148], [32.0581427, 34.7720355], [32.057026, 34.771733], [32.0605436, 34.7704922], [32.0596552, 34.772445], [32.0606384, 34.7726995], [32.0575104, 34.7700725], [32.060333, 34.7696562], [32.0571443, 34.7683958], [32.0574695, 34.7683517], [32.0575986, 34.7695311], [32.0562065, 34.771665], [32.0596783, 34.767508], [32.0594758, 34.768251], [32.0588007, 34.7702842], [32.059552, 34.770187], [32.060281, 34.770078], [32.0580717, 34.7704126], [32.0596132, 34.7697429], [32.0597538, 34.7711161], [32.0579306, 34.7691531], [32.0586717, 34.7690581], [32.0601055, 34.7688604], [32.0589461, 34.7718507], [32.0598089, 34.7715224], [32.0589034, 34.7722299], [32.059745, 34.7720522], [32.058836, 34.773028], [32.0580982, 34.7728226], [32.0594991, 34.7732124], [32.0572576, 34.7708452], [32.0599964, 34.7684012], [32.0597076, 34.7706225], [32.0595258, 34.7689162], [32.059482, 34.7684796], [32.0547446, 34.7702197], [32.0571238, 34.7672825], [32.0586001, 34.7681549], [32.0587844, 34.76786], [32.0568861, 34.7729035], [32.0569816, 34.7725449], [32.0570552, 34.772156], [32.0561221, 34.7721185], [32.057207, 34.7713435], [32.0562758, 34.7712495], [32.0581673, 34.7715051], [32.0572657, 34.7704185], [32.0578347, 34.7682988], [32.0578838, 34.7687291], [32.0562842, 34.7701577], [32.0587956, 34.7734078], [32.0562653, 34.7744105], [32.0594004, 34.7735596], [32.0580503, 34.7732269], [32.0595662, 34.7693194], [32.0587125, 34.7694584], [32.0579827, 34.7695673], [32.0576882, 34.7670528], [32.0602062, 34.7691959], [32.0582995, 34.7682298], [32.0567748, 34.7695913], [32.0595823, 34.7728247], [32.0581161, 34.7724135], [32.0588704, 34.7726404], [32.0571224, 34.7678622], [32.0572459, 34.7695435], [32.0603349, 34.7714482], [32.0602878, 34.7722081], [32.0606156, 34.7730959], [32.0605886, 34.7735006], [32.0606575, 34.772311], [32.0606842, 34.7713883], [32.0586272, 34.7686122], [32.0562396, 34.7695882], [32.0602437, 34.76791], [32.0564701, 34.7722224], [32.0577976, 34.7679831], [32.0602574, 34.7717693]]

timee=time.time()
nbrs = NearestNeighbors(n_neighbors=1, metric=distance).fit(points)

distances, indices = nbrs.kneighbors(points)
userlocation = [32.0602868, 34.7694884]
userlocation = np.array([[userlocation[0], userlocation[1]]])
distances, indices = nbrs.kneighbors(userlocation)

print(indices)
print(distances[0][0])
# get the 5 nearest stations in a list
print(time.time()-timee)

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
    bbox = '<bbox-query e="34.773" n="32.060" s="32.057" w="34.768"/>'
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
#     box = '34.768,32.057,34.773,32.060'
#     cmd = 'cmd /c "osmconvert64-0.8.8p.exe israel-and-palestine-latest.osm.pbf ' \
#           ' -b={0} --drop-author --drop-version  --complete-ways -o=edry.o5m"'.format(box)
#     print(cmd)
#     os.system(cmd)
#     cmd = 'cmd /c "osmfilter edry.o5m --keep="highway=pedestrian =tertiary =residential  " --drop-relations --drop-author --drop-version  -o=edry2.o5m"'
#     os.system(cmd)
#     cmd = 'cmd /c "osmconvert64-0.8.8p.exe edry2.o5m -o=edry.pbf"'
#     os.system(cmd)
#     ways = []
#     nodes_info = {}
#     for entity in parse_file('edry.pbf'):
#         if isinstance(entity, Way):
#             nodes = []
#             for n in entity.nodes:
#                 nodes.append(nodes_info[n])
#             ways.append(ArtingWay(nodes))
#         elif isinstance(entity, Relation):
#             pass
#         else:
#             nodes_info[entity.id] = ArtingNode(entity.id, entity.lat, entity.lon)
#     return ways,nodes_info.keys()

def local_convert(location_node,km_distance):
    south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node, km_distance)
    box = '{0},{1},{2},{3}'.format(west_long, south_lat, east_long, north_lat)
    print(box)
    box = '34.768,32.057,34.773,32.060'
    cmd = 'cmd /c "osmconvert64-0.8.8p.exe israel-and-palestine-latest.osm.pbf ' \
          ' -b={0} --drop-author --drop-version  --complete-ways -o=edry.o5m"'.format(box)
    print(cmd)
    os.system(cmd)
    cmd = 'cmd /c "osmfilter edry.o5m --keep="highway=pedestrian =tertiary =residential  " --drop-relations --drop-author --drop-version  -o=edry2.o5m"'
    os.system(cmd)
    cmd = 'cmd /c "osmconvert64-0.8.8p.exe edry2.o5m -o=edry.pbf"'
    os.system(cmd)
    ways = []
    nodes_info = {}
    distances_nodes = []
    timee = time.time()
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

#a = local_convert()
# get_interections(a)
# # print(a)
def intersection_nodes(location_node,km_distance):
    south_lat, west_long, north_lat, east_long = get_boundng_box_intersection_nodes(location_node, km_distance)
    box = '{0},{1},{2},{3}'.format(west_long, south_lat, east_long, north_lat)
    print("moshhoon: ", box)
    box = '34.768,32.057,34.773,32.060'
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