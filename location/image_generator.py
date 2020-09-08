import navpy
import overpy
from geopy.distance import geodesic
import geopy
import math
from PIL import Image
import time

# south_lat, west_long, north_lat, east_long
coordinates = 32.05909518216471,34.76966961513913,32.059996994204475,34.77072857795746
location_node = (32.05954608820065, 34.770199096548296)
distance = 0.08
distance_threshold = 100
line_distance_threshold = 1
target_nodes_coordinates = []
coordinates_to_pixels = {}
lines = []
binary_data = []


class Line(object):
    '''
    This class holds all the information related to the lines which represents the roads in the osm bounding box. They used by the binary image generator.
    '''

    def __init__(self, params, points, p1, p2):
        # A, B, C --> Ax + By + c = 0
        self.parameters = params
        self.points = points
        self.p1 = p1
        self.p2 = p2



def calculate_distance_between_2points(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def is_between(a,c,b):
    return abs(calculate_distance_between_2points(a,c) + calculate_distance_between_2points(c,b) - calculate_distance_between_2points(a,b)) < 0.01


def line_from_points(p1, p2):
    a = p2[1] - p1[1]
    b = p2[0] - p1[0]
    m = a / b
    c = -p1[0]*m + p1[1]
    return (float(m), float(-1), float(c))


def distance_from_point_to_line(point, line):
    return abs(line[0]*point[0] + line[1]*point[1] + line[2]) / math.sqrt(line[0]**2 + line[1]**2)


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
            if geodesic((node.lat, node.lon), location_node).meters < distance_threshold and (node.lat, node.lon) not in target_nodes_coordinates:
                target_nodes_coordinates.append((node.lat, node.lon))
            p1 = navpy.lla2ecef(float(node.lat), float(node.lon), 0)
            if idx < len(way_nodes) - 1:
                next_node = way_nodes[idx + 1]
                p2 = navpy.lla2ecef(float(next_node.lat), float(next_node.lon), 0)
                lines.append(Line(line_from_points(p1, p2), [(float(node.lat), float(node.lon)), (float(next_node.lat), float(next_node.lon))], p1, p2))

preprocessing_ways(location_node)


def generate_binary_image():
    #starting from the south west cornerof the bounding box
    current_coordinates = (coordinates[0], coordinates[1])
    destination = geopy.distance.distance(kilometers=0.0005).destination((current_coordinates[0], current_coordinates[1]), 90)
    longitude = destination.longitude
    destination = geopy.distance.distance(kilometers=0.0005).destination((current_coordinates[0], current_coordinates[1]), 0)
    current_coordinates = (destination.latitude, longitude)
    #calculating starting coordinates array for each line in the image we will create
    starting_coordinates_array = [current_coordinates]
    tmp_coordinates = current_coordinates
    num_of_iterations = int(distance*2000) - 1
    for i in range(num_of_iterations):
        destination = geopy.distance.distance(kilometers=0.001).destination((tmp_coordinates[0], tmp_coordinates[1]), 0)
        tmp_coordinates = (destination.latitude, destination.longitude)
        starting_coordinates_array.append(tmp_coordinates)
    #fill the binary image matrix
    for i in range(0, len(starting_coordinates_array)):
        time1 = time.time()
        current_coordinates = (starting_coordinates_array[i][0], starting_coordinates_array[i][1])
        for j in range(int(distance*2*1000)):
            point = navpy.lla2ecef(float(current_coordinates[0]), float(current_coordinates[1]), 0)
            near_road = False
            for line in lines:
                dis = distance_from_point_to_line(point, line.parameters)
                if dis < line_distance_threshold and is_between(line.p1, point, line.p2):
                    near_road = True
                    # for target_node in target_nodes_coordinates:
                    #     if geodesic((current_coordinates[0], current_coordinates[1]), target_node).meters < 0.5:
                    #         coordinates_to_pixels[(current_coordinates[0], current_coordinates[1])] = (point[0], point[1])
                    #         break
                    break
            binary_data.append(0) if near_road else binary_data.append(1)
            destination = geopy.distance.distance(kilometers=0.001).destination((current_coordinates[0], current_coordinates[1]), 90)
            current_coordinates = (destination.latitude, destination.longitude)
        time2 = time.time()
        print('function took {:.3f} ms'.format((time2 - time1) * 1000.0))


generate_binary_image()
binary_data[:] = [binary_data[i:i + 160] for i in range(0, 160*160, 160)]
img = Image.new('1', (160, 160))
pixels = img.load()
for i in range(img.size[0]):
    for j in range(img.size[1]):
        pixels[i, j] = binary_data[i][j]
img.show()
print(coordinates_to_pixels)



