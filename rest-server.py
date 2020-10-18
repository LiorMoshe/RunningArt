import base64
import io
from location.optimal_start import get_optimal_start
from flask import Flask, jsonify, request, make_response
from flask_cors import CORS, cross_origin
from flask_httpauth import HTTPBasicAuth
from osm.nodes_manager import NodesManager

from algorithm.arting import *
from algorithm.osm import *
from segments.cv_contours import *
from segments.utils import *
from algorithm.fit_algorithm import FitAlgorithm
import time

#example client request: curl -u running:art -F "file=@valtho.jpeg" -i http://localhost:5000/polylines

app = Flask(__name__, static_url_path = "")

cors = CORS(app, resources={r"/polylines": {"origins": "http://localhost:port"}})


auth = HTTPBasicAuth()

IMAGE_KEY = 'Image'

TEXT_KEY = 'Text'

POSITION_KEY = 'Position'

DISTANCE_KEY= 'Distance'

dynamic_starting_location = False

@auth.get_password
def get_password(username):
    if username == 'running':
        return 'art'
    return None

@auth.error_handler
def unauthorized():
    return make_response(jsonify( { 'error': 'Unauthorized access' } ), 403)

@app.errorhandler(400)
def not_found(error):
    return make_response(jsonify( { 'error': 'Bad request' } ), 400)

@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify( { 'error': 'Not found' } ), 404)


nodes = None
converted_nodes = None
fit_algorithm = None
nodes_manager = None
required_average = None

@app.route('/nodes', methods = ['GET'])
# @auth.login_required
@cross_origin(origin='localhost',headers=['Content- Type','Authorization'])
def send_nodes():
    return jsonify({"nodes_map": get_nodes_map(nodes_manager)})
    # return jsonify({"nodes": convert_nodes_to_float(nodes)})


def get_segments_based_on_location(starting_pos, polyline, distance, average_road_length):
    polyline = remove_redundancies(polyline)
    polyline = scale_route_to_distance(distance, polyline, average_distance=average_road_length)

    # Perform averaging to remove redundant points.
    polyline = polyline_averaging(polyline, average_dist=average_road_length)
    geo_polyline = convert_coordinates(polyline, starting_pos)
    return geo_polyline

@app.route('/polylines', methods = ['POST'])
# @auth.login_required
@cross_origin(origin='localhost',headers=['Content- Type','Authorization'])
def send_drawing():
    print("Got Request: ",request)
    initial_pos = request.json[POSITION_KEY]
    distance = request.json[DISTANCE_KEY]

    start_time = time.time()
    km_distance = distance/1000
    print("Requesting intersection_nodes_idx")
    intersections_nodes_idx = get_intersection_nodes_idx(initial_pos, km_distance, file=True)
    print("--- %s seconds ---" % (time.time() - start_time))
    print("Requesting with ways, nodes_idx: {0}".format(len(intersections_nodes_idx)))
    ways, nodes = intersection_nodes_with_ways(initial_pos, km_distance)
    print("Ways: {0}, Nodes: {1}".format(len(ways), len(nodes)))
    print("--- %s seconds ---" % (time.time() - start_time))
    nodes_manager = NodesManager(intersections_nodes_idx)
    nodes_manager.initialize_ways_graph(ways)
    required_average = compute_average_distance(nodes_manager)
    fit_algorithm = FitAlgorithm(nodes_manager)
    print("Finished misc")
    # Parse base64 image url.
    if IMAGE_KEY in request.json:
        imageStr = request.json[IMAGE_KEY].split('base64,', 1)[1]
        msg = base64.b64decode(imageStr)
        buf = io.BytesIO(msg)
        img = Image.open(buf)
        lines = findExternalContours(img)
        polyline = connect_polylines(lines)
    else:
        polyline = text_to_polyline(request.json[TEXT_KEY])

    geo_polyline = get_segments_based_on_location(initial_pos, polyline, distance, required_average)

    # Once we have the required shape, compute, the optimal starting position.
    if dynamic_starting_location:
        initial_pos = get_optimal_start(initial_pos, geo_polyline, km_distance, nodes_manager, nearest_one_mode=False)
        geo_polyline = get_segments_based_on_location(initial_pos, polyline, distance, required_average)


    # Convert the given segments.
    decimal_polyline = convert_polyline_to_decimal(geo_polyline)
    connected_segments = preprocess_segments(decimal_polyline)
    print("Connected segments: ",connected_segments)
    fit_algorithm.set_segments(connected_segments)
    out, dijkstra_paths = fit_algorithm.algorithm((Decimal(initial_pos[0]), Decimal(initial_pos[1])))
    print("Finished Algo")
    updated_paths = append_ids_to_paths(dijkstra_paths, nodes_manager)
    return jsonify({"segments": geo_polyline, "result": out, "paths": updated_paths, "nodes_map": get_nodes_map(nodes_manager)})

if __name__ == '__main__':
    app.run()