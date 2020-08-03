import base64
import io

from flask import Flask, jsonify, request, make_response
from flask_cors import CORS, cross_origin
from flask_httpauth import HTTPBasicAuth

from algorithm.arting import *
from segments.cv_contours import *
from segments.utils import *
from osm.bounding_box_calculation import *

#example client request: curl -u running:art -F "file=@valtho.jpeg" -i http://localhost:5000/polylines

app = Flask(__name__, static_url_path = "")

cors = CORS(app, resources={r"/polylines": {"origins": "http://localhost:port"}})


auth = HTTPBasicAuth()

IMAGE_KEY = 'Image'

TEXT_KEY = 'Text'

POSITION_KEY = 'Position'

DISTANCE_KEY= 'Distance'

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
required_average = None

@app.route('/nodes', methods = ['GET'])
# @auth.login_required
@cross_origin(origin='localhost',headers=['Content- Type','Authorization'])
def send_nodes():
    return jsonify({"nodes_map": get_nodes_map()})
    # return jsonify({"nodes": convert_nodes_to_float(nodes)})


@app.route('/polylines', methods = ['POST'])
# @auth.login_required
@cross_origin(origin='localhost',headers=['Content- Type','Authorization'])
def send_drawing():
        reset_maps()
        initial_pos = request.json[POSITION_KEY]
        distance = request.json[DISTANCE_KEY]
        ways,intersections_nodes_idx = local_convert(initial_pos, distance/1000)
        intersections_nodes_idx = get_intersection_nodes_idx(initial_pos, distance/1000)
        intersections_nodes_idx = initialize_ways_graph(ways, intersections_nodes_idx)
        required_average = compute_average_distance(intersections_nodes_idx)

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

        # TODO-Currently we scale the image only after we receive a full polyline of the image.
        # Convert the format from a single polyline to a set of polylines.
        polyline = remove_redundancies(polyline)
        polyline = scale_route_to_distance(distance, polyline, average_distance=required_average)


        # Perform averaging to remove redundant points.
        polyline = polyline_averaging(polyline, average_dist=required_average)
        geo_polyline = convert_coordinates(polyline, initial_pos)
        # Currently we don't connect any letters at all.
        # geo_lines = connect_letters(initial_pos, geo_lines)

        # Currently we do not use the algorithm.
        decimal_polyline = convert_polyline_to_decimal(geo_polyline)

        connected_segments = preprocess_segments(decimal_polyline)
        print("Connected segments.")
        print(connected_segments,len(connected_segments))
        out, dijkstra_paths = algorithm((Decimal(initial_pos[0]), Decimal(initial_pos[1])), connected_segments, intersections_nodes_idx, threshold=required_average)
        updated_paths = append_ids_to_paths(dijkstra_paths)
        print("Paths: ", dijkstra_paths)
        return jsonify({"segments": geo_polyline, "result": out, "paths": updated_paths, "nodes_map":get_nodes_map()})


if __name__ == '__main__':
    app.run()