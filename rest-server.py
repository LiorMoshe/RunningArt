import base64
import io

from flask import Flask, jsonify, request, make_response
from flask_cors import CORS, cross_origin
from flask_httpauth import HTTPBasicAuth

from arting import *
from segments.cv_contours import *
from segments.utils import *

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
    return jsonify({"nodes": convert_nodes_to_float(nodes)})


@app.route('/polylines', methods = ['POST'])
# @auth.login_required
@cross_origin(origin='localhost',headers=['Content- Type','Authorization'])
def send_drawing():
    # try:

        initial_pos = request.json[POSITION_KEY]
        distance = request.json[DISTANCE_KEY]
        print(initial_pos)
        print("Received distance: ", distance)

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
        polyline = scale_route_to_distance(distance, polyline, average_distance=required_average)


        # Perform averaging to remove redundant points.
        while True:
            print("Averaging")
            polyline, changed = polyline_averaging(polyline, average_dist=required_average)
            if not changed:
                break


        geo_polyline = convert_coordinates(polyline, initial_pos)

        # Currently we don't connect any letters at all.
        # geo_lines = connect_letters(initial_pos, geo_lines)

        # Currently we do not use the algorithm.
        decimal_polyline = convert_polyline_to_decimal(geo_polyline)
        connected_segments = preprocess_segments(decimal_polyline)
        print("Connected segments.")
        out = algorithm((Decimal(initial_pos[0]), Decimal(initial_pos[1])), connected_segments)
        return jsonify({"segments": geo_polyline, "result": out})


if __name__ == '__main__':
    # Get the intersection nodes.

    ways, nodes = get_intersection_nodes_with_ways()
    initialize_ways_graph(ways)
    required_average = compute_average_distance()
    app.run()