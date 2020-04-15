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

POSITION_KEY = 'Position'

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


@app.route('/polylines', methods = ['POST'])
# @auth.login_required
@cross_origin(origin='localhost',headers=['Content- Type','Authorization'])
def send_drawing():
    # try:
        # Parse base64 image url.
        imageStr = request.json[IMAGE_KEY].split('base64,',1)[1]
        initial_pos = request.json[POSITION_KEY]
        print(initial_pos)
        msg = base64.b64decode(imageStr)
        buf = io.BytesIO(msg)
        img = Image.open(buf)
        lines = find_thick_contours(img)

        # TODO-Currently we scale the image only after we receive a full polyline of the image.
        # Convert the format from a single polyline to a set of polylines.
        if len(lines) == 1:
            polyline = lines[0]
            updated_poly = scale_route_to_distance(10000, polyline, average_distance=required_average)
            lines = [updated_poly]


        # Perform averaging to remove redundant points.
        while True:
            lines, changed = segments_averaging(lines, average_dist=required_average)
            if not changed:
                break


        geo_lines = convert_coordinates(lines, initial_pos)

        # Currently we don't connect any letters at all.
        # geo_lines = connect_letters(initial_pos, geo_lines)

        # Currently we do not use the algorithm.
        decimal_lines = segments_as_decimal(geo_lines)
        connected_segments = preprocess_segments(decimal_lines)
        # out = algorithm((Decimal(initial_pos[0]), Decimal(initial_pos[1])), connected_segments, decimal_nodes)
        out = []
        return jsonify({"segments": geo_lines, "result": out, "nodes": converted_nodes})


if __name__ == '__main__':
    # Get the intersection nodes.
    decimal_nodes, nodes, tree = get_intersection_nodes_from_file()
    converted_nodes = convert_nodes_to_float(decimal_nodes)
    required_average = average_euclidean_distance(tree, nodes)
    # print("average meters distance is: ", average_euclidean_distance(tree, nodes))
    app.run(debug = True)