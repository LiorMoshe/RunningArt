import base64
import cv2
import io
import traceback
import math

from PIL import Image
from flask import Flask, jsonify, request, make_response
from flask_cors import CORS, cross_origin
from flask_httpauth import HTTPBasicAuth
import numpy as np
from arting import *
from decimal import Decimal
from segments.cv_contours import *


'''
Converts osm's nodes to a serializable format.
'''
def convert_nodes_to_float(nodes):
    converted_nodes = []
    for node in nodes:
        converted_nodes.append([float(node[0]),float(node[1])])

    return converted_nodes


def convert_coordinates(polylines, initial_pos, scale=1):
    '''
    TODO - Move this elsewhere.
    Given the polylines given in the output of our algorithm and an initial geocentric location
    on the map. Return a new set of polylines in geocentric location.
    Apply scaling if necessary.
    We use the fact that 111111 meters in the y direction is equivalent to 1 degree of latitude and
    111111 * cos(latitude) meters in the x direction is 1 degree of longitude.
    :param polylines The set of polylines given by the algorithm.
    :param initial_pos Initial geo poistion in which we start our drawing.
    :param scale The scale of the edges. TODO- Will be used in the future.
    '''
    if len(polylines) == 0 or len(polylines[0]) == 0:
        raise ValueError("Received empty list of polylines")

    current_xy = polylines[0][0]
    current_geo = initial_pos
    geo_polylines = []

    for polyline in polylines:
        geo_polyline = []
        for point in polyline:
            x_diff = point[0] - current_xy[0]
            y_diff = -(point[1] - current_xy[1])

            lat_diff = 1 / 111111 * y_diff
            long_diff = 1 / (111111 * math.cos(current_geo[0] * math.pi / 180)) * x_diff

            geo_pos = (current_geo[0] + lat_diff, current_geo[1] + long_diff)
            geo_polyline.append(geo_pos)
            current_geo = geo_pos
            current_xy = point
        geo_polylines.append(geo_polyline)
    return geo_polylines

def segments_as_decimal(polylines):
    updated_polylines = []

    # adjust input
    polylines_fixes = []
    for polyline in polylines:
        for pol in polyline:
            polylines_fixes.append(pol)

    for polyline in polylines_fixes:
        updated_polylines.append((Decimal(polyline[0]), Decimal(polyline[1])))
    return updated_polylines

def preprocess_segments(segments):
    connected_segments = []

    for idx in range(len(segments)):
        connected_segments.append((segments[idx % len(segments)], segments[(idx + 1) % len(segments)]))
    return connected_segments


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

        # Perform averaging to remove redundant points.
        while True:
            lines, changed = segments_averaging(lines)
            if not changed:
                break

        geo_lines = convert_coordinates(lines, initial_pos)
        geo_lines = connect_letters(initial_pos, geo_lines)
        print("Number of segments: ", len(geo_lines))
        out = []

        # Currently we do not use the algorithm.
        decimal_lines = segments_as_decimal(geo_lines)
        connected_segments = preprocess_segments(decimal_lines)
        out = algorithm((Decimal(initial_pos[0]), Decimal(initial_pos[1])), connected_segments, nodes)
        return jsonify({"segments": geo_lines, "result": out, "nodes": converted_nodes})


if __name__ == '__main__':
    # Get the intersection nodes.
    decimal_nodes, nodes, tree = get_intersection_nodes_from_file()
    converted_nodes = convert_nodes_to_float(decimal_nodes)
    print("average meters distance is: ", average_euclidean_distance(tree, nodes))
    app.run(debug = True)