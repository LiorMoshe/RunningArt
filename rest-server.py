import base64
import io
import traceback
import math

from PIL import Image
from flask import Flask, jsonify, request, make_response
from flask_cors import CORS, cross_origin
from flask_httpauth import HTTPBasicAuth

from linedraw.draw import sketchImage

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

    flattened_polylines = []
    for polyline in polylines:
        for point in polyline:
            flattened_polylines.append(point)

    geo_polylines = [initial_pos]
    current_geo = initial_pos
    current_xy = flattened_polylines[0]
    for idx, point in enumerate(flattened_polylines):
        if idx > 0:
            x_diff = point[0] - current_xy[0]
            y_diff = -(point[1] - current_xy[1])

            lat_diff = 1 / 111111 * y_diff
            long_diff = 1 / (111111 * math.cos(current_geo[0] * math.pi / 180)) * x_diff

            geo_pos = (current_geo[0] + lat_diff, current_geo[1] + long_diff)
            geo_polylines.append(geo_pos)
            current_geo = geo_pos
            current_xy = point

    return geo_polylines


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

@app.route('/polylines', methods = ['POST'])
# @auth.login_required
@cross_origin(origin='localhost',headers=['Content- Type','Authorization'])
def send_drawing():
    try:
        # Parse base64 image url.
        imageStr = request.json[IMAGE_KEY].split('base64,',1)[1]
        initial_pos = request.json[POSITION_KEY]
        print(initial_pos)
        msg = base64.b64decode(imageStr)
        buf = io.BytesIO(msg)
        img = Image.open(buf)
        lines = sketchImage(img)
        geo_lines = convert_coordinates(lines, initial_pos)
        return jsonify(geo_lines)
    except Exception:
        traceback.print_tb()
        return ""

if __name__ == '__main__':
    app.run(debug = True)