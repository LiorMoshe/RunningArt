import base64
import io
import traceback

from PIL import Image
from flask import Flask, jsonify, request, make_response
from flask_cors import CORS, cross_origin
from flask_httpauth import HTTPBasicAuth

from linedraw.draw import sketchImage

#example client request: curl -u running:art -F "file=@valtho.jpeg" -i http://localhost:5000/polylines

app = Flask(__name__, static_url_path = "")

cors = CORS(app, resources={r"/polylines": {"origins": "http://localhost:port"}})


auth = HTTPBasicAuth()

IMAGE_KEY = 'Image'

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
        msg = base64.b64decode(imageStr)
        buf = io.BytesIO(msg)
        img = Image.open(buf)
        lines = sketchImage(img)
        return jsonify(lines)
    except Exception:
        traceback.print_tb()
        return ""

if __name__ == '__main__':
    app.run(debug = True)