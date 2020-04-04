from flask import Flask, jsonify, request, make_response
from flask_httpauth import HTTPBasicAuth
from linedraw.draw import sketch, visualize
import cv2 as cv
import numpy as np
import os

#example client request: curl -u running:art -F "file=@valtho.jpeg" -i http://localhost:5000/polylines

app = Flask(__name__, static_url_path = "")
auth = HTTPBasicAuth()

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
@auth.login_required
def send_drawing():
    data = request.files['file'].read()
    npimg = np.fromstring(data, np.uint8)
    img = cv.imdecode(npimg, cv.IMREAD_COLOR)
    if __name__ == "__main__":
        path ='resources/images/input.jpg'
        cv.imwrite(path, img)
        lines = sketch(path)
        os.remove(path)
    return jsonify(lines)

if __name__ == '__main__':
    app.run(debug = True)