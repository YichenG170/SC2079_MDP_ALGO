#!/usr/bin/env python3

from flask import Flask, jsonify, request
import endpoint

import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

# config
app = Flask(__name__)

@app.route('/status', methods=['GET'])
def status():
    return jsonify({"status": "Live"}), 200

# To get the algo path
@app.route('/path', methods=['POST'])
def path():
    json_input = request.json
    print("Input received: ", json_input)

    t = endpoint.map_to_inst(json_input)
    return t
    # >>>> End of function

if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0', port=6000) 


