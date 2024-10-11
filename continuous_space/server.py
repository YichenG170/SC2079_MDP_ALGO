import flask
import endpoint

app = flask.Flask(__name__)

@app.route("/", methods=["GET"])
def index():
    return "Hello, World!"

@app.route("/obstacles", methods=["POST"])
def obstacles():
    json_input = flask.request.json
    print(json_input)

    path = endpoint.map_to_inst(json_input)

    return path

if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5001)
