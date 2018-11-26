from flask import Flask, render_template, request, jsonify
import json

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/update') 
def update():
    coordinates_file = open('/home/odroid/catkin_buggy2/src/solar_buggy/server/settings/telemetry.json')
    coordinates_object = json.loads(coordinates_file.read())

    suggestions_list = []
    _pretify(suggestions_list, coordinates_object)
    
    return render_template('suggestions.html', suggestions=suggestions_list)

@app.route('/coords', methods = ['POST']) 
def coords():

    try:
        coordinates_object = {}
        coordinates_object['longitude'] = float(request.form.get('longitude'))
        coordinates_object['latitude'] = float(request.form.get('latitude'))

        coordinates_string = json.dumps(coordinates_object)

        coordinates_file = open('/home/odroid/catkin_buggy2/src/solar_buggy/server/settings/initial_coordinates.json', 'w')
        coordinates_file.write(coordinates_string)

        return jsonify({'response': 'OK'}), 201

    except:
        return jsonify({'response': 'Internal Server Error'}), 500


def _pretify(ls, d):
    for key, val in d.items():
        string = str(key).replace('_', ' ') + ': '
        if isinstance(val, dict):
            ls.append('')
            ls.append(string)
            _pretify(ls, val)
        else:
            ls.append(string + str(val))

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')