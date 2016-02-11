from flask import Flask, render_template, request, jsonify
import json

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/update') 
def update():
    coordinates_file = open('/home/odroid/catkin_buggy2/src/solar_buggy/settings/telemetry.json')
    coordinates_object = json.loads(coordinates_file.read())

    suggestions_list = []
    for statistic in coordinates_object:
        suggestions_list.append(statistic + ': ' + str(coordinates_object[statistic]))
    
    return render_template('suggestions.html', suggestions=suggestions_list)

@app.route('/coords', methods = ['POST']) 
def coords():
    coordinates_file = open('/home/odroid/catkin_buggy2/src/solar_buggy/settings/initial_coordinates.json', 'w')
    coordinates_object = {}

    coordinates_object['longitude'] = float(request.form.get('longitude'))
    coordinates_object['latitude'] = float(request.form.get('latitude'))

    coordinates_string = json.dumps(coordinates_object)

    coordinates_file.write(coordinates_string)

    return jsonify({'response': 'OK'}), 201

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')