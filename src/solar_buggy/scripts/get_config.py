import json, os

script_dir = os.path.dirname(__file__)
file_path = os.path.join(script_dir, '../config.json')

with open(file_path, 'r') as f:
    config = json.load(f)
