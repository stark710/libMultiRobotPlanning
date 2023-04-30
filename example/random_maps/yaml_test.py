import yaml

goalTimeWindows = [[[0, 10], [0, 20]]]

with open('output.yaml', 'w') as file:
    yaml.safe_dump({'goalTimeWindows': goalTimeWindows}, file)