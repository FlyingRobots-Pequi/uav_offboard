import yaml

CONFIG_PATH = '/home/orin2/flying/hermit_autonomy/ros_packages/hermit_offboard/config/goto_setpoints.yaml'

class MyDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, indentless=False)

def represent_list(dumper, data):
    # Check if it's a top-level list (for setpoints), use block style for top level only
    if all(isinstance(i, list) for i in data):
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=False)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

MyDumper.add_representer(list, represent_list)

def add_setpoints_to_file(file_path, setpoints):
    """
    Write all setpoints to the YAML file at once in the specified format.
    """
    config = {}
    # Read existing setpoints if file exists
    try:
        with open(file_path, 'r') as file:
            existing_config = yaml.safe_load(file)
            if existing_config and 'setpoints' in existing_config:
                config['setpoints'] = [sp for sp in existing_config['setpoints'] if sp is not None]
            else:
                config['setpoints'] = []
    except FileNotFoundError:
        config['setpoints'] = []

    # Append new setpoints, ensuring no duplicates
    for sp in setpoints:
        if sp not in config['setpoints']:
            config['setpoints'].append(sp)

    # Write the updated setpoints back to the file with the correct block style
    with open(file_path, 'w') as file:
        yaml.dump(config, file, Dumper=MyDumper, default_flow_style=False, sort_keys=False)

# Example usage
setpoints = [
    [0.0, 1.0, 2.0],
    [1.0, 2.0, 3.0]
]
add_setpoints_to_file(CONFIG_PATH, setpoints)
