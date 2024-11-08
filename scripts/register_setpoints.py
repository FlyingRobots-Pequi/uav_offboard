import yaml

class MyDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, indentless=False)

def represent_list(dumper, data):
    # If it's a list of numbers, use block style; otherwise, use flow style
    if all(isinstance(i, list) for i in data):
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=None)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

MyDumper.add_representer(list, represent_list)

def add_setpoint_to_file(file_path, new_setpoint):
    # Read the existing setpoints from the file
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    
    # Append the new setpoint to the list
    config['setpoints'].append(new_setpoint)
    
    # Write the updated setpoints back to the file with the correct block style
    with open(file_path, 'w') as file:
        yaml.dump(config, file, Dumper=MyDumper, default_flow_style=False)

# Usage example
new_setpoint = [-2.0, 1.0, -2.0]
add_setpoint_to_file('/home/matteus/sandbox/flying/larc_tmp/hermit_offboard/config/goto_setpoints.yaml', new_setpoint)
