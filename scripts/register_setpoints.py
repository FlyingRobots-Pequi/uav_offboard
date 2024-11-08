import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class MyDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, indentless=False)

def represent_list(dumper, data):
    # If it's a list of numbers, use block style; otherwise, use flow style
    if all(isinstance(i, list) for i in data):
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=None)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

# Register the custom list representer with MyDumper
MyDumper.add_representer(list, represent_list)

class SetpointAdderNode(Node):
    def __init__(self):
        super().__init__('setpoint_adder')
        
        # Retrieve the path to the configuration file
        package_path = get_package_share_directory('hermit_offboard')
        self.config_file_path = os.path.join(package_path, 'config', 'goto_setpoints.yaml')
        
        # Example setpoint to add
        new_setpoint = [-2.0, 1.0, -2.0]
        
        # Call function to add setpoint to file
        self.add_setpoint_to_file(self.config_file_path, new_setpoint)
        self.get_logger().info(f"Setpoint {new_setpoint} added to {self.config_file_path}")

    def add_setpoint_to_file(self, file_path, new_setpoint):
        # Read the existing setpoints from the file
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Append the new setpoint to the list
        config['setpoints'].append(new_setpoint)
        
        # Write the updated setpoints back to the file with the correct block style
        with open(file_path, 'w') as file:
            yaml.dump(config, file, Dumper=MyDumper, default_flow_style=False)

def main(args=None):
    rclpy.init(args=args)
    node = SetpointAdderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
