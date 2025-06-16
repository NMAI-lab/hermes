import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import Dock
from std_msgs.msg import String

import time

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from

class DockSensor(Node):
    '''
    The Node in charge of listening to the dock status.

    @Subscribers:
    - Listens to /dock for new dock updates.

    @Publishers:
    - Publishes new updates to /dock_status.
    '''
    def __init__(self):
        super().__init__('dock_sensor_node')

        # Declare the parameters
        self.declare_parameter('sensor_params')

        # Get the value from parameter server
        self.sensor_params = load_yaml(self.get_parameter('sensor_params').get_parameter_value().string_value)

        # Wait for the robot to launch
        while self.count_publishers(self.sensor_params['robot_availability_topic']) == 0:
            self.get_logger().info('Waiting for the robot to launch...')
            time.sleep(self.sensor_params['idle_sleep_duration'])

        # The publishers for the node.
        self.publisher = self.create_publisher(String, self.sensor_params['publisher_topic'], 
                                               self.sensor_params['queue_size'])
        
        self.dock_info_sub = self.create_subscription(Dock, self.sensor_params['subscriber_topic'], 
                                                      self.dock_info_callback,
                                                      qos_profile=rclpy.qos.qos_profile_sensor_data)

    def dock_info_callback(self, dock_data):
        '''
        The callback for /dock.
        Reads the dock data information and acts accordingly.

        Parameters:
        - dock_data(Dock): The current dock info.

        Publishes a scan update to /dock_status.
        '''  
        self.publisher.publish(create_string_msg_from({
            "dock_station": {
                "is_docked": dock_data.is_docked,
                "dock_visible": dock_data.dock_visible
            }
        }))

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    dock_sensor = DockSensor()
    rclpy.spin(dock_sensor)
    
if __name__ == '__main__':
    main()