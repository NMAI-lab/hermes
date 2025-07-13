import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import time

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from

class DockSensor(Node):
    '''
    The Node in charge of listening to the dock status.

    @Subscribers:
    - Listens to /dock (in the simulator) and /dock_status (on the robot) for new dock updates.

    @Publishers:
    - Publishes new updates to /dock_state.
    '''
    def __init__(self):
        super().__init__('dock_sensor_node')

        # Declare the parameters
        self.declare_parameter('sensor_params')
        self.declare_parameter('mode')

        # Get the value from parameter server
        self.sensor_params = load_yaml(self.get_parameter('sensor_params').get_parameter_value().string_value)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # Wait for the robot to launch
        while self.count_publishers(self.sensor_params['robot_availability_topic']) == 0:
            self.get_logger().info('Waiting for the robot to launch...')
            time.sleep(self.sensor_params['idle_sleep_duration'])

        # The publishers for the node.
        self.publisher = self.create_publisher(String, self.sensor_params['publisher_topic'], 
                                               self.sensor_params['queue_size'])
        
        # The subscribers for the node.
        if self.mode == 'simulator':
            from irobot_create_msgs.msg import Dock
            subscriber_topic = self.sensor_params['simulator_subscriber_topic']
            dock_msg = Dock
        else:
            from irobot_create_msgs.msg import DockStatus
            subscriber_topic = self.sensor_params['robot_subscriber_topic']
            dock_msg = DockStatus

        self.dock_info_sub = self.create_subscription(dock_msg, subscriber_topic, self.dock_info_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

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
