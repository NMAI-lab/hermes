from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import time

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from

class BumperSensor(Node):
    '''
    The Node in charge of listening to the bumper sensor.

    @Subscribers:
    - Listens to /bump to read the current state of the bumper sensor.

    @Publishers:
    - Publishes new messages to /bumper_status.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('bumper_sensor')
        
        # Sets the default values for the sensor.
        self.bump_count = 0
        self.bumper_pressed = False

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
        
        # The subscribers for the node.
        self.bumper_info_sub = self.create_subscription(HazardDetectionVector, self.sensor_params['subscriber_topic'], 
                                                        self.hazard_info_callback, 
                                                        qos_profile=QoSProfile(
                                                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                            depth=self.sensor_params['queue_size']
                                                        ))
    
    def hazard_info_callback(self, hazard_data):
        '''
        The callback for /hazard_detection.
        Reads the hazard data and acts accordingly.

        Parameters:
        - hazard_data(HazardDetectionVector): The new hazard data received.
        '''
        hazards = hazard_data.detections
        got_hazard = False
        if len(hazards) != 0:
            for h in hazards:
                # Type one is for bump events (from the Create 3 Docks)
                if h.type == 1:
                    got_hazard = True
                    break

        if got_hazard != self.bumper_pressed or self.bump_count > self.sensor_params['bump_count_limit']:
            self.bump_count = 0
            self.bumper_pressed = got_hazard
            self.publisher.publish(create_string_msg_from({
                "bumper": {
                    "bump": got_hazard,
                }
            }))

        self.bump_count += 1

def main():
    '''
    Starts up the node. 
    '''
    rclpy.init()
    bumper_sensor = BumperSensor()
    rclpy.spin(bumper_sensor)

if __name__ == '__main__':
    main()