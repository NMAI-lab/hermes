import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import time
import statistics

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from, get_msg_content_as_dict

class BeaconSensor(Node):
    '''
    The Node in charge of listening to the beacons.
    It will try to report a reliable beacon observation.

    @Subscribers:
    - Listens to /rf_signal for new beacon signals.

    @Publishers:
    - Publishes new updates to /beacon.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('beacon_sensor_node')

        # The beacons observed
        self.observations = {}

        # Declare the parameters
        self.declare_parameter('sensor_params')

        # Get the value from parameter server
        self.sensor_params = load_yaml(self.get_parameter('sensor_params').get_parameter_value().string_value)

        # Wait for the robot to launch
        while self.count_publishers(self.sensor_params['robot_availability_topic']) == 0:
            self.get_logger().info('Waiting for the robot to launch...')
            time.sleep(self.sensor_params['idle_sleep_duration'])

        # The publishers for the node.
        self.publisher = self.create_publisher(String, self.sensor_params['publisher_topic'], self.sensor_params['queue_size'])
        
        # The subscribers for the node.
        self.beacon_info_sub = self.create_subscription(String, self.sensor_params['subscriber_topic'], 
                                                        self.rf_signal_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

    def rf_signal_callback(self, rf_signal):
        '''
        The callback for /rf_signal.
        Reads the RSSI signal directly..

        Parameters:
        - rf_signal(String): The current signal for the beacon.
        '''
        beacon_data = get_msg_content_as_dict(rf_signal)
        if beacon_data['SignalStrength'] >= self.sensor_params['beacon_detection_rssi_threshold']:
            current_beacon = beacon_data['Beacon']
            current_rssi = beacon_data['SignalStrength']
            if current_beacon in self.observations:
                self.observations[current_beacon].append(current_rssi)
            else:
                self.observations[current_beacon] = [current_rssi]

            # Update the stack
            if len(self.observations[current_beacon]) > self.sensor_params['beacon_observation_stack_size']:
                self.observations[current_beacon].pop(0)

            # Find the best observed beacon so far
            closest_beacon = None
            highest_average_rssi = -100 # a really low value indicating the beacon was not observed
            for beacon in self.observations:
                # Only use the beacons that have been observed enough!
                if len(self.observations[beacon]) != self.sensor_params['beacon_observation_stack_size']:
                    continue

                average_rssi = statistics.mean(self.observations[beacon])

                if average_rssi > highest_average_rssi:
                    highest_average_rssi = average_rssi
                    closest_beacon = beacon

            # Do not have sufficient information to report a beacon
            if closest_beacon is None:
                return
            
            self.get_logger().info("Beacon {} was chosen from the following observations {}".format(closest_beacon, self.observations))
                
            # Clear the observations!
            self.observations = {}
            self.publisher.publish(create_string_msg_from({
                'beacon': closest_beacon
            }))

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    beacon_sensor = BeaconSensor()
    rclpy.spin(beacon_sensor)
    
if __name__ == '__main__':
    main()