import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from, get_msg_content_as_dict

class BeaconSensor(Node):
    '''
    The Node in charge of listening to the beacons.

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

        # Declare the parameters
        self.declare_parameter('sensor_params')

        # Get the value from parameter server
        self.sensor_params = load_yaml(self.get_parameter('sensor_params').get_parameter_value().string_value)

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
            self.publisher.publish(create_string_msg_from({
                'beacon': beacon_data['Beacon'],
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