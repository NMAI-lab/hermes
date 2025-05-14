from std_msgs.msg import String
import rclpy
from rclpy.node import Node

import math

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

        Parameters:
        - params(dict): The beacon parameters.
        '''
        super().__init__('beacon_sensor_node')

        # Declare parameters with defaults
        self.declare_parameter('publisher_topic', '/rf_signal')

        # Get the value from parameter server
        published_topic = self.get_parameter('publisher_topic').get_parameter_value().string_value

        # The publishers for the node.
        self.publisher = self.create_publisher(String, 'beacon' , 10)
        
        # The subscribers for the node.
        self.beacon_info_sub = self.create_subscription(String, published_topic, self.rf_signal_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

    def rf_signal_callback(self, rf_signal):
        '''
        The callback for /rf_signal.
        Reads the RSSI signal directly..

        Parameters:
        - rf_signal(String): The current signal for the beacon.
        '''
        # self.get_logger().info("I got a rf_signal: {}".format(rf_signal.data))

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    beacon_sensor = BeaconSensor()
    rclpy.spin(beacon_sensor)
    
if __name__ == '__main__':
    main()