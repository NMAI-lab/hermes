from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import math

class LidarSensor(Node):
    '''
    The Node in charge of listening to the lidar sensor.

    @Subscribers:
    - Listens to /scan for new lidar scans.

    @Publishers:
    - Publishes new updates to /perceptions.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.

        Parameters:
        - params(dict): The lidar parameters.
        '''
        super().__init__('lidar_sensor')

        # Declare parameters with defaults
        self.declare_parameter('publisher_topic', '/scan')

        # Get the value from parameter server
        published_topic = self.get_parameter('publisher_topic').get_parameter_value().string_value

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, 'lidar' , 10)
        
        # The subscribers for the node.
        self.lidar_info_sub = self.create_subscription(LaserScan, published_topic, self.scan_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)

    def scan_callback(self, scan):
        '''
        The callback for /scan.
        Reads the lidar scan and acts accordingly.

        Parameters:
        - scan(LaserScan): The current lidar scan.
        '''
        self.get_logger().info("I got a scan from {} to {} with increment {}".format(scan.angle_min, scan.angle_max, scan.angle_increment))

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    lidar_sensor = LidarSensor()
    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()