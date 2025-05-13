from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import math
import yaml

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
        super().__init__('lidar_sensor_node')

        # Declare the parameters
        self.declare_parameter('sensor_params')
        self.declare_parameter('lidar_params')

        # Get the value from parameter server
        self.sensor_params = self.load_yaml(self.get_parameter('sensor_params').get_parameter_value().string_value)
        self.lidar_params = self.load_yaml(self.get_parameter('lidar_params').get_parameter_value().string_value)['lidar_node']['ros__parameters']

        publisher_topic = self.sensor_params['publisher_topic']
        subscriber_topic = self.sensor_params['subscriber_topic']
        update_rate = self.sensor_params['update_rate']

        # The publishers for the node.
        self.publisher_ = self.create_publisher(String, publisher_topic , update_rate)
        
        # The subscribers for the node.
        self.lidar_info_sub = self.create_subscription(LaserScan, subscriber_topic, self.scan_callback,         
                                                       qos_profile=rclpy.qos.qos_profile_sensor_data)

    def scan_callback(self, scan):
        '''
        The callback for /scan.
        Reads the lidar scan and acts accordingly.

        Parameters:
        - scan(LaserScan): The current lidar scan.

        Publishes a scan update to /lidar.
        '''
        calc = String()
        
        right_wall_dist, right_wall_angle  = self.calculate(scan)
        calc.data = str(right_wall_dist) + ":" + str(right_wall_angle)

        self.publisher_.publish(calc)
        # self.get_logger().info('right_wall_distance: {}, right_wall_angle: {}'.format(right_wall_dist, right_wall_angle))

    def calculate(self, scan):
        '''
        Calculates the robot's distance with its surroundings.

        Parameters:
        - scan: The current lidar scan.

        Returns:
        - (float, float): the distance and angle with the right wall.
        '''
        sample_index = 0
        min_distance = self.lidar_params['max_range']
        angle_with_wall = 0
        curr_angle = scan.angle_min
        while curr_angle < scan.angle_max:
            angle_degrees = math.degrees(curr_angle)
            curr_angle += scan.angle_increment

            curr_distance = scan.ranges[sample_index]
            sample_index += 1
        
            if curr_distance == math.inf:
                continue

            #wall_following
            if angle_degrees >= self.sensor_params['wall_follow_min_angle'] and angle_degrees <= self.sensor_params['wall_follow_max_angle'] and curr_distance < min_distance:
                min_distance = curr_distance
                angle_with_wall = angle_degrees

        return min_distance, angle_with_wall

    def load_yaml(self, path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    lidar_sensor = LidarSensor()
    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()