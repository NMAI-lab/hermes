import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import math
import statistics
import time

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from

class LidarSensor(Node):
    '''
    The Node in charge of listening to the lidar sensor.

    @Subscribers:
    - Listens to /scan for new lidar scans.

    @Publishers:
    - Publishes new updates to /lidar.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('lidar_sensor_node')

        # Declare the parameters
        self.declare_parameter('sensor_params')
        self.declare_parameter('lidar_params')
        self.declare_parameter('map_params')

        # Get the value from parameter server
        self.sensor_params = load_yaml(self.get_parameter('sensor_params').get_parameter_value().string_value)
        self.lidar_params = load_yaml(self.get_parameter('lidar_params').get_parameter_value().string_value)['lidar_node']['ros__parameters']
        self.map_params = load_yaml(self.get_parameter('map_params').get_parameter_value().string_value)

        # Wait for the robot to launch
        while self.count_publishers(self.sensor_params['robot_availability_topic']) == 0:
            self.get_logger().info('Waiting for the robot to launch...')
            time.sleep(self.sensor_params['idle_sleep_duration'])

        # The publishers for the node.
        self.publisher = self.create_publisher(String, self.sensor_params['publisher_topic'], 
                                               self.sensor_params['queue_size'])
        
        # The subscribers for the node.
        self.lidar_info_sub = self.create_subscription(LaserScan, self.sensor_params['subscriber_topic'], 
                                                       self.scan_callback, 
                                                       qos_profile=rclpy.qos.qos_profile_sensor_data)

    def scan_callback(self, scan):
        '''
        The callback for /scan.
        Reads the lidar scan and acts accordingly.

        Parameters:
        - scan(LaserScan): The current lidar scan.

        Publishes a scan update to /lidar.
        '''        
        wall_following_info, intersection_info  = self.calculate(scan)

        lidar_info = {
            'wall_following' : {
                'right_wall_dist': wall_following_info[0],
                'right_wall_angle': wall_following_info[1]
            }
        }

        if intersection_info is not None:
            lidar_info['intersection'] = {
                'forward_distance': intersection_info[0],
                'l_turn_distance': intersection_info[1],
                'u_turn_distance': intersection_info[2]
            }

        self.publisher.publish(create_string_msg_from(lidar_info))

    def calculate(self, scan):
        '''
        Calculates the robot's distance with its surroundings.

        Parameters:
        - scan: The current lidar scan.

        Returns:
        - (Tuple, Tuple): the wall following and intersection handling information.
        Checks to see if there is an intersection and if so it returns its estimated size
        '''
        sample_index = 0
        min_distance = self.lidar_params['max_range']
        angle_with_wall = 0
        curr_angle = scan.angle_min
        right_wall_distances = []
        front_wall_distances = []
        left_wall_distances = []

        forward_distances = []
        l_turn_distances = []
        u_turn_distances = []
        while curr_angle < scan.angle_max:
            angle_degrees = math.degrees(curr_angle)
            curr_angle += scan.angle_increment

            curr_distance = scan.ranges[sample_index]
            sample_index += 1
        
            if curr_distance == math.inf:
                curr_distance = self.lidar_params['max_range']

            curr_distance = round(curr_distance, 3)

            #wall_following
            if angle_degrees >= self.sensor_params['wall_follow_min_angle'] and angle_degrees <= self.sensor_params['wall_follow_max_angle'] and curr_distance < min_distance:
                min_distance = curr_distance
                angle_with_wall = angle_degrees

            #intersection detection
            if angle_degrees >= self.sensor_params['right_wall_min_angle'] and angle_degrees <= self.sensor_params['right_wall_max_angle']:
                right_wall_distances.append(curr_distance)
            if angle_degrees >= self.sensor_params['front_wall_min_angle'] and angle_degrees <= self.sensor_params['front_wall_max_angle']:
                front_wall_distances.append(curr_distance)
            if angle_degrees >= self.sensor_params['left_wall_min_angle'] and angle_degrees <= self.sensor_params['left_wall_max_angle']:
                left_wall_distances.append(curr_distance)

            #intersection size estimation
            if angle_degrees >= self.sensor_params['forward_min_angle'] and angle_degrees <= self.sensor_params['forward_max_angle']:
                forward_distances.append(curr_distance)
            if angle_degrees >= self.sensor_params['l_turn_min_angle'] and angle_degrees <= self.sensor_params['l_turn_max_angle']:
                l_turn_distances.append(curr_distance)
            if angle_degrees >= self.sensor_params['u_turn_min_angle'] and angle_degrees <= self.sensor_params['u_turn_max_angle']:
                u_turn_distances.append(curr_distance)
            
        # Capture wall following info
        wall_following_info = (min_distance, angle_with_wall + 90)

        # Intersection detection
        # Lost wall constants
        lost_wall_distance_threshold = self.sensor_params['lost_wall_tunnel_width_percent_threshold'] * self.map_params['tunnel_width']
        lost_wall_percent_threshold = self.sensor_params['intersection_detection_lost_wall_percent_threshold']
        # Count the number of lost walls
        num_lost_walls_right = sum(distance >= lost_wall_distance_threshold for distance in right_wall_distances)
        num_lost_walls_front = sum(distance >= lost_wall_distance_threshold for distance in front_wall_distances)
        num_lost_walls_left = sum(distance >= lost_wall_distance_threshold for distance in left_wall_distances)
        # Test for lost walls
        lost_wall_on_right = len(right_wall_distances) == 0 or num_lost_walls_right/len(right_wall_distances) >= lost_wall_percent_threshold
        lost_wall_on_front = len(front_wall_distances) == 0 or num_lost_walls_front/len(front_wall_distances) >= lost_wall_percent_threshold
        lost_wall_on_left = len(left_wall_distances) == 0 or num_lost_walls_left/len(left_wall_distances) >= lost_wall_percent_threshold

        # Estimate the size of the intersection
        forward_distance = round(statistics.mean(forward_distances), 3)
        left_turn_distance = round(statistics.mean(l_turn_distances), 3)
        u_turn_distance = round(statistics.mean(u_turn_distances), 3)

        # Check to see if at least 2 of the sides are lost to set an intersecton
        # If there is an intersection, return its estimated size
        intersection_info = (forward_distance, left_turn_distance, u_turn_distance) if (lost_wall_on_right + lost_wall_on_front + lost_wall_on_left) >= 2 else None

        return wall_following_info, intersection_info

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    lidar_sensor = LidarSensor()
    rclpy.spin(lidar_sensor)
    
if __name__ == '__main__':
    main()