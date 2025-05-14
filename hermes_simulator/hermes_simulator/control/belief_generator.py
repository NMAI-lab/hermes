from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import math

from hermes_simulator.tools.yaml_parser import load_yaml

SET_POINT = 0.8
AIM_ANGLE = 60
ANGLE_CHANGE_THRESHOLD = 0.1
SPEED = 0.4
ERROR  = SPEED * math.sin(AIM_ANGLE * math.pi / 180.0)

class BeliefGenerator(Node):
    '''
    The Node in charge of listening to the perceptions and converting them into beliefs.

    @Subscribers:
    - Listens to /lidar for the new lidar scans.
    - Listens to /beacon for the new beacons.

    @Publishers:
    - Publishes to new beliefs to /beliefs.
    '''    
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('belief_generator_node')

        # Declare the parameters
        self.declare_parameter('belief_generator_params')

        # Get the value from parameter server
        self.belief_generator_params = load_yaml(self.get_parameter('belief_generator_params').get_parameter_value().string_value)

        # The publisher for the node
        self.belief_publisher = self.create_publisher(String, self.belief_generator_params['publisher_topic'],
                                                      self.belief_generator_params['update_rate'])

        # The subscribers for the node
        self.lidar_subscriber = self.create_subscription(String, 
                                                         self.belief_generator_params['lidar_subscriber_topic'],
                                                         self.decode_lidar, 
                                                         self.belief_generator_params['update_rate'])

        self.count = 0

    def decode_lidar(self, lidar_data):
        '''
        Decodes the lidar data and generates the right action.

        Parameters:
        - lidar_data(String): the lidar distances and angles.

        Publishes a wall follow message.
        '''
        self.count += 1

        if self.count % 10 == 0:
            self.get_logger().info('decoding this lidar data {}'.format(lidar_data.data))
            lidar_data_split = lidar_data.data.split(':')
            distance = float(lidar_data_split[0])
            angle = -1 * float(lidar_data_split[1])

            res_angle = 0
            if distance > SET_POINT + ERROR:
                res_angle = -1 * AIM_ANGLE + angle
            elif distance < SET_POINT - ERROR:
                res_angle = AIM_ANGLE + angle
            else:
                res_angle = angle

            angular_speed = res_angle * math.pi / 180.0

            action_message = String()
            # For minor changes avoid big arcs.
            if abs(angular_speed) > ANGLE_CHANGE_THRESHOLD:
                linear_speed = SPEED / 2
            else:
                linear_speed = SPEED

            action_message.data = 'Twist:' + str(linear_speed) + ':' + str(angular_speed)
            self.belief_publisher.publish(action_message)

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    belief_generator = BeliefGenerator()
    rclpy.spin(belief_generator)