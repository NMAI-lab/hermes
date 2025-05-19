from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import math
import json

from hermes_simulator.tools.yaml_parser import load_yaml

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

        self.goal = "wall_follow"
        self.lidar_beliefs = []

        # Declare the parameters
        self.declare_parameter('belief_generator_params')

        # Get the value from parameter server
        self.belief_generator_params = load_yaml(self.get_parameter('belief_generator_params').get_parameter_value().string_value)

        # The publishers for the node
        self.beliefs_publisher = self.create_publisher(String, self.belief_generator_params['publisher_topic'],
                                                       self.belief_generator_params['queue_size'])

        # The subscribers for the node
        self.lidar_subscriber = self.create_subscription(String, 
                                                         self.belief_generator_params['lidar_subscriber_topic'],
                                                         self.decode_lidar, 
                                                         self.belief_generator_params['queue_size'])

        # A timer to send a status update
        self.update_timer = self.create_timer(self.belief_generator_params['update_rate'], self.send_update)


    def send_update(self):
        '''
        Sends an update of the current snapshot of the system.

        Publishes a state update message.
        '''
        update_message = String()
        update_dict = {'goal': self.goal}
        update_dict['beliefs'] = self.lidar_beliefs
        update_message.data = json.dumps(update_dict)
        self.get_logger().info('Simulator state update {}'.format(update_message.data))
        self.beliefs_publisher.publish(update_message)

    def decode_lidar(self, lidar_data):
        '''
        Decodes the lidar data and generates the right action.

        Parameters:
        - lidar_data(String): the lidar distances and angles.

        Publishes a wall follow message.
        '''
        lidar_data = json.loads(lidar_data.data)
        self.lidar_beliefs = ['facing_wall({distance}, {angle})'.format(distance=lidar_data['right_wall_dist'], angle=lidar_data['right_wall_angle'])]
        '''
        distance = lidar_data['right_wall_dist']
        angle = lidar_data['right_wall_angle']
        
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

        action_message.data = json.dumps({
            'name': 'Twist',
            'linear_x': linear_speed,
            'linear_y': 0,
            'linear_z': 0,
            'angular_x': 0,
            'angular_y': 0,
            'angular_z': angular_speed,
        })
        self.belief_publisher.publish(action_message)
        '''

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    belief_generator = BeliefGenerator()
    rclpy.spin(belief_generator)