import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist

import time 

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from, get_msg_content_as_dict

class ActionTranslator(Node):
    '''
    The Node in charge of listening to the actions.

    @Subscribers:
    - Listens to /actions for new actions.

    @Publishers:
    - Publishes Twist commands to /cmds_vel to move the robot.
    - Publishes to /dock to dock the robot.
    - Publishes to /undock to undock the robot.
    '''    
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('action_translator_node')

        # Declare the parameters
        self.declare_parameter('action_translator_params')

        # Get the value from parameter server
        self.action_translator_params = load_yaml(self.get_parameter('action_translator_params').get_parameter_value().string_value)

        # The publishers for the node
        self.action_status_publisher = self.create_publisher(String, 
                                                             self.action_translator_params['action_status_publisher_topic'],
                                                             self.action_translator_params['queue_size'])
        self.drive_publisher = self.create_publisher(Twist, self.action_translator_params['movement_publisher_topic'],
                                                     self.action_translator_params['queue_size'])
        self.undock_publisher = self.create_publisher(Empty, self.action_translator_params['dock_publisher_topic'],
                                                      self.action_translator_params['queue_size'])
        self.dock_publisher = self.create_publisher(Empty, self.action_translator_params['undock_publisher_topic'],
                                                    self.action_translator_params['queue_size'])

        # The subscribers for the node
        self.action_subscriber = self.create_subscription(String, self.action_translator_params['subscriber_topic'],
                                                          self.decode_action, 
                                                          self.action_translator_params['queue_size'])

        # Wait for the robot to launch
        while self.count_publishers(self.action_translator_params['robot_availability_topic']) == 0:
            self.get_logger().info('Waiting for for the robot to launch...')
            time.sleep(self.action_translator_params['idle_sleep_duration'])

    def decode_action(self, action):
        '''
        Decodes the given action and sends it.

        Parameters:
        - action(String): the action to translate.

        Publishes the appropriate action message.
        '''
        action_data = get_msg_content_as_dict(action)
        self.get_logger().info('Decoding this action {}'.format(action.data))
        self.action_status_publisher.publish(create_string_msg_from({
            'action_id': action_data['action_id']
        }))

        action_term = action_data['name']
        if '(' in action_term and ')' in action_term:
            action_name = action_term[:action_term.index('(')]
            action_params = list(map(lambda x: float(x), action_term[action_term.index('(')+1:action_term.index(')')].split(',')))
        else:
            action_name = action_term
            action_params = []

        if action_name == 'cmd_vel':
            message = Twist()
            message.linear.x = action_params[0]
            message.linear.y = action_params[1]
            message.linear.z = action_params[2]
            message.angular.x = action_params[3]
            message.angular.y = action_params[4]
            message.angular.z = action_params[5]
        else:
            message = Empty()

        # self.get_logger().info('Publishing this action {}'.format(message))
        self.drive_publisher.publish(message)


def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)