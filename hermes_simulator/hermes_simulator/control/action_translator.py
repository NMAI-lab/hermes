from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist

import math

from hermes_simulator.tools.yaml_parser import load_yaml

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
        self.drive_publisher = self.create_publisher(Twist, self.action_translator_params['movement_publisher_topic'],
                                                     2)
        self.undock_publisher = self.create_publisher(Empty, self.action_translator_params['dock_publisher_topic'],
                                                      self.action_translator_params['update_rate'])
        self.dock_publisher = self.create_publisher(Empty, self.action_translator_params['undock_publisher_topic'],
                                                    self.action_translator_params['update_rate'])

        # The subscribers for the node
        self.actionSubscriber = self.create_subscription(String, self.action_translator_params['subscriber_topic'],
                                                         self.decode_action, 
                                                         self.action_translator_params['update_rate'])

    def decode_action(self, action):
        '''
        Decodes the given action and sends it.

        Parameters:
        - action(String): the action to translate.

        Publishes the appropriate action message.
        '''
        action_split = action.data.split(':')
        self.get_logger().info('decoding this action {}'.format(action.data))

        if action_split[0] == 'Twist':
            message = Twist()
            message.linear.x = float(action_split[1])
            message.angular.z = float(action_split[2])
            self.drive_publisher.publish(message)


def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)