import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from irobot_create_msgs.action import DockServo

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

        # Whether the robot is attempting to dock or not.
        self.docking = False

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
        self.docking_client = ActionClient(self, DockServo, self.action_translator_params['dock_publisher_topic'])

        # The subscribers for the node
        self.action_subscriber = self.create_subscription(String, self.action_translator_params['subscriber_topic'],
                                                          self.decode_action, 
                                                          self.action_translator_params['queue_size'])
    def decode_action(self, action):
        '''
        Decodes the given action and sends it.

        Parameters:
        - action(String): the action to translate.

        Publishes the appropriate action message.
        '''

        # The robot is already docking!
        if self.docking:
            return

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

            self.drive_publisher.publish(message)
        elif action_name == 'dock':
            self.get_logger().info("Attempting to dock")
            self.docking = True
            dock_goal_future = self.docking_client.send_goal_async(DockServo.Goal())
        else:
            self.get_logger().info('GOT AN INVALID ACTION {}'.format(action_name))
        


def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)