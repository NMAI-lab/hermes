import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from irobot_create_msgs.action import DockServo, Dock

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

        self.current_goal_id = None

        # Declare the parameters
        self.declare_parameter('action_translator_params')
        self.declare_parameter('mode')

        # Get the value from parameter server
        self.action_translator_params = load_yaml(self.get_parameter('action_translator_params').get_parameter_value().string_value)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        if self.mode == 'simulator':
            self.dock_message = DockServo
        else:
            self.dock_message = Dock

        # The publishers for the node
        self.action_status_publisher = self.create_publisher(String, 
                                                             self.action_translator_params['action_status_publisher_topic'],
                                                             self.action_translator_params['queue_size'])
        self.drive_publisher = self.create_publisher(Twist, self.action_translator_params['movement_publisher_topic'],
                                                     self.action_translator_params['queue_size'])
        self.docking_client = ActionClient(self, self.dock_message, self.action_translator_params['dock_publisher_topic'])
        self.agent_request_publisher = self.create_publisher(String, 
                                                             self.action_translator_params['agent_request_publisher_topic'],
                                                             self.action_translator_params['queue_size'])

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

        action_data = get_msg_content_as_dict(action)
        self.get_logger().info('Decoding this action {}'.format(action.data))

        action_term = action_data['name']
        if '(' in action_term and ')' in action_term:
            action_name = action_term[:action_term.index('(')]
            action_params = list(map(lambda x: float(x), action_term[action_term.index('(')+1:action_term.index(')')].split(',')))
        else:
            action_name = action_term
            action_params = []

        if action_name == 'dock':
            self.current_goal_id = action_data['action_id']
            dock_goal_future = self.docking_client.send_goal_async(self.dock_message.Goal(), feedback_callback=self.feedback_callback)
            dock_goal_future.add_done_callback(self.goal_response_callback)
            return
        elif action_name == 'cmd_vel':
            message = Twist()
            message.linear.x = action_params[0]
            message.linear.y = action_params[1]
            message.linear.z = action_params[2]
            message.angular.x = action_params[3]
            message.angular.y = action_params[4]
            message.angular.z = action_params[5]
            self.drive_publisher.publish(message)
        elif action_name == 'request_trip':
            self.agent_request_publisher.publish(create_string_msg_from({
                'request_type': 'request_trip'
            }))
        else:
            self.get_logger().info('GOT AN INVALID ACTION {}'.format(action_name))

        # Send the action status
        self.action_status_publisher.publish(create_string_msg_from({
            'action_id': action_data['action_id']
        }))

    def goal_response_callback(self, future):
        '''
        The callback function to obtain the goal response.

        Parameters:
        - future(Goal): the goal response. 
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        The callback function to obtain the result of the goal.

        Parameters:
        - future(Goal): the goal response. 
        '''
        self.get_logger().info(f'Goal Result: {future.result().result}')
        self.action_status_publisher.publish(create_string_msg_from({
            'action_id': self.current_goal_id
        }))
        self.current_goal_id = None

    def feedback_callback(self, feedback_msg):
        '''
        The callback for the goal feedback.

        Parameters:
        - feedback_msg(Feedback): the feedback message.
        '''
        self.get_logger().info(f'Goal feedback: {str(feedback_msg.feedback)}')

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)