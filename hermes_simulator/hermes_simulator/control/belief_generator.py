import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from, get_msg_content_as_dict

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

        self.lidar_beliefs = {}
        self.navigation_beliefs = {}

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
        self.navigation_subscriber = self.create_subscription(String, 
                                                              self.belief_generator_params['navigation_subscriber_topic'],
                                                              self.decode_navigation, 
                                                              self.belief_generator_params['queue_size'])

        # A timer to send a status update
        self.update_timer = self.create_timer(self.belief_generator_params['update_rate'], self.send_update)

    def send_update(self):
        '''
        Sends an update of the current snapshot of the system.

        Publishes a state update message.
        '''
        # Collect the current perceptions
        update_dict = {}
        update_dict.update(self.lidar_beliefs)
        update_dict.update(self.navigation_beliefs)
        
        # Clear the perceptions
        self.lidar_beliefs = {}
        self.navigation_beliefs = {}

        update_message = create_string_msg_from(update_dict)
        self.get_logger().info('Simulator state update {}'.format(update_message.data))
        self.beliefs_publisher.publish(update_message)

    def decode_lidar(self, lidar_data):
        '''
        Decodes the lidar data and saves the belief.

        Parameters:
        - lidar_data(String): the lidar distances and angles.
        '''
        self.lidar_beliefs = get_msg_content_as_dict(lidar_data)

    def decode_navigation(self, navigation_data):
        '''
        Decodes the navigation data and saves the belief.

        Parameters:
        - navigation_data(String): the navigation data.
        '''
        self.navigation_beliefs = get_msg_content_as_dict(navigation_data)

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    belief_generator = BeliefGenerator()
    rclpy.spin(belief_generator)