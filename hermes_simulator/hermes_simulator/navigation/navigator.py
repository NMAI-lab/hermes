import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json

from hermes_simulator.navigation.map_utilities import MapUtilities

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from, get_msg_content_as_dict

class Navigator(Node):
    '''
    The Node in charge of managing the robot's navigation.

    @Subscribers:
    - Listens to /beacon for new beacons.

    @Publishers:
    - Publishes new updates to /navigation.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('navigator_node')

        # Declare the parameters
        self.declare_parameter('navigator_params')
        self.declare_parameter('map_file')
        self.declare_parameter('destination')

        # Get the value from parameter server
        self.navigator_params = load_yaml(self.get_parameter('navigator_params').get_parameter_value().string_value)
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.destination = self.get_parameter('destination').get_parameter_value().string_value

        with open(map_file) as json_data:
            self.beacons = json.load(json_data)['beacons']
            self.map_utilities = MapUtilities(beacons=self.beacons, logger=self.get_logger())

        self.previous_beacon = None
        self.observations = []

        # The publishers for the node
        self.publisher = self.create_publisher(String, self.navigator_params['publisher_topic'], self.navigator_params['queue_size'])

        # The subscribers for the node
        self.beacon_subscriber = self.create_subscription(String, 
                                                          self.navigator_params['subscriber_topic'],
                                                          self.decode_beacon, 
                                                          self.navigator_params['queue_size'])
                                                          
    def decode_beacon(self, beacon_data):
        '''
        Decodes the beacon data and publishes a navigation event.

        Parameters:
        - beacon_data(String): the beacon data.
        '''
        current_beacon = get_msg_content_as_dict(beacon_data)['beacon']
        self.observations.append(current_beacon)

        # Update the stack
        if len(self.observations) > self.navigator_params['beacon_observation_stack_size']:
            self.observations.pop(0)

        # Consistently observed the same beacon
        if len(self.observations) == self.navigator_params['beacon_observation_stack_size'] and len(set(self.observations)) == 1:
            # Observed a new intersection so a path is needed.
            # Make sure the beacon was not previously observed!
            if self.beacons[current_beacon]['type'] == 'intersection' and current_beacon != self.previous_beacon:
                navigation_instruction = self.map_utilities.get_turn_direction(curr_beacon=current_beacon, destination=self.destination, prev_beacon=self.previous_beacon)
                self.publisher.publish(create_string_msg_from({
                    'navigation': navigation_instruction,
                }))

            # Save the observed beacon
            self.previous_beacon = current_beacon


def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)