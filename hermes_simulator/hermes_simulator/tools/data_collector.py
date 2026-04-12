import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import os
import signal
from datetime import datetime

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import get_msg_content_as_dict

class DataCollector(Node):
    '''
    The Node in charge of collecting and documenting metrics.

    @Subscribers:
    - Listens to /beliefs for new beliefs.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('data_collector')

        self.observations = {}

        # Declare the parameters
        self.declare_parameter('data_collector_params')
        self.declare_parameter('metrics_file')

        # Get the value from parameter server
        self.data_collector_params = load_yaml(self.get_parameter('data_collector_params').get_parameter_value().string_value)
        self.output_file = self.get_parameter('metrics_file').get_parameter_value().string_value
        
        self.trial_data = {
            'start_time': datetime.now().isoformat(),
            'end_time': None,
            'navigation_instructions': [],
            'wall_following': [],
            'actions': [],
            'bumps': 0,
            'docked': False
        }
        
        if os.path.exists(self.output_file):
            with open(self.output_file, 'r') as f:
                self.all_trials = json.load(f)
        else:
            self.all_trials = []

        self.flush()
        
        signal.signal(signal.SIGTERM, self.shutdown_handler)
        
        self.beliefs_subscriber = self.create_subscription(String, 
                                                           self.data_collector_params['beliefs_subscriber_topic'],
                                                           self.decode_beliefs,
                                                           self.data_collector_params['queue_size'])
        
        self.actions_subscriber = self.create_subscription(String, 
                                                           self.data_collector_params['actions_subscriber_topic'],
                                                           self.decode_actions,
                                                           self.data_collector_params['queue_size'])
        
    def decode_beliefs(self, beliefs_data):
        '''
        The callback for /beliefs.
        Reads the beliefs and collects the necessary metrics.

        Parameters:
        - beliefs_data(String): The current beliefs of the agent.
        '''
        beliefs = get_msg_content_as_dict(beliefs_data)

        if 'wall_following' in beliefs:
            self.trial_data['wall_following'].append(
                {'right_wall_distance': beliefs['wall_following']['right_wall_dist'],
                 'intersection': 'intersection' in beliefs}
                 )

        if 'bumper' in beliefs and beliefs['bumper']['bump']:
             self.trial_data['bumps'] += 1

        if 'dock_station' in beliefs and beliefs['dock_station']['is_docked']:
             self.trial_data['docked'] = True

        if 'navigation' in beliefs and beliefs['navigation'] != 'NONE':
             self.trial_data['navigation_instructions'].append(beliefs['navigation'])
        
        self.flush()

    def decode_actions(self, actions_data):
        '''
        The callback for /actions.
        Reads the actions and collects the necessary metrics.

        Parameters:
        - actions_data(String): The current actions of the agent.
        '''
        action = get_msg_content_as_dict(actions_data)
        self.trial_data['actions'].append(action['name'])
        self.flush()
    
    def shutdown_handler(self, signum, frame):
        '''
        The handler for when the node receives a SIG_TERM.
        '''
        self.get_logger().info('SIGTERM received, flushing final data...')
        self.trial_data['end_time'] = datetime.now().isoformat()
        self.flush()
        rclpy.shutdown()

    def flush(self):
        '''
        Writes down the metrics into the JSON file.
        '''
        self.trial_data['end_time'] = datetime.now().isoformat()
        if self.all_trials and self.all_trials[-1]['start_time'] == self.trial_data['start_time']:
            self.all_trials[-1] = self.trial_data
        else:
            self.all_trials.append(self.trial_data)
        
        with open(self.output_file, 'w') as f:
            json.dump(self.all_trials, f, indent=2)
            f.flush()
            os.fsync(f.fileno())

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    beacon_sensor = DataCollector()
    rclpy.spin(beacon_sensor)
    
if __name__ == '__main__':
    main()