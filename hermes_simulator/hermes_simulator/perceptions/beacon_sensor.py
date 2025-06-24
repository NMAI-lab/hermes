import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bluepy.btle import Scanner, DefaultDelegate

import time
import statistics

from hermes_simulator.tools.yaml_parser import load_yaml
from hermes_simulator.tools.string_msg_helper import create_string_msg_from, get_msg_content_as_dict


class ScanDelegate(DefaultDelegate):
    '''
    A default delegate for the scanner class.
    This enables handleNotification and handleDiscovery debugging logs
    '''
    def __init__(self):
        DefaultDelegate.__init__(self)

class BeaconSensor(Node):
    '''
    The Node in charge of listening to the beacons.
    It will try to report a reliable beacon observation.

    @Subscribers:
    - Listens to /rf_signal for new beacon signals.

    @Publishers:
    - Publishes new updates to /beacon.
    '''
    def __init__(self):
        '''
        The constructor for the node.
        Defines the necessary publishers and subscribers.
        '''
        super().__init__('beacon_sensor_node')

        # The beacons observed
        self.observations = {}

        # Declare the parameters
        self.declare_parameter('sensor_params')
        self.declare_parameter('beacons_list')
        self.declare_parameter('mode')

        # Get the value from parameter server
        self.sensor_params = load_yaml(self.get_parameter('sensor_params').get_parameter_value().string_value)
        self.beacons_list = load_yaml(self.get_parameter('beacons_list').get_parameter_value().string_value)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # Wait for the robot to launch
        while self.count_publishers(self.sensor_params['robot_availability_topic']) == 0:
            self.get_logger().info('Waiting for the robot to launch...')
            time.sleep(self.sensor_params['idle_sleep_duration'])

        # The publishers for the node.
        self.publisher = self.create_publisher(String, self.sensor_params['publisher_topic'], self.sensor_params['queue_size'])
        
        if self.mode == 'rf':
            # The subscribers for the node.
            self.beacon_info_sub = self.create_subscription(String, self.sensor_params['subscriber_topic'], 
                                                            self.rf_signal_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)
        else:
            # The Bluetooth scanner for the node.
            self.scanner = Scanner().withDelegate(ScanDelegate())
        
        # Timer set up.
        self.beacon_scan_timer = self.create_timer(self.sensor_params['scan_rate'], self.check_for_beacons) 

    def rf_signal_callback(self, rf_signal):
        '''
        The callback for /rf_signal.
        Reads the RSSI signal directly..

        Parameters:
        - rf_signal(String): The current signal for the beacon.
        '''
        beacon_data = get_msg_content_as_dict(rf_signal)
        if beacon_data['SignalStrength'] >= self.sensor_params['beacon_detection_rssi_threshold']:
            current_beacon = beacon_data['Beacon']
            current_rssi = beacon_data['SignalStrength']
            
            self.observe_beacon(current_beacon, current_rssi)

    def observe_beacon(self, current_beacon, current_rssi):
        '''
        Updates the observation stack with the current beacon.

        Parameters:
        - current_beacon(String): adds the current beacon to the observation stack.
        - current_rssi(Int): the rssi of the currently observed beacon.
        '''
        if current_beacon in self.observations:
            self.observations[current_beacon].append(current_rssi)
        else:
            self.observations[current_beacon] = [current_rssi]

        # Update the stack
        if len(self.observations[current_beacon]) > self.sensor_params['beacon_observation_stack_size']:
            self.observations[current_beacon].pop(0)

    def perform_bluetooth_scan(self):
        '''
        Performs a Bluetooth scan to find beacons.
        '''
        self.get_logger().info("I AM HEREEEE")
        devices = self.scanner.scan(self.sensor_params['scan_duration']) # Listen for ADV_IND packages.

        # For each scanned device check if device address matches beacon in list
        for dev in devices:
            for beacon in self.beacons_list:
                if beacon  == dev.addr:
                    # Observes the beacon only if within range.
                    beacon_rssi = int(dev.rssi)
                    if beacon_rssi >= self.sensor_params['beacon_detection_rssi_threshold']:
                        self.observe_beacon(self.beacons_list[beacon], beacon_rssi)
                    break

    def check_for_beacons(self):
        '''
        The callback for the timer.
        If the Bluetooth option is selected: it performs a scan for the available Bluetooth devices.
        '''
        # First perform a Bluetooth scan if it is bluetooth sensor
        if self.mode == 'bluetooth':
            self.perform_bluetooth_scan()

        # Find the best observed beacon so far
        closest_beacon = None
        highest_average_rssi = -100 # a really low value indicating the beacon was not observed
        for beacon in self.observations:
            # Only use the beacons that have been observed enough!
            if len(self.observations[beacon]) != self.sensor_params['beacon_observation_stack_size']:
                continue

            average_rssi = statistics.mean(self.observations[beacon])

            if average_rssi > highest_average_rssi:
                highest_average_rssi = average_rssi
                closest_beacon = beacon

        # Do not have sufficient information to report a beacon
        if closest_beacon is None:
            return
            
        self.get_logger().info("Beacon {} was chosen from the following observations {}".format(closest_beacon, self.observations))
                
        # Clear the observations!
        self.observations = {}
        self.publisher.publish(create_string_msg_from({
            'beacon': closest_beacon
        }))

def main(args=None):
    '''
    Starts up the node. 
    '''
    rclpy.init(args=args)
    beacon_sensor = BeaconSensor()
    rclpy.spin(beacon_sensor)
    
if __name__ == '__main__':
    main()