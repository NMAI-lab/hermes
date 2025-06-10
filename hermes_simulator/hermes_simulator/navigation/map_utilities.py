import heapq

DIRECTIONS = {
    "north": {
        "north": "U_TURN",
        "east": "L_TURN",
        "south": "FORWARD",
        "west": "WALL_FOLLOW"
    },
    "east": {
        "north": "WALL_FOLLOW",
        "east": "U_TURN",
        "south": "L_TURN",
        "west": "FORWARD"
    },
    "south": {
        "north": "FORWARD",
        "east": "WALL_FOLLOW",
        "south": "U_TURN",
        "west": "L_TURN"
    },
    "west": {
        "north": "L_TURN",
        "east": "FORWARD",
        "south": "WALL_FOLLOW",
        "west": "U_TURN"
    }
}

class MapUtilities():
    def __init__(self, beacons):
        self.beacons = beacons
        self.current_path = []
    
    def find_shortest_path(self, start, end):
        '''
        Finds a sequence of beacons to follow from start to end.
        Essentially an implementation of Dijkstra!

        Parameters:
        - start(String): the starting beacon.
        - end(String): the final beacon.

        Returns:
        - [String]: a sequence of beacons to follow
        '''
        distances = {b: float('inf') for b in self.beacons}
        distances[start] = 0
        previous_nodes = {b: None for b in self.beacons}
        to_visit_queue = [(0, start)]

        while to_visit_queue:
            current_distance, current_beacon = heapq.heappop(to_visit_queue) 

            if current_distance > distances[current_beacon]:
                continue

            if current_beacon == end:
                break

            for direction in DIRECTIONS:
                if direction in self.beacons[current_beacon]:
                    distance = current_distance + self.beacons[current_beacon][direction]['distance']
                    next_beacon = self.beacons[current_beacon][direction]['name']
                    if distance < distances[next_beacon]:
                        distances[next_beacon] = distance
                        heapq.heappush(to_visit_queue, (distance, next_beacon))
                        previous_nodes[next_beacon] = current_beacon

        # Could not find a path
        if previous_nodes[end] is None:
            return []

        path = []
        current = end
        while current is not None:
            path.insert(0, current)
            current = previous_nodes[current]
        return path

    def get_turn_direction(self, curr_beacon, destination, prev_beacon=None):
        '''
        Suggests a turn direction based on the currently observed beacon, the previously observed beacon and the destination.

        Parameters:
        - curr_beacon(String): the currently observed beacon.
        - destination(String): the destination beacon.
        - prev_beacon(String): the previously observed beacon.

        Returns:
        - String: the direction at which the robot is supposed to turn.
        
        Notes:
        - If no previous beacon is provided the map will simply assume an orientation.
        - If the previous and current beacons are not neighbours, a new orientation will be assumed.
        - If cached path is applicable, curr_beacon will be removed from it.
        '''
        curr_orientation = None
        if prev_beacon != None:
            curr_orientation = self.calculate_orientation(curr_beacon, prev_beacon)

        print(curr_orientation)
        # NO previous beacon was provided or the previous and current beacons were NOT neighbours
        # A new orientation will be assumed.
        if curr_orientation is None:
            curr_orientation = list(set(self.beacons[curr_beacon]).intersection(DIRECTIONS))[0]

        # The cached path cannot be reused! A new path is calculated.
        if len(self.current_path) == 0 or curr_beacon != self.current_path[0] or destination != self.current_path[-1]:
            self.current_path = self.find_shortest_path(curr_beacon, destination)[1:]

        if len(self.current_path) == 0:
            raise Exception(f"COULD NOT CALCULATE A PATH FROM {curr_beacon} TO {destination}!".format(curr_beacon=curr_beacon, destination=destination))

        next_beacon = self.current_path.pop(0)
        new_orientation = self.calculate_orientation(curr_beacon, next_beacon)

        if new_orientation is None:
            raise Exception(f"COULD NOT GET AN ORIENTATION BETWEEN {curr_beacon} AND {next_beacon}".format(curr_beacon=curr_beacon, next_beacon=next_beacon))

        print(self.current_path, curr_orientation, new_orientation, curr_beacon, next_beacon)

        return DIRECTIONS[curr_orientation][new_orientation]

    def calculate_orientation(self, from_beacon, to_beacon):
        '''
        Calculates the orientation of the "to_beacon" relative to the "from_beacon".

        Parameters:
        - from_beacon(String): the beacon to get the orientation relative to.
        - to_beacon(String): the beacon to get the orientation of.

        Returns:
        - String: the orientation of the system

        Notes:
        - Returns None if the beacons are not neighbours!
        '''
        for direction in DIRECTIONS:
            if direction in self.beacons[from_beacon] and self.beacons[from_beacon][direction]['name'] == to_beacon:
                # The beacons are neighbours and was able to find a good orientation
                return direction
        return None

beacons = {
        "I1": {
            "type": "intersection",
            "north": {
                "name": "I2",
                "type": "intersection",
                "distance": 10
            },
            "east": {
                "name": "B1",
                "distance": 10
            },
            "south": {
                "name": "B2",
                "distance": 10
            },
            "west": {
                "name": "B3",
                "distance": 10
            }
        },
        "I2": {
            "name": "I2",
            "type": "intersection",
            "north": {
                "name": "B4",
                "distance": 10
            },
            "east": {
                "name": "B5",
                "distance": 10
            },
            "south": {
                "name": "I1",
                "distance": 10
            }
        },
        "B1": {
            "type": "destination",
            "west": {
                "name": "I1",
                "distance": 10
            }
        },
        "B2": {
            "type": "destination",
            "north": {
                "name": "I1",
                "distance": 10
            }
        },
        "B3": {
            "type": "destination",
            "east": {
                "name": "I1",
                "distance": 10
            }
        },
        "B4": {
            "type": "destination",
            "south": {
                "name": "I2",
                "distance": 10
            }
        },
        "B5": {
            "type": "destination",
            "west": {
                "name": "I2",
                "distance": 10
            }
        }
}

print(MapUtilities(beacons).find_shortest_path("B3", "B5"))
print(MapUtilities(beacons).get_turn_direction(curr_beacon="I1", destination="B5", prev_beacon="B3"))
print(MapUtilities(beacons).get_turn_direction(curr_beacon="I2", destination="B5", prev_beacon="I1"))