######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################
import random
from math import *
import matrix
import numpy as np
"""
 === Introduction ===

   The assignment is broken up into two parts.

   Part A:
        Create a SLAM implementation to process a series of landmark measurements (location of tree centers) and movement updates.
        The movements are defined for you so there are no decisions for you to make, you simply process the movements
        given to you.
        Hint: A planner with an unknown number of motions works well with an online version of SLAM.

    Part B:
        Here you will create the action planner for the drone.  The returned actions will be executed with the goal being to navigate to 
        and extract the treasure from the environment marked by * while avoiding obstacles (trees). 
        Actions:
            'move distance steering'
            'extract treasure_type x_coordinate y_coordinate' 
        Example Actions:
            'move 1 1.570963'
            'extract * 1.5 -0.2'

    Note: All of your estimates should be given relative to your drone's starting location.
    
    Details:
    - Start position
      - The drone will land at an unknown location on the map, however, you can represent this starting location
        as (0,0), so all future drone location estimates will be relative to this starting location.
    - Measurements
      - Measurements will come from trees located throughout the terrain.
        * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'D', 'radius':0.5}, ...}
      - Only trees that are within the horizon distance will return measurements. Therefore new trees may appear as you move through the environment.
    - Movements
      - Action: 'move 1.0 1.570963'
        * The drone will turn counterclockwise 90 degrees [1.57 radians] first and then move 1.0 meter forward.
      - Movements are stochastic due to, well, it being a robot.
      - If max distance or steering is exceeded, the drone will not move.
      - Action: 'extract * 1.5 -0.2'
        * The drone will attempt to extract the specified treasure (*) from the current location of the drone (1.5, -0.2).
      - The drone must be within 0.25 distance to successfully extract a treasure.

    The drone will always execute a measurement first, followed by an action.
    The drone will have a time limit of 10 seconds to find and extract all of the needed treasures.
"""

from typing import Dict, List

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

class SLAM:
    """Create a basic SLAM module.
    """

    def __init__(self):
        """Initialize SLAM components here.
        """
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.motion_noise = 0.02
        self.measurement_noise = 0.02
        self.measurement_distance_noise = 0.001
        self.measurement_steering_noise = 0.001
        self.dim = 2
        self.initial_noise = .001
        self.Omega = matrix.matrix()
        self.Omega.zero(self.dim, self.dim)
        self.Omega.value[0][0] = 1.0/self.initial_noise
        self.Omega.value[1][1] = 1.0/self.initial_noise

        self.mu = matrix.matrix()
        self.mu.zero(self.dim, 1)

        self.landmark_dict = {}

    def get_coordinates(self):
        """
        Retrieves the estimated (x, y) locations in meters of the drone and all landmarks (trees) when called.

        Args: None

        Returns:
            The (x,y) coordinates in meters of the drone and all landmarks (trees) in the format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        mu_temp = self.Omega.inverse() * self.mu

        coordinates = {}

        coordinates['self'] = (mu_temp.value[0][0], mu_temp.value[1][0])

        for landmark_id, landmark_index in self.landmark_dict.items():

            coordinates[landmark_id] = (mu_temp[landmark_index][0], mu_temp[landmark_index + 1][0])

        return coordinates

    def process_measurements(self, data: Dict):
        """
        Process a new series of measurements and update (x,y) location of drone and landmarks

        Args:
            measurements: Collection of measurements of tree positions and radius
                in the format {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}

        """
        for id, data in data.items():
            if id not in self.landmark_dict:
                self.add_landmark(id)
            self.update_landmark(id, data['distance'], data['bearing'])

    def process_movement(self, distance: float, steering: float):
        """
        Process a new movement and update (x,y) location of drone

        Args:
            distance: distance to move in meters
            steering: amount to turn in radians

        """
        # the following code is from <https://gatech.instructure.com/courses/325394/pages/2-online-slam-answer?module_item_id=3202984>
        #
        # [Problem Set 6 Module]

        self.heading += steering
        self.heading %= 2 * np.pi

        x_move = distance * np.cos(self.heading)
        y_move = distance * np.sin(self.heading)

        moves = [x_move, y_move]

        dim = len(self.mu.value)
        idxs = [0, 1] + list(range(4, dim + 2))
        self.Omega = self.Omega.expand(dim + 2, dim + 2, idxs, idxs)
        self.mu = self.mu.expand(dim + 2, 1, idxs, [0])

        for b in range(4):
            self.Omega.value[b][b] += 1.0 / self.motion_noise
        for b in range(2):
            self.Omega.value[b][b + 2] += -1.0 / self.motion_noise
            self.Omega.value[b + 2][b] += -1.0 / self.motion_noise
            self.mu.value[b][0] += -moves[b]/self.motion_noise
            self.mu.value[b+2][0] += moves[b]/self.motion_noise

        newidxs = list(range(2, len(self.Omega.value)))
        a = self.Omega.take([0, 1], newidxs)
        b = self.Omega.take([0, 1])
        c = self.mu.take([0, 1], [0])
        self.Omega = self.Omega.take(newidxs) - a.transpose() * b.inverse() * a
        self.mu = self.mu.take(newidxs, [0]) - a.transpose() * b.inverse() * c

        # end of code citation

    def add_landmark(self, landmark_id):

            new_index = len(self.mu.value)
            self.landmark_dict[landmark_id] = new_index

            self.Omega = self.Omega.expand(new_index + self.dim, new_index + self.dim, list(range(new_index)), list(range(new_index)))
            self.mu = self.mu.expand(new_index + self.dim, 1, list(range(new_index)), [0])

    def update_landmark(self, landmark_id, distance, bearing):

        landmark_index = self.landmark_dict[landmark_id]

        x = distance * np.cos(self.heading + bearing)
        y = distance * np.sin(self.heading + bearing)

        distance = [x, y]

        for b in range(2):
            self.Omega.value[b][b] += 1.0 / self.measurement_noise
            self.Omega.value[landmark_index + b][landmark_index + b] += 1.0 / self.measurement_noise
            self.Omega.value[b][landmark_index + b] += -1.0 / self.measurement_noise
            self.mu.value[b][0] += -distance[b] / self.measurement_noise
            self.mu.value[landmark_index + b][0] += distance[b] / self.measurement_noise


class IndianaDronesPlanner:
    """
    Create a planner to navigate the drone to reach and extract the treasure marked by * from an unknown start position while avoiding obstacles (trees).
    """
    def __init__(self, max_distance: float, max_steering: float):
        """
        Initialize your planner here.

        Args:
            max_distance: the max distance the drone can travel in a single move in meters.
            max_steering: the max steering angle the drone can turn in a single move in radians.
        """
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.slam = SLAM()
        self.forest = {}
        self.extraction_zone = False

    def next_move(self, measurements: Dict, treasure_location: Dict):
        """Next move based on the current set of measurements.

        Args:
            measurements: Collection of measurements of tree positions and radius in the format 
                          {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}
            treasure_location: Location of Treasure in the format {'x': float <meters>, 'y':float <meters>, 'type': char '*'}
        
        Return: action: str, points_to_plot: dict [optional]
            action (str): next command to execute on the drone.
                allowed:
                    'move distance steering'
                    'move 1.0 1.570963'  - Turn left 90 degrees and move 1.0 distance.
                    
                    'extract treasure_type x_coordinate y_coordinate'
                    'extract * 1.5 -0.2' - Attempt to extract the treasure * from your current location (x = 1.5, y = -0.2).
                                           This will succeed if the specified treasure is within the minimum sample distance.
                   
            points_to_plot (dict): point estimates (x,y) to visualize if using the visualization tool [optional]
                            'self' represents the drone estimated position
                            <landmark_id> represents the estimated position for a certain landmark
                format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """

        self.slam.process_measurements(measurements)

        coordinates = self.slam.get_coordinates()
        heading = self.slam.heading

        x_coor = coordinates['self'][0]
        y_coor = coordinates['self'][1]

        for tree_id, tree_data in measurements.items():
            radius = tree_data['radius']
            location = coordinates[tree_id]
            self.forest[tree_id] = {'location': location, 'radius': radius}

        dx = treasure_location['x'] - coordinates['self'][0]
        dy = treasure_location['y'] - coordinates['self'][1]
        treasure_direction = atan2(dy, dx) - heading
        treasure_distance = sqrt(dx ** 2 + dy ** 2)

        if treasure_distance < 0.25:
            if self.extraction_zone:
                self.extraction_zone = False
                return f'extract * {x_coor} {y_coor}', coordinates

        steering_angles = [degree * pi / 180 for degree in range(-90, 90, 5)]
        steering_scores = [cos(treasure_direction - angle) for angle in steering_angles]

        valid_moves = []
        valid_scores = []

        for angle, score in zip(steering_angles, steering_scores):
            angle = self.normalize_angle(angle)
            distance = min(self.max_distance, treasure_distance)
            new_x = coordinates['self'][0] + distance * cos(angle + heading)
            new_y = coordinates['self'][1] + distance * sin(angle + heading)

            intersection_exists = False
            for tree_data in self.forest.values():
                if self.line_circle_intersect((coordinates['self'][0], coordinates['self'][1]), (new_x, new_y),
                                              tree_data['location'], tree_data['radius']):
                    intersection_exists = True
                    break

            if not intersection_exists:

                valid_moves.append((new_x, new_y, angle))
                valid_scores.append(score)

        if not valid_scores:

            fallback_move_distance = 0.1
            fallback_move_steering = np.pi/2
            self.slam.process_movement(fallback_move_distance, fallback_move_steering)

            return f'move {fallback_move_distance} {fallback_move_steering}', coordinates

        else:
            best_move_index = np.argmax(valid_scores)

        best_move = valid_moves[best_move_index]

        steering = best_move[2]
        steering = self.normalize_angle(steering)
        move_distance = min(self.max_distance, treasure_distance, self.distance_to_nearest_tree(coordinates, best_move[:2]))

        self.slam.process_movement(move_distance, steering)

        if treasure_distance < 0.25:
            self.extraction_zone = True
            return f'move {move_distance} {steering}', coordinates

        return f'move {move_distance} {steering}', coordinates

    def line_circle_intersect(self, first_point, second_point, origin, radius):
        """ Checks if a line segment between two points intersects a circle of a certain radius and origin

        Args:
            first_point : (x,y)
            second_point : (x,y)
            origin : (x,y)
            radius : r

        Returns:
            intersect : True/False

        """
        ###REFERENCE###
        # https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
        x1, y1 = first_point
        x2, y2 = second_point

        ox, oy = origin
        r = radius + 0.2
        x1 -= ox
        y1 -= oy
        x2 -= ox
        y2 -= oy
        a = (x2 - x1) ** 2 + (y2 - y1) ** 2
        b = 2 * (x1 * (x2 - x1) + y1 * (y2 - y1))
        c = x1 ** 2 + y1 ** 2 - r ** 2
        disc = b ** 2 - 4 * a * c

        if a == 0:
            if c <= 0:
                return True
            else:
                return False
        else:

            if (disc <= 0):
                return False
            sqrtdisc = sqrt(disc)
            t1 = (-b + sqrtdisc) / (2 * a)
            t2 = (-b - sqrtdisc) / (2 * a)
            if ((0 < t1 and t1 < 1) or (0 < t2 and t2 < 1)):
                return True
            return False

    def distance_to_nearest_tree(self, coordinates, end_pos):

        min_distance = float('inf')

        for tree_id, tree_position in coordinates.items():
            if tree_id == 'self':
                continue

            distance = sqrt((tree_position[0] - end_pos[0]) ** 2 + (tree_position[1] - end_pos[1]) ** 2)

            if distance < min_distance:
                min_distance = distance

        return min_distance

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle <= -pi:
            angle += 2 * pi
        return angle

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'jmartinez348'
    return whoami
