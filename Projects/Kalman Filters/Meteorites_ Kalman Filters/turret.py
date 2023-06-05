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

# Optional: You may use deepcopy to help prevent aliasing
# from copy import deepcopy

# You may use either the numpy library or Sebastian Thrun's matrix library for
# your matrix math in this project; uncomment the import statement below for
# the library you wish to use, and ensure that the library you are not using is
# commented out.

import numpy as np
from matrix import matrix
import math

# If you see different scores locally and on Gradescope this may be an
# indication that you are uploading a different file than the one you are
# executing locally. If this local ID doesn't match the ID on Gradescope then
# you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class Turret(object):
    """The laser used to defend against invading Meteorites."""

    def __init__(self, init_pos, max_angle_change,
                 dt):
        """Initialize the Turret."""
        self.x_pos = init_pos['x']
        self.y_pos = init_pos['y']
        self.max_angle_change = max_angle_change
        self.dt = dt
        self.meteorite_data = {}

    def predict_from_observations(self, meteorite_observations):
        """Observe meteorite locations and predict their positions at time t+1.

        Parameters
        ----------
        self = a reference to the current object, the Turret
        meteorite_observations = a list of noisy observations of
            meteorite locations, taken at time t

        Returns
        -------
        A tuple or list of tuples containing (i, x, y), where i, x, and y are:
        i = the meteorite's ID
        x = the estimated x-coordinate of meteorite i's position for time t+1
        y = the estimated y-coordinate of meteorite i's position for time t+1

        Return format hint:
        For a tuple of tuples, this would look something like
        ((1, 0.4, 0.381), (2, 0.77, 0.457), ...)
        For a list of tuples, this would look something like
        [(1, 0.4, 0.381), (2, 0.77, 0.457), ...]

        Notes
        -----
        Each observation in meteorite_observations is a tuple
        (i, x, y), where i is the unique ID for a meteorite, and x, y are the
        x, y locations (with noise) of the current observation of that
        meteorite at this timestep. Only meteorites that are currently
        'in-bounds' will appear in this list, so be sure to use the meteorite
        ID, and not the position/index within the list to identify specific
        meteorites.
        The list/tuple of tuples you return may change in size as meteorites
        move in and out of bounds.
        """

        predictions = []
        S = 1/3

        F = matrix([[1, 0, self.dt, 0, (S/2)*self.dt**2],
                           [0, 1,0,self.dt,0.5*self.dt**2],
                           [0, 0, 1, 0, (S)*self.dt],
                           [0,0,0,1,self.dt],
                           [0,0,0,0,1]])  # next state function

        H = matrix([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0 ,0]])  # measurement function

        R = matrix([[0.1, 0],
                           [0, 0.1]])  # measurement uncertainty

        I = matrix([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1]])  # identity matrix

        for meteorite in meteorite_observations:
            meteorite_id = meteorite[0]

            if meteorite_id in self.meteorite_data:
                P = self.meteorite_data[meteorite_id]['P']
                x = self.meteorite_data[meteorite_id]['x']

            else:

                initial_xy = [0, 0]
                x = matrix([[initial_xy[0], initial_xy[1], 0, 0, 0]]).transpose()

                P = matrix([[10, 0, 0, 0, 0],
                            [0, 10, 0, 0, 0],
                            [0, 0, 10, 0, 0],
                            [0, 0, 0, 10, 0],
                            [0, 0, 0, 0, 10]])

                distance = math.sqrt((initial_xy[0] - 0)**2 + (initial_xy[0] + 1)**2)
                self.meteorite_data[meteorite_id] = {'x': x, 'P': P, 'distance':distance}

            Z = matrix([[meteorite[1], meteorite[2]]]).transpose() - H * x
            S = H * P * H.transpose() + R
            K = P * H.transpose() * S.inverse()

            x_p = x + (K * Z)
            P_p = (I - K * H) * P

            # Predictions
            x = F * x_p
            P = F * P_p * F.transpose()
            distance = math.sqrt((x[0][0] - 0) ** 2 + (x[1][0] + 1) ** 2)

            self.meteorite_data[meteorite_id]['x'] = x
            self.meteorite_data[meteorite_id]['P'] = P
            self.meteorite_data[meteorite_id]['distance'] = distance

            predictions.append((meteorite[0], x[0][0], x[1][0]))

        return predictions

    def get_laser_action(self, current_aim_rad):
        """Return the laser's action; it can change its aim angle and/or fire.

        Parameters
        ----------
        self = a reference to the current object, the Turret
        current_aim_rad = the laser turret's current aim angle, in radians,
            provided by the simulation.


        Returns
        -------
        Float (desired change in laser aim angle, in radians)
        Bool (True if the laser should fire next timestep, False otherwise)
        Note that the float should be returned first, and the bool second

        Notes
        -----
        The laser can aim in the range [0.0, pi].

        The maximum amount the laser's aim angle can change in a given timestep
        is self.max_angle_change radians. Larger change angles will be
        clamped to self.max_angle_change, but will keep the same sign as the
        returned desired angle change (e.g. an angle change of -3.0 rad would
        be clamped to -self.max_angle_change).

        If the laser is aimed at 0.0 rad, it will point horizontally to the
        right; if it is aimed at pi rad, it will point to the left.

        If bool returned from this function is True, the laser will fire.
        """

        filtered_meteorites = {id: meteorite for id, meteorite in self.meteorite_data.items()
                               if id != -1 and
                               -1 < meteorite['x'][0][0] < 1 and
                               meteorite['x'][1][0] > -1 and
                               meteorite['distance'] > 0.4}

        closest_meteorite_id = min(filtered_meteorites , key=lambda id: filtered_meteorites[id]['distance'])

        x = self.meteorite_data[closest_meteorite_id]['x'][0][0]
        y = self.meteorite_data[closest_meteorite_id]['x'][1][0]+1

        angle = abs(math.atan2(y,x))

        if current_aim_rad < angle:
            angle_change_rad = angle - current_aim_rad
        else:
            angle_change_rad = (current_aim_rad - angle)*-1.0

        if abs(angle_change_rad) > 0.0873:
            fire = False
        else:
            fire = True

        if closest_meteorite_id in self.meteorite_data and fire is True:
            del self.meteorite_data[closest_meteorite_id]

        return angle_change_rad, fire


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'jmartinez348'
    return whoami

