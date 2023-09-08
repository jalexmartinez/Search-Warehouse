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

from Utilities.robot_pid import Robot
from copy import deepcopy
import numpy as np
from math import *
import matplotlib.pyplot as plt

# PID LESSON MODULES
print("PID LESSON MODULES", end="")

# --------------------------------------------------------------------
# 6. PATH SMOOTHING
print("\n6. PATH SMOOTHING")
# Define a function smooth that takes a path as its input
# (with optional parameters for weight_data, weight_smooth,
# and tolerance) and returns a smooth path. The first and
# last points should remain unchanged.
#
# Smoothing should be implemented by iteratively updating
# each entry in newpath until some desired level of accuracy
# is reached. The update should be done according to the
# gradient descent equations given in the instructor's note
# below (the equations given in the video are not quite
# correct).

# thank you to EnTerr for posting this on our discussion forum
def printpaths(path, newpath):
    for old, new in zip(path, newpath):
        print('[' + ', '.join('%.3f' % x for x in old) + \
              '] -> [' + ', '.join('%.3f' % x for x in new) + ']')


# Don't modify path inside your function.
path = [[0, 0],
        [0, 1],
        [0, 2],
        [1, 2],
        [2, 2],
        [3, 2],
        [4, 2],
        [4, 3],
        [4, 4]]


def smooth(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
    # Make a deep copy of path into newpath
    newpath = deepcopy(path)
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)-1):
            for j in range(len(path[0])):
                aux = newpath[i][j]
                newpath[i][j] += weight_data * (path[i][j] - newpath[i][j]) + weight_smooth * (newpath[i-1][j] \
                + newpath[i+1][j] - 2.0 * newpath[i][j])
                change += abs(aux - newpath[i][j])

    return newpath


printpaths(path, smooth(path))

# --------------------------------------------------------------------
# 10. IMPLEMENT P CONTROLLER
print("\n10. IMPLEMENT P CONTROLLER")
# Implement a P controller by running 100 iterations
# of robot motion. The desired trajectory for the
# robot is the x-axis. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau * crosstrack_error

'''
Errata Note:  In this solution Dr. Thrun is using the rear axle as the reference point 
to determine the distance of the robot from the target (x-axis).  Instead, we should be 
using the front axle as the reference point.  As such, the crosstrack_error (cte) should 
be calculated as below:
y_distance_to_front_axle = np.sin(robot.orientation) * robot.length
cte = robot.y + y_distance_to_front_axle
'''

# run - does a single control run
robot = Robot()
robot.set(0.0, 1.0, 0.0)


def run(robot, tau, n=100, speed=1.0):

    x_trajectory = []
    y_trajectory = []

    for i in range(n):
        y_distance_to_front_axle = np.sin(robot.orientation) * robot.length
        crosstrack_error = robot.y + y_distance_to_front_axle
        steer = -tau * crosstrack_error
        robot.move(steer, speed)
        print(robot, steer)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)


    return x_trajectory, y_trajectory


x_trajectory, y_trajectory = run(robot, 0.1)
n = len(x_trajectory)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
ax1.legend()
plt.show()

# --------------------------------------------------------------------
# 13. IMPLEMENT PD CONTROLLER
print("\n13. IMPLEMENT PD CONTROLLER")
# Implement a PD controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau_p and tau_d so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE
# where differential crosstrack error (diff_CTE)
# is given by CTE(t) - CTE(t-1)

robot = Robot()
robot.set(0.0, 1.0, 0.0)


def run(robot, tau_p, tau_d, n=1000, speed=1.0):
    x_trajectory = []
    y_trajectory = []

    for i in range(n):
        y_distance_to_front_axle = np.sin(robot.orientation) * robot.length
        crosstrack_error = robot.y + y_distance_to_front_axle
        if i == 0:
            crosstrack_error_old = crosstrack_error
        steer = -tau_p * crosstrack_error - tau_d*(crosstrack_error-crosstrack_error_old)
        robot.move(steer, speed)
        print(robot, steer)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        crosstrack_error_old = crosstrack_error

    return x_trajectory, y_trajectory


x_trajectory, y_trajectory = run(robot, 0.01, 1)
n = len(x_trajectory)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
ax1.legend()
plt.show()

# --------------------------------------------------------------------
# 17. PID IMPLEMENTATION
print("\n17. PID IMPLEMENTATION")
# Implement a P controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
#
# where the integrated crosstrack error (int_CTE) is
# the sum of all the previous crosstrack errors.
# This term works to cancel out steering drift.

robot = Robot()
robot.set(0.0, 1.0, 0.0)
robot.set_steering_drift(10.0 / 180.0 * pi)


def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []

    crosstrack_error_total = 0.0
    for i in range(n):
        y_distance_to_front_axle = np.sin(robot.orientation) * robot.length
        crosstrack_error = robot.y + y_distance_to_front_axle
        crosstrack_error_total += crosstrack_error
        if i == 0:
            crosstrack_error_old = crosstrack_error
        steer = -tau_p * crosstrack_error - tau_d*(crosstrack_error-crosstrack_error_old) - tau_i * crosstrack_error_total
        robot.move(steer, speed)
        print(robot, steer)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        crosstrack_error_old = crosstrack_error
    return x_trajectory, y_trajectory


x_trajectory, y_trajectory = run(robot, 0.2, 3.0, 0.004)
n = len(x_trajectory)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
ax1.legend()
plt.show()

# --------------------------------------------------------------------
# 20. PARAMETER OPTIMIZATION
print("\n20. PARAMETER OPTIMIZATION")
# Implement twiddle as shown in the previous two videos.
# Your accumulated error should be very small!
#
# You don't have to use the exact values as shown in the video
# play around with different values! This quiz isn't graded just see
# how low of an error you can get.
#
# Try to get your error below 1.0e-10 with as few iterations
# as possible (too many iterations will cause a timeout).


def make_robot():
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(0.0, 1.0, 0.0)
    robot.set_steering_drift(10.0 / 180.0 * np.pi)
    return robot


# NOTE: We use params instead of tau_p, tau_d, tau_i
def run(robot, params, n=200, speed=1.0, printflag = False):
    x_trajectory = []
    y_trajectory = []
    tau_p = params[0]
    tau_d = params[1]
    tau_i = params[2]
    err = 0
    crosstrack_error_total = 0.0
    for i in range(2*n):
        y_distance_to_front_axle = np.sin(robot.orientation) * robot.length
        crosstrack_error = robot.y + y_distance_to_front_axle
        crosstrack_error_total += crosstrack_error
        if i == 0:
            crosstrack_error_old = crosstrack_error
        steer = -tau_p * crosstrack_error - tau_d*(crosstrack_error-crosstrack_error_old) - tau_i * crosstrack_error_total
        robot.move(steer, speed)
        print(robot, steer)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        crosstrack_error_old = crosstrack_error
        if i >= n:
            err += crosstrack_error  ** 2
        if printflag:
            print(robot, steer/pi *180.0)

    return x_trajectory, y_trajectory, err / n


# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.0001):  # tolerance changed towards end of solution video to 0.01
    # Don't forget to call `make_robot` before every call of `run`!
    params = [0.0, 0.0, 0.0]
    dparams = [0.01, 0.01, 0.01]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, params)
    n = 0

    while sum(dparams) > tol:
        for i in range(len(params)):
            params[i] += dparams[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, params)
            if err < best_err:
                best_err = err
                dparams[i] *= 1.1
            else:
                params[i] -= 2.0 * dparams[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot,params)
                if err < best_err:
                    best_err = err
                    dparams[i] *= 1.1
                else:
                    params[i] += dparams[i]
                    dparams[i] *= 0.9

        n += 1

    return params, best_err


params, err = twiddle()
print("Final parameters = {}".format(params))
print("Final twiddle error = {}".format(err))
robot = make_robot()
x_trajectory, y_trajectory, err = run(robot, params)
n = len(x_trajectory)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
ax1.legend()
plt.show()
