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

from Utilities.matrix import *

# KALMAN FILTER LESSON MODULES
print("KALMAN FILTER LESSON MODULES", end="")

# --------------------------------------------------------------------
# 8. MAXIMIZE GAUSSIAN
print("\n8. MAXIMIZE GAUSSIAN")
# For this problem, you aren't writing any code.
# Instead, please just change the last argument
# in f() to maximize the output.


def f(mu, sigma2, x):
    return 1/sqrt(2.*pi*sigma2) * exp(-.5*(x-mu)**2 / sigma2)


print(f(10, 4, 8))  # TODO: CHANGE/UPDATE CODE HERE (Change the 8. to something else!)

# --------------------------------------------------------------------
# 17. NEW MEAN AND VARIANCE
print("\n17. NEW MEAN AND VARIANCE")
# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.


def update(mean1, var1, mean2, var2):
    new_mean = None  # TODO: CHANGE/UPDATE CODE HERE
    new_var = None
    return [new_mean, new_var]


print(update(10, 8, 13, 2))

# --------------------------------------------------------------------
# 19. PREDICT FUNCTION
print("\n19. PREDICT FUNCTION")
# Write a program that will predict your new mean
# and variance given the mean and variance of your
# prior belief and the mean and variance of your
# motion.


def predict(mean1, var1, mean2, var2):
    new_mean = None  # TODO: CHANGE/UPDATE CODE HERE
    new_var = None
    return [new_mean, new_var]


print(predict(10, 4, 12, 4))

# --------------------------------------------------------------------
# 20. KALMAN FILTER CODE
print("\n20. KALMAN FILTER CODE")
# Write a program that will iteratively update and
# predict based on the location measurements
# and inferred motions shown below.

measurements = [5, 6, 7, 9, 10]
motion = [1, 1, 2, 1, 1]
measurement_sig = 4
motion_sig = 2
mu = 0
sig = 10000  # Changed throughout answer video:  1000, 0.000000001

# Please print out ONLY the final values of the mean
# and the variance in a list [mu, sig].

# TODO: ADD CODE HERE

print([mu, sig])

# --------------------------------------------------------------------
# 27. KALMAN MATRICES
print("\n27. KALMAN MATRICES")
# Write a function 'kalman_filter' that implements a multi-
# dimensional Kalman Filter for the example given


def kalman_filter(x, P):
    for n in range(len(measurements)):
        # TODO: ADD CODE HERE
        # measurement update
        x = x
        # prediction
        P = P

    return x, P


# use the code below to test your filter!

measurements = [1, 2, 3]

x = matrix([[0], [0]])  # initial state (location and velocity)
P = matrix([[1000, 0], [0, 1000]])  # initial uncertainty
u = matrix([[0], [0]])  # external motion
F = matrix([[1, 1], [0, 1]])  # next state function
H = matrix([[1, 0]])  # measurement function
R = matrix([[1]])  # measurement uncertainty
I = matrix([[1, 0], [0, 1]])  # identity matrix

x, P = kalman_filter(x, P)
print('x:')
x.show()
print('P:')
P.show()
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]
