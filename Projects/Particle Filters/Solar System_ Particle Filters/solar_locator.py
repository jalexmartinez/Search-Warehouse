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

# These import statements give you access to library functions which you may
# (or may not?) want to use.
import random
import time
from math import *
from body import *
from solar_system import *
from satellite import *
import numpy as np
from testing_suite_full import *
from statistics import stdev
from copy import deepcopy
def estimate_next_pos(gravimeter_measurement, gravimeter_sense_func, distance, steering, other=None):
    """
    Estimate the next (x,y) position of the satelite.
    This is the function you will have to write for part A.
    :param gravimeter_measurement: float
        A floating point number representing
        the measured magnitude of the gravitation pull of all the planets
        felt at the target satellite at that point in time.
    :param gravimeter_sense_func: Func
        A function that takes in (x,y) and outputs the magnitude of the gravitation pull of all the planets
        felt that (x,y) location at that point in time.
    :param distance: float
        The target satellite's motion distance
    :param steering: float
        The target satellite's motion steering
    :param other: any
        This is initially None, but if you return an OTHER from
        this function call, it will be passed back to you the next time it is
        called, so that you can use it to keep track of important information
        over time. (We suggest you use a dictionary so that you can store as many
        different named values as you want.)
    :return:
        estimate: Tuple[float, float]. The (x,y) estimate of the target satellite at the next timestep
        other: any. Any additional information you'd like to pass between invocations of this function
        optional_points_to_plot: List[Tuple[float, float, float]].
            A list of tuples like (x,y,h) to plot for the visualization
    """

    num_particles = 3000
    N = 100
    if other ==  None:

        x_axis = np.random.uniform(-4, 4, num_particles)
        y_axis = np.random.uniform(-4, 4, num_particles)
        orientations = np.linspace(0,2*pi, num_particles)
        list_particles = []

        grav_list = []
        for x in range(num_particles):
            grav_measured = gravimeter_sense_func(x_axis[x]*AU, y_axis[x]*AU)
            particle = [x_axis[x]*AU, y_axis[x]*AU, orientations[x], grav_measured, 0]
            list_particles.append(particle)
            grav_list.append(grav_measured)
        sigma = 0.1 * (max(grav_list) - min(grav_list))

        for x in range(num_particles):
            list_particles[x][4] = Gaussian(list_particles[x][3], sigma, gravimeter_measurement)

        measurement_buffer = []

        sorted_particles = sorted(list_particles, key=lambda x: x[4], reverse=True)
    else:
        list_particles = other["estimates"]
        measurement_buffer = other["grav_measurements"]
        grav_list = []
        for x in range(num_particles):
            grav_measured = gravimeter_sense_func(list_particles[x][0], list_particles[x][1])
            list_particles[x][3] = grav_measured
            grav_list.append(grav_measured)
            if len(measurement_buffer) >= N:
                measurement_buffer = measurement_buffer[5:]

            measurement_buffer.append(grav_measured)

        sigma = np.std(measurement_buffer)*.6
        if sigma < .000001:
            sigma = .000001

        for x in range(num_particles):
            list_particles[x][4] = Gaussian(list_particles[x][3], sigma, gravimeter_measurement)

        sorted_particles = sorted(list_particles, key=lambda x: x[4], reverse=True)

    # the following code is from <https://gatech.instructure.com/courses/325394/pages/21-resampling-wheel-answer?module_item_id=3202482>
    #
    # [Particle Filters: 21. Resampling Wheel.]

    mw = sorted_particles[0][4] * 0.9
    beta = 0.0
    index = int(random.random() * num_particles)
    resampled_particles = []
    for i in range(num_particles):
        beta += random.random() * 2.0 * mw
        while beta > sorted_particles[index][4]:
            beta -= sorted_particles[index][4]
            index = (index + 1) % num_particles
        resampled_particle = deepcopy(sorted_particles[index])

        noise_x = np.random.normal(0, AU*0.001)
        noise_y = np.random.normal(0, AU*0.001)
        resampled_particle[0] += noise_x
        resampled_particle[1] += noise_y
        resampled_particles.append(resampled_particle)

    # end of code citation

    list_particles = resampled_particles
    sorted_particles_final = sorted(list_particles, key=lambda x: x[4], reverse=True)

    for idx, particle in enumerate(sorted_particles_final):
        radius = sqrt(particle[0]**2 + particle[1]**2)
        beta_angle = distance/radius
        theta = atan2(particle[1],particle[0])
        angle = (theta + beta_angle) % (2 * pi)
        new_x = cos(angle) * radius
        new_y = sin(angle) * radius
        noise_x = np.random.normal(0, 0.01*AU)
        noise_y = np.random.normal(0, 0.01*AU)
        new_x += noise_x
        new_y += noise_y
        particle[0] = new_x
        particle[1] = new_y
        particle[2] = theta

    list_of_tuples = [tuple([particle[0],particle[1], particle[2]+pi/2]) for particle in sorted_particles_final]
    xy_estimate = (sorted_particles_final[0][0], sorted_particles_final[0][1])

    values_dict = {"estimates": sorted_particles_final, "grav_measurements" : measurement_buffer}

    return xy_estimate, values_dict , list_of_tuples


def next_angle(solar_system, percent_illuminated_measurements, percent_illuminated_sense_func,
               distance, steering, other=None):
    """
    Gets the next angle at which to send out an sos message to the home planet,
    the last planet in the solar system.
    This is the function you will have to write for part B.
    The input parameters are exactly the same as for part A.
    :param solar_system: SolarSystem
        A model of the solar system containing the sun and planets as Bodys (contains positions, velocities, and masses)
        Planets are listed in order from closest to furthest from the sun
    :param percent_illuminated_measurements: List[float]
        A list of floating point number from 0 to 100 representing
        the measured percent illumination of each planet in order from closest to furthest to sun
        as seen by the target satellite.
    :param percent_illuminated_sense_func: Func
        A function that takes in (x,y) and outputs the list of percent illuminated measurements of each planet
        as would be seen by satellite at that (x,y) location.
    :param distance: float
        The target satellite's motion distance
    :param steering: float
        The target satellite's motion steering
    :param other: any
        This is initially None, but if you return an OTHER from
        this function call, it will be passed back to you the next time it is
        called, so that you can use it to keep track of important information
        over time. (We suggest you use a dictionary so that you can store as many
        different named values as you want.)
    :return:
        bearing: float. The absolute angle from the satellite to send an sos message between -pi and pi
        xy_estimate: Tuple[float, float]. The (x,y) estimate of the target satellite at the next timestep
        other: any. Any additional information you'd like to pass between invocations of this function
        optional_points_to_plot: List[Tuple[float, float, float]].
            A list of tuples like (x,y,h) to plot for the visualization
    """

    num_particles = 1000
    N = 50
    planets = len(percent_illuminated_measurements)

    if other ==  None:

        x_axis = np.random.uniform(-4, 4, num_particles)
        y_axis = np.random.uniform(-4, 4, num_particles)
        orientations = np.linspace(0,2*pi, num_particles)
        list_particles = []
        measurement_buffers = [[] for _ in range(planets)]

        for x in range(num_particles):
            illumination_measured = percent_illuminated_sense_func(x_axis[x]*AU, y_axis[x]*AU)
            particle = [x_axis[x]*AU, y_axis[x]*AU, orientations[x], illumination_measured, 0]
            list_particles.append(particle)
            for planet in range(planets):
                if len(measurement_buffers[planet]) >= N:
                    measurement_buffers[planet] = measurement_buffers[planet][5:]
                measurement_buffers[planet].append(illumination_measured[planet])

        sigmas = []

        for planet in range(planets):
            sigmas.append(np.std(measurement_buffers[planet]))

        for x in range(num_particles):
            error = 1.0
            for planet in range(planets):
                error *= Gaussian(list_particles[x][3][planet], sigmas[planet], percent_illuminated_measurements[planet])
            list_particles[x][4] = error

        sorted_particles = sorted(list_particles, key=lambda x: x[4], reverse=True)


    else:
        list_particles = other["estimates"]
        measurement_buffers = other["illumination_measurements"]

        for x in range(num_particles):
            illumination_measured = percent_illuminated_sense_func(list_particles[x][0], list_particles[x][1])
            list_particles[x][3] = illumination_measured
            for y in range(planets):
                if len(measurement_buffers[y]) >= N:
                    measurement_buffers[y] = measurement_buffers[y][5:]
                measurement_buffers[y].append(illumination_measured)

        sigmas = []

        for planet in range(planets):
            if np.std(measurement_buffers[planet]) > .000001:
                sigmas.append(np.std(measurement_buffers[planet]))
            else:
                sigmas.append(.000001)

        for x in range(num_particles):
            error = 1.0
            for planet in range(planets):
                error *= Gaussian(list_particles[x][3][planet], sigmas[planet], percent_illuminated_measurements[planet])
            list_particles[x][4] = error

        sorted_particles = sorted(list_particles, key=lambda x: x[4], reverse=True)

    # the following code is from <https://gatech.instructure.com/courses/325394/pages/21-resampling-wheel-answer?module_item_id=3202482>
    #
    # [Particle Filters: 21. Resampling Wheel.]

    mw = sorted_particles[0][4] * 0.9
    beta = 0.0
    index = int(random.random() * num_particles)
    resampled_particles = []
    for i in range(num_particles):
        beta += random.random() * 2.0 * mw
        while beta > sorted_particles[index][4]:
            beta -= sorted_particles[index][4]
            index = (index + 1) % num_particles
        resampled_particle = deepcopy(sorted_particles[index])

        noise_x = np.random.normal(0, AU*0.001)
        noise_y = np.random.normal(0, AU*0.001)
        resampled_particle[0] += noise_x
        resampled_particle[1] += noise_y
        resampled_particles.append(resampled_particle)

    # end of code citation

    list_particles = resampled_particles
    sorted_particles_final = sorted(list_particles, key=lambda x: x[4], reverse=True)

    for idx, particle in enumerate(sorted_particles_final):
        radius = sqrt(particle[0]**2 + particle[1]**2)
        beta_angle = distance/radius
        theta = atan2(particle[1],particle[0])
        angle = (theta + beta_angle) % (2 * pi)
        new_x = cos(angle) * radius
        new_y = sin(angle) * radius
        noise_x = np.random.normal(0, 0.01*AU)
        noise_y = np.random.normal(0, 0.01*AU)
        new_x += noise_x
        new_y += noise_y
        particle[0] = new_x
        particle[1] = new_y
        particle[2] = theta

    list_of_tuples = [tuple([particle[0],particle[1], particle[2]+pi/2]) for particle in sorted_particles_final]
    xy_estimate = (sorted_particles_final[0][0], sorted_particles_final[0][1])

    home_planet_coordinates = solar_system.planets[-1].r

    home_planet_coordinates_x = random.gauss(home_planet_coordinates[0], AU*.03)
    home_planet_coordinates_y = random.gauss(home_planet_coordinates[1], AU*.03)


    angle_to_planet = atan2(home_planet_coordinates_y - xy_estimate[1], home_planet_coordinates_x - xy_estimate[0])

    values_dict = {"estimates": sorted_particles_final, "illumination_measurements" : measurement_buffers}


    return angle_to_planet, xy_estimate, values_dict, list_of_tuples


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'jmartinez348'
    return whoami

def Gaussian(mu, sigma, x):

    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

def measurement_prob(sigma, measurement):


    prob = 1.0
    prob *= Gaussian(dist, sigma, measurement)

    return prob



