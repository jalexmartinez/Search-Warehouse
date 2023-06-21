import copy
import math
import random
import time
import unittest
import multiprocessing as mproc
import queue
import traceback

from solar_system import SolarSystem, AU, G

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

########################################################################
# For debugging set the time limit to a big number (like 600 or more)
# Note, if you turn on Verbose Logging
# or Plotting of Particles, you will want to increase this
# number from the 15 used in grading to a much higher value.
# If you have a fast computer, you may want to reduce this
# number to match your computer's speed to that of the
# VM used by GradeScope
########################################################################
TIME_LIMIT = 15  # seconds

########################################################################
# Additional flags for debug output and visualization
########################################################################
VERBOSE = False  # False for grading
PLOT_PARTICLES = False # False for grading  (Set to True for Visualization!)
PAUSE_FIRST = False  # Pause button for visualization enabled at first time step
PAUSE_DURATION = 3  # Number of seconds to pause for (increase TIME_LIMIT accordingly)

########################################################################
# Toggles for different parts of the assignment
########################################################################
PART_A = True  # Enable/disable Part A (Estimation) - True for grading
PART_B = True  # Enable/disable Part B (Steering) - True for grading

########################################################################
# If your debugger does not handle multiprocess debugging very easily
# then when debugging set the following flag true.
########################################################################
DEBUGGING_SINGLE_PROCESS = True

WINDOW_SIZE = 500  # Size of the window in "units"

# Note for Mac OS High Sierra users having problems with
# "an error occurred while attempting to obtain endpoint for listener" errors:
# Running with the following environment variable fixed this issue for one student.
# OBJC_DISABLE_INITIALIZE_FORK_SAFETY=YES
# Looks like a problem specific to Mac OS High Sierra and internal threading libraries.

PI = math.pi
CREDIT_PER_PASS = 7  # points per test case pass.

# 10 test cases, ran in both parts A & B for 20 total.
# Max score if you get all test cases is potentially 140, but capped at 101 
# This means you do not need to get all test cases for a full score.

# The real test cases will be generated using generate_params_planet.py
# You are welcome to make your own test cases
# and share them on Ed Discussions to expose issues that these test cases may miss.

MASS_SUN = 1.98847e30    # NOT FOR STUDENT USE
MASS_EARTH = 5.97219e24  # NOT FOR STUDENT USE

TRAIL_LENGTH = 25        # NOT FOR STUDENT USE

GLOBAL_PARAMETERS = [None,
                     # Note that sun_mass is multiplied by MASS_SUN,
                     # distances are multiplied by AU
                     # and planet/target masses are multiplied by MASS_EARTH

                     # Cases 1-2 have no noise.
                     # Cases 2-5 have a single planet.
                     # Cases 1, 5-10 have 2-5 planets.
                     # Cases 8-10 have elliptical planet orbits.

                     # Case 1 has no noise to make things easy for you!
                     {'test_case': 1,
                      'sun_mass': 43.842061607797625,
                      'planets': [((-0.9122000568392201, 0.307965701950435), 743.109985022544),
                                  ((1.0405026545494007, 1.565422786259), 198.63961059006155),
                                  ((-0.03098529348182052, -2.8597720510726266), 795.9959700442942)],
                      'elliptical_orbit_factor': 1,
                      'target': ((0.6560702910754909, 0.6560702910754909), 0.012056073017406379),
                      # 'target': ((150000000000/AU, 0), 0.012056073017406379),
                      'map_seed': 3193,
                      'g_measurement_noise': 0,
                      'pi_measurement_noise': 0,
                      'max_steps': 300
                      },
                     # Case 2 also has no noise
                     {'test_case': 2,
                      'sun_mass': 40.32651123452419,
                      'planets': [((0.748307466022684, -0.5501621585566954), 697.5659122010433)],
                      'elliptical_orbit_factor': 1,
                      'target': ((1.7448493082830823, -0.33922659527285975), 0.019530496057453304),
                      'map_seed': 2832,
                      'g_measurement_noise': 0,
                      'pi_measurement_noise': 0,
                      'max_steps': 300
                      },
                     {'test_case': 3,
                      'sun_mass': 45.9390279105321,
                      'planets': [((-0.9804211803264786, -0.1715964281996233), 419.5934553172455)],
                      'elliptical_orbit_factor': 1,
                      'target': ((-2.225980367790211, -0.8399415743473132), 0.020020151506820256),
                      'map_seed': 1818,
                      'g_measurement_noise': 3.940227066209606e-08,
                      'pi_measurement_noise': 0.8962691586363383,
                      'max_steps': 300
                      },
                     {'test_case': 4,
                      'sun_mass': 46.68269826828132,
                      'planets': [((0.02695849446593085, 0.9637141458293313), 381.5299176351419)],
                      'elliptical_orbit_factor': 1,
                      'target': ((1.9126536132602072, 1.9126536132602072), 0.042374002771617604),
                      'map_seed': 3436,
                      'g_measurement_noise': 4.099203572901015e-08,
                      'pi_measurement_noise': 0.9934930265547395,
                      'max_steps': 300
                      },
                     {'test_case': 5,
                      'sun_mass': 49.36514115433552,
                      'planets': [((0.06879472039655281, -2.9089969870426433), 173.0925410323392),],
                      'elliptical_orbit_factor': 1,
                      'target': ((-0.36547756780945817, -0.36547756780945817), 0.0424805207809843),
                      'map_seed': 2672,
                      'g_measurement_noise': 9.181367944633899e-08,
                      'pi_measurement_noise': 0.1260063677334369,
                      'max_steps': 300
                      },
                     {'test_case': 6,
                      'sun_mass': 55.32631131722907,
                      'planets': [
                          ((0.4434836752141118, -0.8162953067647695), 429.28721320049937),
                          ((3.7903161415513944, -0.1991159080741424), 492.1434881251449)
                      ],
                      'elliptical_orbit_factor': 1,
                      'target': ((1.2510568595262288, -1.3762042660614746), 0.047297324822015985),
                      'map_seed': 544,
                      'g_measurement_noise': 1.553888521128858e-08,
                      'pi_measurement_noise': 0.87325995585796,
                      'max_steps': 300
                      },
                     {'test_case': 7,
                      'sun_mass': 53.303044101503424,
                      'planets': [((0.9458667144043527, 0.05816442524818104), 327.84830063909425),
                                  ((-1.1402374326096836, -1.4665094237415308), 7.044467038743739),
                                  ((-0.21658966138766014, 2.7730007047186285), 135.8947328659042),
                                  ((-3.6964927615549446, 0.005075847628367366), 387.3032558130718)],
                      'elliptical_orbit_factor': 1,
                      'target': ((1.7142346679058131, 1.1905174737961082), 0.024192383469398712),
                      'map_seed': 4396,
                      'g_measurement_noise': 8.465291054793808e-08,
                      'pi_measurement_noise': 0.9510991975586898,
                      'max_steps': 300
                      },
                     {'test_case': 8,
                      'sun_mass': 51.42491139397881,
                      'planets': [((-0.8822853436169761, 0.35956874874350164), 202.629302718903),
                                  ((1.6041334569510557, 1.0467146998751635), 475.0665454825634),
                                  ((2.500103605684226, -1.462887693110299), 204.90588999278523),
                                  ((2.8683176141710107, -2.562869682480658), 258.229807739354),
                                  ((3.1599286499947588, 3.594378979419109), 976.8801103238544)],
                      'elliptical_orbit_factor': 0.7473696829601248,
                      'target': ((0.30414513207533167, 0.8538929820336406), 0.015488566252828347),
                      'map_seed': 3454,
                      'g_measurement_noise': 3.246153585769787e-08,
                      'pi_measurement_noise': 0.8248077335107799,
                      'max_steps': 300
                      },
                     {'test_case': 9,
                      'sun_mass': 44.65547999808227,
                      'planets': [((0.6348717055275535, -0.6923360388055648), 593.3821544996055),
                                  ((1.4979452945200982, -1.169299103527468), 360.2409619519721),
                                  ((1.15963923726884, 2.5827195690908415), 367.6590824762749),
                                  ((3.747648062849885, 0.1346082262665511), 346.3476965183801),
                                  ((4.4009787368270175, 1.568320682246742), 708.1145889638916)],
                      'elliptical_orbit_factor': 0.5131219019570615,
                      'target': ((-1.678502067820535, -1.678502067820535), 0.03332174377942925),
                      'map_seed': 4265,
                      'g_measurement_noise': 7.965728904474786e-08,
                      'pi_measurement_noise': 0.5769410155427117,
                      'max_steps': 300
                      },
                     {'test_case': 10,
                      'sun_mass': 43.91101670058105,
                      'planets': [((-0.6159704854212522, 0.7572261480543556), 745.260269507586),
                                  ((-1.773657035462997, -0.7715133207842724), 294.17182205477417),
                                  ((-2.473479942931434, -1.4355488298363925), 68.31428555435444)],
                      'elliptical_orbit_factor': 0.6005628722695515,
                      'target': ((2.7329789298341423, 2.7329789298341423), 0.04180544561258218),
                      'map_seed': 1859,
                      'g_measurement_noise': 4.359226973135861e-08,
                      'pi_measurement_noise': 0.1648536351317786,
                      'max_steps': 300
                      },
]


# Try importing the student code here:

try:
    import solar_locator
    planet1Exc = None
    stack_trace = None
except Exception as e:
    planet1Exc = e
    stack_trace = traceback.format_exc()


class PlanetSimulator(object):
    """Run student submission code.

    Attributes:
        satellite_steps(Queue): synchronized queue to store planet steps.
        satellite_found(Queue): synchronized queue to store if planet located.
        satellite_error(Queue): synchronized queue to store exception messages.
    """

    def __init__(self):

        if DEBUGGING_SINGLE_PROCESS:

            self.satellite_steps = queue.Queue(1)
            self.satellite_found = queue.Queue(1)
            self.satellite_error = queue.Queue(1)

        else:

            self.satellite_steps = mproc.Queue(1)
            self.satellite_found = mproc.Queue(1)
            self.satellite_error = mproc.Queue(1)

    def _reset(self):
        """Reset submission results.
        """
        while not self.satellite_steps.empty():
            self.satellite_steps.get()

        while not self.satellite_found.empty():
            self.satellite_found.get()

        while not self.satellite_error.empty():
            self.satellite_found.get()

    @staticmethod
    def distance(p, q):
        """Calculate the distance between two points.

        Args:
            p(tuple): point 1.
            q(tuple): point 2.

        Returns:
            distance between points.
        """
        x1, y1 = p[0], p[1]
        x2, y2 = q

        dx = x2 - x1
        dy = y2 - y1

        return math.sqrt(dx ** 2 + dy ** 2)

    def simulate_without_sos(self, estimate_next_pos, params):
        """Run simulation only to locate planet.

        Args:
            estimate_next_pos(func): Student submission function to estimate next planet position.
            params(dict): Test parameters.

        Raises:
            Exception if error running submission.
        """
        self._reset()

        # make the test somewhat repeatable by seeding the RNG.
        random.seed(params['map_seed'])

        # build world
        planets_r_and_mass = []
        for planet_params in params['planets']:
            planet_r = tuple(v * AU for v in planet_params[0])
            planet_mass = planet_params[1] * MASS_EARTH
            planets_r_and_mass.append((planet_r, planet_mass))

        solar_system = SolarSystem(mass_sun=MASS_SUN * params['sun_mass'],
                                   planets_r_and_mass=planets_r_and_mass,
                                   elliptical_orbit_factor=params['elliptical_orbit_factor'])

        target_r = tuple(v * AU for v in params['target'][0])
        target_mass = params['target'][1] * MASS_EARTH
        target = SolarSystem.init_body_in_orbit_at_x_and_y(
            mass_sun=solar_system.sun.mass,
            r=target_r,
            mass_body=target_mass)

        tolerance = .01 * AU
        other_info = None
        steps = 0

        # Set up the particle plotter if requested
        if PLOT_PARTICLES:
            import turtle  # Only import if plotting is on.
            turtle.setup(width=WINDOW_SIZE, height=WINDOW_SIZE)
            turtle.setworldcoordinates(-5, -5, 5, 5)

            # paint bg black
            turtle.clearscreen()
            turtle.colormode(255)
            turtle.bgcolor("black")
            turtle.delay(0)
            turtle.hideturtle()
            turtle.penup()

            # set starting point for satellite trail
            turtle.setposition(target.r[0] / AU, target.r[1] / AU)
            turtle.pencolor("red")
            turtle.pendown()
            turtle.ht()

            # set turtle for removing of trail for satellite
            satellite_trail = [(target.r[0] / AU, target.r[1] / AU)]
            turtle_trail_removal = turtle.Turtle()
            turtle_trail_removal.pencolor("black")
            turtle_trail_removal.pendown()
            turtle_trail_removal.ht()
            # declare turtles
            sun_turtle = None
            planet_turtle_list = None
            target_turtle = None
            estimate_turtle = None
            turtle_list = []
            time_turtle = turtle.Turtle(visible=False)
            time_turtle.setposition(3, -5)
            tc_turtle = turtle.Turtle(visible=False)
            tc_turtle.setposition(-5, -5)
            tc_turtle.color("white")
            tc_turtle.write(f'Test Case: {params["test_case"]}')

            def pause(x, y):
                time.sleep(PAUSE_DURATION)
            button = turtle.Turtle()
            button.shapesize(0.5, 0.5, 0.5)
            button.shape('square')
            button.color('white')
            button.fillcolor('orange')
            button.penup()
            button.goto(3.5, 4.9)
            button.write("Pause:", align='left')
            button.setx(4.35)
            button.sety(5.05)
            button.onclick(pause)
            button.showturtle()

        try:
            while steps < params['max_steps']:

                def compute_gravity_magnitude_at_x_y(planets, x, y):
                    """
                    Computes the magnitude of the sum of gravitational acceleration vectors
                    from the planets at the (x,y) position.
                    """

                    r = [x, y]
                    gravity = [0., 0.]
                    for body in planets:
                        # acceleration = G * M / r^2 = G * M / |r|^2 * -r / |r|
                        direction = [body.r[i] - r[i] for i in range(2)]
                        # catch for divide by zero by shifting it 1 meter
                        if direction[0] == 0 and direction[1] == 0:
                            direction[0] = 1
                        c = G * body.mass / (direction[0] ** 2 + direction[1] ** 2) ** (3. / 2)
                        gravity_by_body = [direction[0] * c, direction[1] * c]
                        gravity = [gravity[i] + gravity_by_body[i] for i in range(2)]

                    magnitude = math.sqrt(gravity[0] ** 2 + gravity[1] ** 2)
                    return magnitude

                def gravimeter_sense_func(x: float, y: float):
                    return compute_gravity_magnitude_at_x_y(solar_system.planets, x, y)

                def sense(x: float, y: float, noise: float):
                    return random.gauss(gravimeter_sense_func(x, y), noise)

                # Invoke student function to get student result
                gravimeter_measurement = sense(target.r[0], target.r[1], params['g_measurement_noise'])
                target2 = solar_system.move_body(copy.deepcopy(target))
                radius = math.sqrt(target.r[0]**2 + target.r[1]**2)
                beta_turning_angle = (math.atan2(target2.v[1], target2.v[0]) - math.atan2(target.v[1], target.v[0]) + 2*math.pi) % (2 * math.pi)
                distance = beta_turning_angle * radius
                satellite_length = 10.2
                steering = math.atan2(satellite_length, radius)

                result = estimate_next_pos(
                   gravimeter_measurement=gravimeter_measurement,
                   distance=distance,
                   steering=steering,
                   gravimeter_sense_func=gravimeter_sense_func,
                   other=other_info
                )
                if len(result) == 3:
                    xy_estimate, other_info, optional_points_to_plot = result
                    if not isinstance(optional_points_to_plot, list):
                        raise TypeError(f"Expected returned optional_points_to_plot to be a list "
                                        f"but it was actually a {type(optional_points_to_plot)}")
                    if len(optional_points_to_plot) > 0:
                        if not isinstance(optional_points_to_plot[0], tuple):
                            raise TypeError(f"Expected the element in optional_points_to_plot to be a tuple "
                                            f"but it was actually a {type(optional_points_to_plot[0])}")
                        if len(optional_points_to_plot[0]) < 2 or len(optional_points_to_plot[0]) > 3:
                            raise TypeError(f"Expected the element in optional_points_to_plot to have 2-3 elements, "
                                            f"such as (x,y) or (x,y,h), "
                                            f"but it had {len(optional_points_to_plot[0])} elements: "
                                            f"{optional_points_to_plot[0]}")
                elif len(result) == 2:
                    xy_estimate, other_info = result
                    optional_points_to_plot = None
                else:
                    msg = "estimate_next_pos did not return correct number of return values!"
                    print(msg)
                    raise TypeError(msg)

                if not isinstance(xy_estimate, tuple):
                    raise TypeError(f"Expected the returned xy_estimate to be a tuple "
                                    f"but it was actually a {type(xy_estimate)}")
                if len(xy_estimate) != 2:
                    raise TypeError(f"Expected the returned xy_estimate to have 2 elements, such as (x,y), "
                                    f"but it had {len(optional_points_to_plot[0])} elements: {xy_estimate}")

                # Calculate the actual result position of the target next timestep.
                target = solar_system.move_body(target)
                target_pos = (target.r[0], target.r[1])

                # Rotate the solar system for the next timestep
                solar_system.move_planets()

                if PLOT_PARTICLES:

                    s = turtle.getscreen()
                    s.tracer(0, 1)

                    # plot particles
                    if optional_points_to_plot:
                        # Add turtles if needed.
                        while len(optional_points_to_plot) > len(turtle_list):
                            new_turtle = turtle.Turtle()
                            new_turtle.penup()
                            turtle_list.append(new_turtle)

                        # remove turtles if needed.
                        while len(optional_points_to_plot) < len(turtle_list):
                            turtle_list[-1].hideturtle()
                            turtle_list = turtle_list[0:-1]

                        # paint particles
                        for i in range(len(optional_points_to_plot)):
                            t = turtle_list[i]
                            p = optional_points_to_plot[i]
                            # Optionally plot heading if provided by student
                            if len(p) > 2:
                                t.shape("triangle")
                                t.shapesize(0.2, 0.4)
                                t.settiltangle(p[2] * 180 / math.pi)
                            else:
                                t.shape("circle")
                                t.shapesize(0.1, 0.1)
                            t.fillcolor("white")
                            t.color("white")
                            t.setposition(p[0] / AU, p[1] / AU)

                    # Draw the target satellite.
                    if target_turtle is not None:
                        target_turtle.hideturtle()
                    if target_turtle is None:
                        target_turtle = turtle.Turtle()
                        target_turtle.shape("circle")
                        target_turtle.shapesize(.25, .25)
                        target_turtle.pencolor("red")
                        target_turtle.fillcolor("red")
                        target_turtle.penup()
                    target_turtle.setposition(target_pos[0] / AU, target_pos[1] / AU)
                    target_turtle.showturtle()
                    satellite_trail.append((target_pos[0] / AU, target_pos[1] / AU))  #Add coordinates to the trail list

                    # move planet target turtle and trail
                    turtle.setposition(target.r[0] / AU, target.r[1] / AU)

                    #Remove trail beyond requested length
                    if len(satellite_trail) >= TRAIL_LENGTH:
                        xt_yt = satellite_trail[len(satellite_trail) - TRAIL_LENGTH]
                        turtle_trail_removal.setposition(xt_yt[0], xt_yt[1])

                    # Draw the student estimate of the satellite
                    if estimate_turtle is not None:
                        estimate_turtle.hideturtle()
                    if estimate_turtle is None:
                        estimate_turtle = turtle.Turtle()
                        estimate_turtle.shape("square")
                        estimate_turtle.shapesize(.2, .2)
                        estimate_turtle.fill = False
                        estimate_turtle.color("cyan")
                        estimate_turtle.penup()
                    estimate_turtle.setposition(xy_estimate[0] / AU, xy_estimate[1] / AU)
                    estimate_turtle.showturtle()

                    # Draw the sun
                    if sun_turtle is not None:
                        sun_turtle.hideturtle()
                    if sun_turtle is None:
                        sun_turtle = turtle.Turtle()
                        sun_turtle.shape("circle")
                        sun_turtle.shapesize(.75, .75)
                        sun_turtle.fill = False
                        sun_turtle.color("yellow")
                        sun_turtle.penup()
                    sun_turtle.setposition(0, 0)
                    sun_turtle.showturtle()

                    # Draw the planets
                    if planet_turtle_list is None:
                        #create trial coordinate instance for each planet
                        planet_trail = [[] for i in range(solar_system.get_num_planets())]

                        planet_turtle_list = [turtle.Turtle() for _ in range(solar_system.get_num_planets())]

                        for i, planet_turtle in enumerate(planet_turtle_list or []):
                            size = .1 + solar_system.planets[i].mass / MASS_EARTH / 1000 * .4
                            planet_turtle.shape("circle")
                            planet_turtle.shapesize(size, size)
                            planet_turtle.pencolor("lime")
                            planet_turtle.fillcolor("lime")
                            planet_turtle.penup()
                            x = solar_system.planets[i].r[0] / AU
                            y = solar_system.planets[i].r[1] / AU
                            planet_turtle.setposition(x, y)
                            planet_turtle.pendown()
                            planet_trail[i].append((x, y))  # Add initial coordinates to the trail list
                        #Create trail turtle list and set base turtle settings
                        planet_trail_list = [turtle.Turtle() for _ in range(solar_system.get_num_planets())]
                        for ti, planet_trail_turtle in enumerate(planet_trail_list or []):
                            planet_trail_turtle.pencolor("black")
                            planet_trail_turtle.pendown()
                            planet_trail_turtle.hideturtle()
                    else:
                        for i, planet_turtle in enumerate(planet_turtle_list or []):
                            x = solar_system.planets[i].r[0] / AU
                            y = solar_system.planets[i].r[1] / AU
                            planet_turtle.setposition(x, y)
                            planet_turtle.showturtle()
                            planet_trail[i].append((x, y))  # Add coordinates to the trail list
                        #Remove trail beyond requested length
                        if len(planet_trail[0]) >= TRAIL_LENGTH:
                            for ti, planet_trail_turtle in enumerate(planet_trail_list or []):
                                xt_yt = planet_trail[ti][len(planet_trail[0]) - TRAIL_LENGTH]
                                planet_trail_turtle.setposition(xt_yt[0], xt_yt[1])

                    # Draw steps
                    time_turtle.clear()
                    time_turtle.color("white")
                    time_turtle.write(f'Time Step: {steps}')

                    s.update()

                separation = self.distance(xy_estimate, target_pos)
                if separation < tolerance:
                    self.satellite_found.put(True)
                    self.satellite_steps.put(steps)
                    return

                if PAUSE_FIRST and steps == 0:
                    time.sleep(PAUSE_DURATION)
                steps += 1

                if VERBOSE is True:
                    print(
                        "\nStep: {} Actual ({})  Predicted: ({})\n  Difference = {}\n  Gravity Magnitude={}".format(
                            steps, target_pos, xy_estimate, separation, gravimeter_measurement))
                    if optional_points_to_plot is not None and len(optional_points_to_plot) > 0:
                        particle_dist = []
                        for p in optional_points_to_plot:
                            dist = self.distance(p, target_pos)
                            particle_dist.append(dist)
                        pMin = min(particle_dist)
                        pMax = max(particle_dist)
                        pAvg = sum(particle_dist) / float(len(particle_dist))
                        print("{} Particles, Min dist: {}, Avg dist: {}, Max Dist: {}".format(len(optional_points_to_plot), pMin,
                                                                                              pAvg, pMax))

            self.satellite_found.put(False)
            self.satellite_steps.put(steps)

        except:
            self.satellite_error.put(traceback.format_exc())

    def simulate_with_sos(self, next_angle, params):
        #Run simulation allow satellite to fire SOS messages.
        """
        Args:
            next_angle(func): Student submission function for angle to send sos message.
            params(dict): Test parameters.

        Raises:
            Exception if error running submission.
        """
        self._reset()

        # make the test somewhat repeatable by seeding the RNG.
        random.seed(params['map_seed'])

        # build world
        planets_r_and_mass = []
        for planet_params in params['planets']:
            planet_r = tuple(v * AU for v in planet_params[0])
            planet_mass = planet_params[1] * MASS_EARTH
            planets_r_and_mass.append((planet_r, planet_mass))

        solar_system = SolarSystem(mass_sun=MASS_SUN * params['sun_mass'],
                                   planets_r_and_mass=planets_r_and_mass,
                                   elliptical_orbit_factor=params['elliptical_orbit_factor'])

        target_r = tuple(v * AU for v in params['target'][0])
        target_mass = params['target'][1] * MASS_EARTH
        target = SolarSystem.init_body_in_orbit_at_x_and_y(
            mass_sun=solar_system.sun.mass,
            r=target_r,
            mass_body=target_mass)

        tolerance = .05 * AU
        other_info = None
        steps = 0

        sos_trans_steps = 3
        hits = 0
        success_hits = 10
        bodies = solar_system.planets
        num_planets = solar_system.get_num_planets()

        # Set up the particle plotter if requested
        if PLOT_PARTICLES:
            import turtle  # Only import if plotting is on.
            turtle.tracer(n=0)
            turtle.setup(width=WINDOW_SIZE, height=WINDOW_SIZE)
            turtle.setworldcoordinates(-5, -5, 5, 5)

            # paint bg black
            turtle.clearscreen()
            turtle.colormode(255)
            turtle.bgcolor("black")
            turtle.delay(0)
            turtle.hideturtle()
            turtle.penup()

            # set starting point for satellite trail
            turtle.setposition(target.r[0] / AU, target.r[1] / AU)
            turtle.pencolor("red")
            turtle.pendown()
            turtle.ht()

            # declare turtles
            sun_turtle = None
            planet_turtle_list = None
            target_turtle = None
            estimate_turtle = None
            comms_turtle = None
            turtle_list = []
            time_turtle = turtle.Turtle(visible=False)
            time_turtle.setposition(3, -5)

            tc_turtle = turtle.Turtle(visible=False)
            tc_turtle.setposition(-5, -5)
            tc_turtle.color("white")
            tc_turtle.write(f'Test Case: {params["test_case"]}')

            msg_turtle = turtle.Turtle(visible=False)
            msg_turtle.setposition(-3, -5)
            msg_turtle.color("white")
            msg_turtle.write(f'Message Transmission Progress:')

            def pause(x, y):
                time.sleep(PAUSE_DURATION)
            button = turtle.Turtle()
            button.shapesize(0.5, 0.5, 0.5)
            button.shape('square')
            button.color('white')
            button.fillcolor('orange')
            button.penup()
            button.goto(3.5, 4.9)
            button.write("Pause:", align='left')
            button.setx(4.35)
            button.sety(5.05)
            button.onclick(pause)
            button.showturtle()

            progress_squares = []
            color_map = [
                '#FF4E11',
                '#FF8E15',
                '#FF8E15',
                '#FAB733',
                '#FAB733',
                '#E8DE04',
                '#E8DE04',
                '#00D100',
                '#00D100',
                '#00D100',
            ]

            for i in range(success_hits):
                sq = turtle.Turtle(visible=False)
                sq.setposition(.2+(.2*i), -5)
                sq.color("white")
                sq.fillcolor(color_map[i])
                for i in range(4):
                        sq.forward(.2)
                        sq.left(90)
                progress_squares.append(sq)

        try:
            while steps < params['max_steps']:

                def get_percent_illumination(a, b, c):
                    # get angle at vertex b between segment ab and bc in radians between 0-2 pi
                    # where a is sun, b is planet, and c is satellite
                    phase_angle = (math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0]))
                    phase_angle = phase_angle + 2*math.pi if phase_angle < 0 else phase_angle
                    percent_illumination = 50 * (1 + math.cos(phase_angle))
                    return percent_illumination

                def percent_illuminations_sense_func(x: float, y: float):
                    return [get_percent_illumination(solar_system.sun.r, planet.r, (x, y)) for planet in solar_system.planets]

                def sense(x: float, y: float):
                    return [random.gauss(get_percent_illumination(solar_system.sun.r, planet.r, (x, y)),
                                         params['pi_measurement_noise'])
                            for planet in solar_system.planets]

                # Invoke student function to get student result
                target2 = solar_system.move_body(copy.deepcopy(target))
                radius = math.sqrt(target.r[0]**2 + target.r[1]**2)
                beta_turning_angle = (math.atan2(target2.v[1], target2.v[0]) - math.atan2(target.v[1], target.v[0]) + 2*math.pi) % (2 * math.pi)
                distance = beta_turning_angle * radius
                satellite_length = 10.2
                steering = math.atan2(satellite_length, radius)
                percent_illuminations = sense(target.r[0], target.r[1])

                result = next_angle(
                    solar_system=copy.deepcopy(solar_system),
                    percent_illuminated_measurements=percent_illuminations,
                    percent_illuminated_sense_func=percent_illuminations_sense_func,
                    distance=distance,
                    steering=steering,
                    other=other_info
                )
                if len(result) == 4:
                    bearing, xy_estimate, other_info, optional_points_to_plot = result

                    if not isinstance(optional_points_to_plot, list):
                        raise TypeError(f"Expected returned optional_points_to_plot to be a list "
                                        f"but it was actually a {type(optional_points_to_plot)}")
                    if len(optional_points_to_plot) > 0:
                        if not isinstance(optional_points_to_plot[0], tuple):
                            raise TypeError(f"Expected the element in optional_points_to_plot to be a tuple "
                                            f"but it was actually a {type(optional_points_to_plot[0])}")
                        if len(optional_points_to_plot[0]) < 2 or len(optional_points_to_plot[0]) > 3:
                            raise TypeError(f"Expected the element in optional_points_to_plot to have 2-3 elements, "
                                            f"such as (x,y) or (x,y,h), "
                                            f"but it had {len(optional_points_to_plot[0])} elements: "
                                            f"{optional_points_to_plot[0]}")
                elif len(result) == 3:
                    bearing, xy_estimate, other_info = result
                    optional_points_to_plot = None
                else:
                    print("next_angle did not return correct number of return values!")

                if not isinstance(bearing, float):
                    raise TypeError(f"Expected the returned bearing to be a float "
                                    f"but it was actually a {type(bearing)}")
                if not isinstance(xy_estimate, tuple):
                    raise TypeError(f"Expected the returned xy_estimate to be a tuple "
                                    f"but it was actually a {type(xy_estimate)}")
                if len(xy_estimate) != 2:
                    raise TypeError(f"Expected the returned xy_estimate to have 2 elements, such as (x,y), "
                                    f"but it had {len(optional_points_to_plot[0])} elements: {xy_estimate}")

                # Calculate actual result position and bearing of the target next timestep
                target = solar_system.move_body(target)
                target_pos = (target.r[0], target.r[1])
                bearing = max(-PI, bearing)
                bearing = min(bearing, PI)

                # Rotate the solar system for the next timestep
                solar_system.move_planets()

                hit = False

                if steps % sos_trans_steps == 0:

                    # Transmit SoS towards home planet
                    home_planet_index = num_planets - 1
                    home_planet_vect = bodies[home_planet_index].r
                    home_planet_loc = (home_planet_vect[0], home_planet_vect[1])
                    target_x, target_y = target_pos

                    t0 = (home_planet_vect[0] - target_x) * math.cos(bearing) + (home_planet_vect[1] - target_y) * math.sin(bearing)

                    closest_point = (target_x + t0*math.cos(bearing) , target_y + t0*math.sin(bearing))
                    separation = self.distance(home_planet_loc, closest_point)

                    point1 = (target_x / AU, target_y / AU)
                    point2 = (target_x / AU + WINDOW_SIZE * math.cos(bearing),
                              target_y / AU + WINDOW_SIZE * math.sin(bearing))

                    if separation < tolerance:
                        point2 = ((target_x + t0 * math.cos(bearing)) / AU, (target_y + t0 * math.sin(bearing)) / AU)
                        hit = True
                        hits += 1
                    else:
                        pass

                    sos_attempt = True

                else:
                    sos_attempt = False

                if PLOT_PARTICLES:

                    s = turtle.getscreen()
                    s.tracer(0, 1)

                    # plot particles
                    if optional_points_to_plot:

                        # Add turtles if needed.
                        while len(optional_points_to_plot) > len(turtle_list):
                            new_turtle = turtle.Turtle()
                            new_turtle.penup()
                            turtle_list.append(new_turtle)

                        # remove turtles if needed.
                        while len(optional_points_to_plot) < len(turtle_list):
                            turtle_list[-1].hideturtle()
                            turtle_list = turtle_list[0:-1]

                        # paint particles
                        for i in range(len(optional_points_to_plot)):
                            t = turtle_list[i]
                            p = optional_points_to_plot[i]
                            # Optionally plot heading if provided by student
                            if len(p) > 2:
                                t.shape("triangle")
                                t.shapesize(0.2, 0.4)
                                t.settiltangle(p[2] * 180 / math.pi)
                            else:
                                t.shape("circle")
                                t.shapesize(0.1, 0.1)
                            t.fillcolor("white")
                            t.color("white")
                            t.setposition(p[0] / AU, p[1] / AU)

                    # Draw the target satellite.
                    if target_turtle is not None:
                        target_turtle.hideturtle()
                    if target_turtle is None:
                        target_turtle = turtle.Turtle()
                        target_turtle.shape("circle")
                        target_turtle.shapesize(.25, .25)
                        target_turtle.pencolor("red")
                        target_turtle.fillcolor("red")
                        target_turtle.penup()
                    target_turtle.setposition(target_pos[0] / AU, target_pos[1] / AU)
                    target_turtle.showturtle()

                    # move planet target turtle and trail
                    turtle.setposition(target.r[0] / AU, target.r[1] / AU)

                    # Draw the student estimate of the satellite
                    if estimate_turtle is not None:
                        estimate_turtle.hideturtle()
                    if estimate_turtle is None:
                        estimate_turtle = turtle.Turtle()
                        estimate_turtle.shape("square")
                        estimate_turtle.shapesize(.2, .2)
                        estimate_turtle.fill = False
                        estimate_turtle.color("cyan")
                        estimate_turtle.penup()
                    estimate_turtle.setposition(xy_estimate[0] / AU, xy_estimate[1] / AU)
                    estimate_turtle.showturtle()

                    # Draw the sun
                    if sun_turtle is not None:
                        sun_turtle.hideturtle()
                    if sun_turtle is None:
                        sun_turtle = turtle.Turtle()
                        sun_turtle.shape("circle")
                        sun_turtle.shapesize(.75, .75)
                        sun_turtle.fill = False
                        sun_turtle.color("yellow")
                        sun_turtle.penup()
                    sun_turtle.setposition(0, 0)
                    sun_turtle.showturtle()

                    # Draw the planets
                    if planet_turtle_list is None:
                        planet_turtle_list = [turtle.Turtle() for _ in range(solar_system.get_num_planets())]
                        for i, planet_turtle in enumerate(planet_turtle_list or []):
                            size = .1 + solar_system.planets[i].mass / MASS_EARTH / 1000 * .4
                            planet_turtle.shape("circle")
                            planet_turtle.shapesize(size, size)
                            planet_turtle.pencolor("lime")
                            planet_turtle.fillcolor("lime")
                            if i == home_planet_index:
                                planet_turtle.pencolor("skyblue")
                                planet_turtle.fillcolor("magenta")
                            planet_turtle.penup()
                            x = solar_system.planets[i].r[0] / AU
                            y = solar_system.planets[i].r[1] / AU
                            planet_turtle.setposition(x, y)
                            planet_turtle.pendown()
                    else:
                        for i, planet_turtle in enumerate(planet_turtle_list or []):
                            x = solar_system.planets[i].r[0] / AU
                            y = solar_system.planets[i].r[1] / AU
                            planet_turtle.setposition(x, y)
                            planet_turtle.showturtle()

                    # Draw steps
                    time_turtle.clear()
                    time_turtle.color("white")
                    time_turtle.write(f'Time Step: {steps}')

                    # Draw communication towards target planet.
                    if comms_turtle is not None:
                        comms_turtle.hideturtle()
                    if comms_turtle is None:
                        comms_turtle = turtle.Turtle()
                        comms_turtle.pencolor("red")
                        comms_turtle.penup()
                    if sos_attempt == True:
                        if hit:
                            comms_turtle.pencolor("green")
                            sq = progress_squares[hits-1]
                            sq.begin_fill()
                            for i in range(4):
                                sq.forward(.2)
                                sq.left(90)
                            sq.end_fill()
                        else:
                            comms_turtle.pencolor("red")

                        comms_turtle.goto(point1)
                        comms_turtle.pendown()
                        comms_turtle.goto(point2)
                        comms_turtle.penup()
                    else:
                        comms_turtle.clear()

                    s.update()

                if hits >= success_hits:
                    self.satellite_found.put(True)
                    self.satellite_steps.put(steps)
                    return

                if PAUSE_FIRST and steps == 0:
                    time.sleep(PAUSE_DURATION)

                steps += 1

            self.satellite_found.put(False)
            self.satellite_steps.put(steps)

        except:
            self.satellite_error.put(traceback.format_exc())


NOT_FOUND = "Part {} - Test Case {}: Time limit reached: You didn't localize the object quick enough. You took {} step(s) which exceeded the {} allowable step(s)."


class CaseRunner(unittest.TestCase):
    """Run test case using specified parameters.

    Attributes:
        simulator(PlanetSimulator): Simulation.
    """

    @classmethod
    def setUpClass(cls):
        """Setup test class.
        """
        cls.simulator = PlanetSimulator()

    def run_with_params(self, k, test_params, test_method, student_method):
        """Run test case with parameters.

        Args:
            k(int): Test case global parameters.
            test_params(dict): Test parameters.
            test_method(func): Test function.
            student_method(func): Student submission function.
        """
        test_params.update(GLOBAL_PARAMETERS[k])

        error_message = ''
        steps = None
        planet_found = False

        if DEBUGGING_SINGLE_PROCESS:
            test_method(student_method, test_params)
        else:
            test_process = mproc.Process(target=test_method, args=(student_method, test_params))

            try:
                test_process.start()
                test_process.join(TIME_LIMIT)
            except Exception as exp:
                error_message += str(exp) + ' '

            if test_process.is_alive():
                test_process.terminate()
                error_message = ('Test aborted due to CPU timeout. ' +
                                 'Test was expected to finish in fewer than {} second(s).'.format(TIME_LIMIT))
                print(error_message)

        if not error_message:
            if not self.simulator.satellite_error.empty():
                error_message += self.simulator.satellite_error.get()

            if not self.simulator.satellite_found.empty():
                planet_found = self.simulator.satellite_found.get()

            if not self.simulator.satellite_steps.empty():
                steps = self.simulator.satellite_steps.get()

        self.assertFalse(error_message, error_message)
        self.assertTrue(planet_found, NOT_FOUND.format(test_params['part'],
                                                       test_params['test_case'],
                                                       steps,
                                                       test_params['max_steps']))


class PartATestCase(CaseRunner):
    """Test Part A (localization only, no messaging)

    Attributes:
        test_method(func): Test function.
        student_method(func): Student submission function.
        params(dict): Test parameters.
    """

    def setUp(self):
        """Setup for each test case.
        """

        if planet1Exc:
            raise planet1Exc

        self.test_method = self.simulator.simulate_without_sos
        self.student_method = solar_locator.estimate_next_pos

        self.params = dict()
        self.params['part'] = 'A'

    def test_case01(self):
        self.run_with_params(1, self.params, self.test_method, self.student_method)

    def test_case02(self):
        self.run_with_params(2, self.params, self.test_method, self.student_method)

    def test_case03(self):
        self.run_with_params(3, self.params, self.test_method, self.student_method)

    def test_case04(self):
        self.run_with_params(4, self.params, self.test_method, self.student_method)

    def test_case05(self):
        self.run_with_params(5, self.params, self.test_method, self.student_method)

    def test_case06(self):
        self.run_with_params(6, self.params, self.test_method, self.student_method)

    def test_case07(self):
        self.run_with_params(7, self.params, self.test_method, self.student_method)

    def test_case08(self):
        self.run_with_params(8, self.params, self.test_method, self.student_method)

    def test_case09(self):
        self.run_with_params(9, self.params, self.test_method, self.student_method)

    def test_case10(self):
        self.run_with_params(10, self.params, self.test_method, self.student_method)


class PartBTestCase(CaseRunner):
    """Test Part B (localization and messaging SOS to home planet )

    Attributes:
        test_method(func): Test function.
        student_method(func): Student submission function.
        params(dict): Test parameters.
    """

    def setUp(self):
        """Setup for each test case.
        """

        if planet1Exc:
            raise planet1Exc

        self.test_method = self.simulator.simulate_with_sos
        self.student_method = solar_locator.next_angle

        self.params = dict()
        self.params['part'] = 'B'

    def test_case01(self):
        self.run_with_params(1, self.params, self.test_method, self.student_method)

    def test_case02(self):
        self.run_with_params(2, self.params, self.test_method, self.student_method)

    def test_case03(self):
        self.run_with_params(3, self.params, self.test_method, self.student_method)

    def test_case04(self):
        self.run_with_params(4, self.params, self.test_method, self.student_method)

    def test_case05(self):
        self.run_with_params(5, self.params, self.test_method, self.student_method)

    def test_case06(self):
        self.run_with_params(6, self.params, self.test_method, self.student_method)

    def test_case07(self):
        self.run_with_params(7, self.params, self.test_method, self.student_method)

    def test_case08(self):
        self.run_with_params(8, self.params, self.test_method, self.student_method)

    def test_case09(self):
        self.run_with_params(9, self.params, self.test_method, self.student_method)

    def test_case10(self):
        self.run_with_params(10, self.params, self.test_method, self.student_method)


# This flag is used to check whether project files listed in the json have been modified.
# Modifications include (but are not limited to) print statements, changing flag values, etc.
# If you have modified the project files in some way, the results may not be accurate.
# Turn file_checker on by setting the flag to True to ensure you are running against
# the same framework as the Gradescope autograder.
file_checker = False  # set to True to turn file checking on

if file_checker:
    import json
    import hashlib
    import pathlib
    print("File checking is turned on.")
    with open('file_check.json', 'r') as openfile:
        json_dict = json.load(openfile)

    modified_files = []
    for file in json_dict:
        f = str(file)
        try:
            current = pathlib.Path(file).read_text().replace(' ', '').replace('\n', '')
            file_hash = hashlib.sha256(current.encode()).hexdigest()
            if file_hash != json_dict[f]:
                modified_files.append(f)
        except:
            print(f'File ({f}) not in project folder.')

    if len(modified_files) == 0:
        print("You are running against the same framework as the Gradescope autograder.")
    else:
        print("Warning. The following files have been modified and the results may not be accurate:")
        print(", ".join(modified_files))

# Only run all of the test automatically if this file was executed from the command line.
# Otherwise, let Nose/py.test do it's own thing with the test cases.
if __name__ == "__main__":
    if stack_trace:
        print(stack_trace)
        print('score: 0')
    else:
        student_id = solar_locator.who_am_i()
        if student_id:
            cases = []
            if PART_A is True:
                cases.append(PartATestCase)
            if PART_B is True:
                cases.append(PartBTestCase)
            suites = [unittest.TestSuite(unittest.TestLoader().loadTestsFromTestCase(case)) for case in cases]

            total_passes = 0

            try:
                for i, suite in zip(list(range(1, 1 + len(suites))), suites):
                    print("====================\nTests for Part {}:".format(i))

                    result = unittest.TestResult()
                    suite.run(result)

                    for x in result.errors:
                        print(x[0], x[1])
                    for x in result.failures:
                        print(x[0], x[1])

                    num_errors = len(result.errors)
                    num_fails = len(result.failures)
                    num_passes = result.testsRun - num_errors - num_fails
                    total_passes += num_passes

                    print("Successes: {}\nFailures: {}\n".format(num_passes, num_errors + num_fails))

                    # We cap the maximum score to 101 if they pass more than 12.5 test cases.
                    overall_score = total_passes * CREDIT_PER_PASS
            except Exception as e:
                print(e)
                overall_score = 0
            if overall_score > 100:
                print("Score above 100:", overall_score, " capped to 100!")
                overall_score = 100
            print("====================\nOverall Score: {}".format(overall_score))
        else:
            print("Student ID not specified.  Please fill in 'whoami' variable.")
            print('score: 0')
