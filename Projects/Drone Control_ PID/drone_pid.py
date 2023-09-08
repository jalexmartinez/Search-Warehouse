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
from scipy.optimize import minimize


def pid_thrust(target_elevation, drone_elevation, tau_p=0.0, tau_d=0.0, tau_i=0.0, data: dict() = {}):
    '''
    Student code for Thrust PID control. Drone's starting x, y position is (0, 0).
    
    Args:
    target_elevation: The target elevation that the drone has to achieve
    drone_elevation: The drone's elevation at the current time step
    tau_p: Proportional gain
    tau_i: Integral gain
    tau_d: Differential gain
    data: Dictionary that you can use to pass values across calls.
        Reserved keys:
            max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.
    
    Returns:
        Tuple of thrust, data
        thrust - The calculated change in thrust using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call. 
            Reserved keys:
                max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.
    '''

    elevation_error = (target_elevation - drone_elevation) #may need to change this order

    if not data:
        data['thrust'] = {'elevation error': elevation_error , 'elevation error total': 0}
        elevation_error_old = data["thrust"]['elevation error']
        data['max_rpm_reached'] = False
    else:
        elevation_error_old = data["thrust"]["elevation error"]
    data["thrust"]["elevation error"] = elevation_error

    if data["max_rpm_reached"] == False:
        data["thrust"]["elevation error total"] += elevation_error

    thrust = tau_p * elevation_error + tau_d*(elevation_error-elevation_error_old) + tau_i * data["thrust"]["elevation error total"]


    return thrust, data


def pid_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data:dict() = {}):
    '''
    Student code for PD control for roll. Drone's starting x,y position is 0, 0.
    
    Args:
    target_x: The target horizontal displacement that the drone has to achieve
    drone_x: The drone's x position at this time step
    tau_p: Proportional gain, supplied by the test suite
    tau_i: Integral gain, supplied by the test suite
    tau_d: Differential gain, supplied by the test suite
    data: Dictionary that you can use to pass values across calls.
    
    Returns:
        Tuple of roll, data
        roll - The calculated change in roll using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.

    '''
    
    x_error = target_x - drone_x  

    if not data:
        data['x'] = {'x error': 0 , 'x error total': 0}
        x_error_old = data["x"]['x error']
        data['max_rpm_reached'] = False
    else:
        x_error_old = data["x"]["x error"]

    if data["max_rpm_reached"] == False:
        data["x"]["x error total"] += x_error

    roll = tau_p * x_error + tau_d*(x_error-x_error_old) + tau_i * data["x"]["x error total"]
    data["x"]["x error"] = x_error
    
    return roll, data 

def total_error(hover_error, drone_max_velocity, max_allowed_velocity, max_oscillations, total_oscillations,w1=1.0,w2=0.0, w3=0.0):

    osc_score = total_oscillations/max_oscillations
    vel_score =  drone_max_velocity/(max_allowed_velocity)
    err = w1*hover_error + w2*hover_error*vel_score + w3*hover_error*osc_score

    return err
def find_parameters_thrust(run_callback, tune='thrust', DEBUG=False, VISUALIZE=True):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on 
    tuning gain values for Thrust test cases only.
    
    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates 
                how well your PID gain values followed the specified path.
        
    tune: This will be passed by the test harness. 
            A value of 'thrust' means you only need to tune gain values for thrust. 
            A value of 'both' means you need to tune gain values for both thrust and roll.
    
    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs
    
    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''
    
    # Initialize a list to contain your gain values that you want to tune
    params = [0.01,10.0,0.0]
    dparams = [0.01, 0.01, 0.0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
    
    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}
    
    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
    
    # Calculate best_error from above returned values

    best_error = total_error(hover_error,drone_max_velocity, max_allowed_velocity, max_allowed_oscillations, total_oscillations)
    tol = .000000001

    while sum(dparams) > tol:
        for i in range(len(params)-1):
            params[i] += dparams[i]
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)

            if total_error(hover_error,drone_max_velocity, max_allowed_velocity, max_allowed_oscillations, total_oscillations) < best_error:
                best_error = total_error(hover_error,drone_max_velocity, max_allowed_velocity, max_allowed_oscillations, total_oscillations)
                dparams[i] *= 1.1
            else:
                params[i] -= 2.0 * dparams[i]
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                    thrust_params, roll_params, VISUALIZE=VISUALIZE)
                if total_error(hover_error,drone_max_velocity, max_allowed_velocity, max_allowed_oscillations, total_oscillations) < best_error:
                    best_error = total_error(hover_error,drone_max_velocity, max_allowed_velocity, max_allowed_oscillations, total_oscillations)
                    dparams[i] *= 1.1
                else:
                    params[i] += dparams[i]
                    dparams[i] *= 0.9


    return thrust_params, roll_params

def find_parameters_with_int(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on 
    tuning gain values for Thrust test case with Integral error
    
    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates 
                how well your PID gain values followed the specified path.
        
    tune: This will be passed by the test harness. 
            A value of 'thrust' means you only need to tune gain values for thrust. 
            A value of 'both' means you need to tune gain values for both thrust and roll.
    
    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs
    
    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''
    # Initialize a list to contain your gain values that you want to tune
    params = [0.0, 0.0, 0.0]
    dparams = [0.25, 15, 0.00003]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
        thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = hover_error
    tol = .0001

    while sum(dparams) > tol:
        for i in range(len(params)):
            params[i] += dparams[i]
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)

            if hover_error < best_error:
                best_error = hover_error
                dparams[i] *= 1.1
            else:
                params[i] -= 2.0 * dparams[i]
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                    thrust_params, roll_params, VISUALIZE=VISUALIZE)
                if hover_error < best_error:
                    best_error = hover_error
                    dparams[i] *= 1.1
                else:
                    params[i] += dparams[i]
                    dparams[i] *= 0.9
    
    return thrust_params, roll_params

def find_parameters_with_roll(run_callback, tune='both', DEBUG=False, VISUALIZE=False): 
    '''
    Student implementation of twiddle algorithm will go here. Here you will 
    find gain values for Thrust as well as Roll PID controllers.
    
    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates 
                how well your PID gain values followed the specified path.
        
    tune: This will be passed by the test harness. 
            A value of 'thrust' means you only need to tune gain values for thrust. 
            A value of 'both' means you need to tune gain values for both thrust and roll.
    
    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs
    
    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''
    # Initialize a list to contain your gain values that you want to tune
    params_thrust = [0.01,10.0,0.0]
    dparams_thrust = [0.01, 5.0, 0.0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params_thrust[0], 'tau_d': params_thrust[1], 'tau_i': params_thrust[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
        thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = hover_error
    tol = .0001

    while sum(dparams_thrust) > tol:
        for i in range(len(params_thrust)):
            params_thrust[i] += dparams_thrust[i]
            thrust_params = {'tau_p': params_thrust[0], 'tau_d': params_thrust[1], 'tau_i': params_thrust[2]}
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)

            if hover_error < best_error:
                best_error = hover_error
                dparams_thrust[i] *= 1.1
            else:
                params_thrust[i] -= 2.0 * dparams_thrust[i]
                thrust_params = {'tau_p': params_thrust[0], 'tau_d': params_thrust[1], 'tau_i': params_thrust[2]}
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                    thrust_params, roll_params, VISUALIZE=VISUALIZE)
                if hover_error < best_error:
                    best_error = hover_error
                    dparams_thrust[i] *= 1.1
                else:
                    params_thrust[i] += dparams_thrust[i]
                    dparams_thrust[i] *= 0.9
    
    

    params_roll = [0.0, 0.0, 0.0]
    dparams_roll = [0.01, 11., 0.0]

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
        thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = hover_error
    tol = .001

    while sum(dparams_roll) > tol:
        for i in range(len(params_roll)):
            params_roll[i] += dparams_roll[i]
            roll_params = {'tau_p': params_roll[0], 'tau_d': params_roll[1], 'tau_i': params_roll[2]}
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)

            if hover_error < best_error:
                best_error = hover_error
                dparams_roll[i] *= 1.1
            else:
                params_roll[i] -= 2.0 * dparams_roll[i]
                roll_params = {'tau_p': params_roll[0], 'tau_d': params_roll[1], 'tau_i': params_roll[2]}
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                    thrust_params, roll_params, VISUALIZE=VISUALIZE)
                if hover_error < best_error:
                    best_error = hover_error
                    dparams_roll[i] *= 1.1
                else:
                    params_roll[i] += dparams_roll[i]
                    dparams_roll[i] *= 0.9
    return thrust_params, roll_params

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'jmartinez348'
    return whoami
