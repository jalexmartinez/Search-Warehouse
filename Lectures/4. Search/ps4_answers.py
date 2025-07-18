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

######################################################################
# How to use:
#
#   For fill-in questions, change the value of each answer variable.
#
#   For programming questions, change the implementation of the
#   function provided.
#
#   Run the file from the command line to check:
#
#       python ps4_answers.py
#
######################################################################

from math import *
import random
import numpy as np # this is pre-imported but you don't have to use it
import traceback
import checkutil
import copy

# NOTE: All questions are presented in the problem set videos (lesson modules), please watch these to get the full
# context on the questions below.  Following each question video is a subsequent video with the answer.

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

# STUDENT ID

# Please specify your GT login ID in the whoami variable (ex: jsmith223).

whoami = 'jamartinez348'

n = float('nan')

# QUESTION 1: ADMISSIBLE HEURISTIC
#
# Is the grid function an admissable heuristic?
# Replace n with 0 for false, 1 for true
#
q1_yes  = 1
q1_no   = 0

# QUESTION 2: ADMISSIBLE HEURISTIC 2
#
# Is the grid function an admissable heuristic?
# Replace n with 0 for false, 1 for true
#
q2_yes = 0
q2_no  = 1

# QUESTION 3: BAD HEURISTIC
#
# What may happen if h is not admissable?
# Replace n with 0 for unchecked, 1 for checked
#
q3_Astar_finds_optimal_path_always   = 0
q3_Astar_may_find_suboptimal_path    = 1
q3_Astar_may_fail_to_find_path       = 0
q3_None_of_the_above                 = 0

# QUESTION 4: DIAGONAL MOTION
#
# Replace the values n with the correct values from
#
q4_dynamic_programming = [[ 3,   2,   2  ],
                          [ 3, 'inf', 1],
                          [ 4, 'inf', 0]]
                                    

# QUESTION 5: STOCHASTIC MOTION

delta = [[-1, 0],   # go up
         [0, -1],   # go left
         [1, 0],    # go down
         [0, 1]]    # go right

delta_name = ['^', '<', 'v', '>']  # Use these when creating your policy grid.


# --------------------------------------------------
#  Modify the function stochastic_value below
# --------------------------------------------------

def q5_stochastic_motion(grid, goal, cost_step, collision_cost, success_prob):

    # Make sure to mark the goal with an asterisk character '*' (which was not shown in the video).
    # Your policy should also ignore any squares not connected to the goal.
    # In the video Sebastian was initializing the value function with 1000 and using a collision
    # cost of 100 to get the displayed result. In the quiz the value function will be initialized
    # to what ever the collision cost is.

    # Be sure to fix or replace the following two initialization lines with the
    # correct initialization of value and policy
    value = [[collision_cost for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]


    goal = copy.deepcopy(goal)
    goal.insert(0,100)
    grid_array = np.array(grid)

    policy_grid = np.array(policy)

    policy_grid[goal[1]][goal[2]] = '*'

    delta = [[0, -1, 0],  # go up
             [0, 0, -1],  # go left
             [0, 1, 0],  # go down
             [0, 0, 1]]  # go right

    delta_name = ['^', '<', 'v', '>']

    delta_array = np.array(delta)
    visited_nodes = np.empty((0, len(goal)-1), int)
    possible_states = np.array([goal])

    value_grid = np.array(value, dtype=float)
    value = 0
    value_grid[goal[1]][goal[2]] = value

    different = True

    while different == True:
        possible_states_old = possible_states.copy()
        sum_states_old = possible_states_old.sum() + value_grid.sum()
        indices = np.argsort(possible_states[:, 2])

        possible_states = possible_states[indices]

        for state in possible_states:

            if visited_nodes.size == 0 or not any((state[1:] == visited_nodes).all(1)):
                visited_nodes = np.append(visited_nodes, [state[1:]], axis=0)

            for idx, movement in enumerate(delta_array):

                new_position = state + movement

                if new_position[1] < 0 or new_position[2] < 0:
                    continue
                elif new_position[1] > len(grid)-1 or new_position[2] > len(grid[0])-1:
                    continue
                elif grid_array[new_position[1]][new_position[2]] == 1:
                    value_grid[new_position[1]][new_position[2]] = 100
                    continue
                else:

                    if any(state[1:] != goal[1:]):
                        right_loc = state[1:] + delta_array[(idx-1) % (len(delta_array))][1:]
                        left_loc = state[1:] + delta_array[(idx+1) % (len(delta_array))][1:]
                        if right_loc[0] < 0 or right_loc[1] < 0:
                            right_cost = 100.0
                        elif right_loc[0] > len(grid)-1 or right_loc[1] > len(grid[0])-1:
                            right_cost = 100.0
                        else:
                            right_cost = value_grid[right_loc[0], right_loc[1]]

                        if left_loc[0] < 0 or left_loc[1] < 0:
                            left_cost = 100.0
                        elif left_loc[0] > len(grid)-1 or left_loc[1] > len(grid[0])-1:
                            left_cost = 100.0
                        else:
                            left_cost = value_grid[left_loc[0], left_loc[1]]

                        new_cost = success_prob * value_grid[new_position[1]][new_position[2]] + (1-success_prob)/2 \
                         * right_cost + (1-success_prob)/2 * left_cost + float(cost_step)

                        if new_cost < value_grid[state[1]][state[2]]:
                            state[0] = new_cost
                            value_grid[state[1]][state[2]] = new_cost
                            policy_grid[state[1]][state[2]] = delta_name[idx]
                            for i in range(len(possible_states)):

                                if np.array_equal(possible_states[i, 1:], state[1:]):
                                    possible_states[i] = state
                        if any((new_position[1:] == x[1:]).all() for x in possible_states):
                            continue
                        possible_states = np.append(possible_states, [new_position], axis=0)
                    else:
                        if any((new_position[1:] == x[1:]).all() for x in possible_states):
                            continue
                        possible_states = np.append(possible_states, [new_position], axis=0)

        sum_states_new = possible_states.sum() + value_grid.sum()
        different = not np.array_equal(sum_states_new, sum_states_old)


    return value_grid, policy_grid


# --------------------------------------------------
#  You can use the code below to test your solution
# --------------------------------------------------

# grid = [[0, 0, 0, 0],
# #         [0, 0, 0, 0],
# #         [0, 0, 0, 0],
# #         [0, 1, 1, 0]]

grid = [[0,0,0],
        [0,0,0]]
goal = [0, len(grid[0]) - 1]  # Goal is in top right corner
cost_step = 1
collision_cost = 100
success_prob = 0.5

value, policy = q5_stochastic_motion(grid, goal, cost_step, collision_cost, success_prob)

for row in value:
    print(row)
for row in policy:
    print(row)


# Expected outputs:
#
# [57.9029, 40.2784, 26.0665,  0.0000]
# [47.0547, 36.5722, 29.9937, 27.2698]
# [53.1715, 42.0228, 37.7755, 45.0916]
# [77.5858, 100.00, 100.00, 73.5458]
#
# ['>', 'v', 'v', '*']
# ['>', '>', '^', '<']
# ['>', '^', '^', '<']
# ['^', ' ', ' ', '^']
# 
# Make sure to mark the goal with an asterisk character '*' (which was not shown in the video). 
# If not, your submission will not be accepted.
# Make sure to output a "*" in the goal position. Your policy should also ignore any squares not connected to the goal.

######################################################################
# Grading methods
#
# Do not modify code below this point.
#
# The auto-grader does not use any of the below code
# 
######################################################################

FILL_IN_TEST_CASES = ({'variable_name': 'q1_yes',
                       'str_func':          checkutil.float_to_str,
                       'answer_hash':       '6631178b642694e7514dacd18f3cde95',
                       'points_avail':      1./2.},
                      {'variable_name': 'q1_no',
                          'str_func':          checkutil.float_to_str,
                       'answer_hash':       '24966cce05b1de3f3a92e19695b25111',
                       'points_avail':      1./2.},
                      {'variable_name': 'q2_yes',
                          'str_func':          checkutil.float_to_str,
                       'answer_hash':       'e964a07660688f477c8dd9bbe735ccaa',
                       'points_avail':      1./2.},
                      {'variable_name': 'q2_no',
                          'str_func':          checkutil.float_to_str,
                       'answer_hash':       '03c7721e1b6f42e30e04ecf61127b9e2',
                       'points_avail':      1./2.},
                      {'variable_name': 'q3_Astar_finds_optimal_path_always',
                          'str_func':          checkutil.float_to_str,
                       'answer_hash':       'd9116fe59e7dacb20397c29fc721a1a9',
                       'points_avail':      1./4.},
                      {'variable_name': 'q3_Astar_may_find_suboptimal_path',
                          'str_func':          checkutil.float_to_str,
                       'answer_hash':       'b24b910189f73d58bdab93b75a53427c',
                       'points_avail':      1./4.},
                      {'variable_name': 'q3_Astar_may_fail_to_find_path',
                          'str_func':          checkutil.float_to_str,
                       'answer_hash':       '3176dd3a23222212919eb60083a62d4d',
                       'points_avail':      1./4.},
                      {'variable_name': 'q3_None_of_the_above',
                          'str_func':          checkutil.float_to_str,
                       'answer_hash':       '6407ff0c145be7c3a815c7f9393a03c7',
                       'points_avail':      1./4.},
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (0,0),
                        'str_func':          checkutil.float_to_str,
                        'answer_hash':       '3a1932aa70013ec7aae31073bfea82f0',
                        'points_avail':      1./6. },
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (0,1),
                        'str_func':          checkutil.float_to_str,
                        'answer_hash':       'c1e220592da6ca631bdb5cd276dc0df9',
                        'points_avail':      1./6},                      
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (0,2),
                        'str_func':          checkutil.float_to_str,
                        'answer_hash':       'e6897e3da513e9a670e3a58495e93b56',
                        'points_avail':      1./6. },
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (1,0),
                        'str_func':          checkutil.float_to_str,
                        'answer_hash':       'b7dc6f2f034d48f14aa8398df8d2618b',
                        'points_avail':      1./6. },
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (1,2),
                        'str_func':          checkutil.float_to_str,
                        'answer_hash':       '587fcfffa049e6d3c1aefdb1577a081d',
                        'points_avail':      1./6},                      
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (2,0),
                        'str_func':          checkutil.float_to_str,
                        'answer_hash':       'bb61102429aa1b3271c6119379b87e0d',
                        'points_avail':      1./6. }, )

# PROGRAMMING


def format_value_and_policy_output(value_policy_lists):
    if len(value_policy_lists) == 4: # if printing student output
        value_grid, policy_grid, correct_values, correct_policy = value_policy_lists
    else: # if printing expected output
        value_grid, policy_grid = value_policy_lists
        correct_policy = correct_values = [[True for _c in _r] for _r in value_grid]

    # symbols to indicate incorrect items
    start_symbol = '('
    end_symbol = ')'

    try:
        # create formatted value grid
        grid = []
        for row in zip(value_grid, correct_values):
            r = '['
            c = []
            for value_grid, v_is_correct in zip(*row):
                c.append(f' {value_grid:>.4f} '.rjust(13) if v_is_correct else f'{start_symbol}{value_grid:>.4f}{end_symbol}'.rjust(13))
            r += ','.join(c) + ']'
            grid.append(r)

        value_grid_formatted = '[' + ',\n '.join(grid) + ']\n\n'

        # create formatted policy grid
        grid = []
        for row in zip(policy_grid, correct_policy):
            r = '['
            c = []
            for p, pol_is_correct in zip(*row):
                c.append(f' {p} ' if pol_is_correct else f'{start_symbol}{p}{end_symbol}')
            r += ','.join(c) + ']'
            grid.append(r)

        policy_grid_formatted = '[' + ',\n '.join(grid) + ']'
        output = value_grid_formatted + policy_grid_formatted

    except Exception:
        stack_trace = traceback.format_exc()
        print(stack_trace)
        output = str(value_policy_lists)
    return output


def q5_check_output(students_values_and_policy, expected_values_and_policy):

    # check that input is as expected
    if len(students_values_and_policy) != len(expected_values_and_policy):
        print("Expected 2 inputs but, received {}".format(len(students_values_and_policy)))
        return False

    student_values, student_policy = students_values_and_policy
    expected_values, expected_policy = expected_values_and_policy

    # margin for difference between expected and resulting policy values
    eps = 1.e-3

    # default return value
    ret_val = True

    # Check the values
    num_rows = len(student_values)
    num_cols = len(student_values[0])

    if len(student_values) != len(expected_values) or len(student_values[0]) != len(expected_values[0]):
        print("Values matrix is the wrong size: {}x{}".format(len(student_values),len(student_values[0])))
        return False

    correct_values = [[True for _ in range(num_cols)] for _ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            if abs(student_values[i][j] - expected_values[i][j]) > eps :
                ret_val = False
                correct_values[i][j] = False

    # Check the policy
    num_rows = len(student_policy)
    num_cols = len(student_policy[0])

    if len(student_policy) != len(expected_policy) or len(student_policy[0]) != len(expected_policy[0]):
        print("Values matrix is the wrong size: {}x{}".format(len(student_policy), len(student_policy[0])))
        return False

    correct_policy = [[True for _ in range(num_cols)] for _ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            if abs(student_policy[i][j] != expected_policy[i][j]):
                ret_val = False
                correct_policy[i][j] = False

    return ret_val, (correct_values, correct_policy)


CODE_TEST_CASES = ({'function_name': 'q5_stochastic_motion',
                    'function_input': dict( grid = [[0,0,0,0],
                                                    [0,0,0,0],
                                                    [0,0,0,0],
                                                    [0,1,1,0]],
                                             goal = [0, 3],
                                             cost_step = 1,
                                             collision_cost = 100,
                                             success_prob = 0.5),
                    'expected_output': ( [[57.9029, 40.2784, 26.0665, 0.0000],
                                          [47.0547, 36.5722, 29.9937, 27.2698],
                                          [53.1715, 42.0228, 37.7755, 45.0916],
                                          [77.5858, 100.00, 100.00, 73.5458]],
                                          [['>', 'v', 'v', '*'],
                                           ['>', '>', '^', '<'],
                                           ['>', '^', '^', '<'],
                                           ['^', ' ', ' ', '^']]),
                    'outputs_match_func': q5_check_output,
                    'output_to_str_func': format_value_and_policy_output},
                   {'function_name': 'q5_stochastic_motion',
                    'function_input': dict(grid=[[0, 0, 0, 0],
                                                 [0, 0, 0, 0],
                                                 [0, 0, 0, 0],
                                                 [0, 1, 1, 0]],
                                           goal=[0, 3],
                                           cost_step=1,
                                           collision_cost=100,
                                           success_prob=0.7),
                    'expected_output': ([[38.2147, 25.8261, 19.1079, 0.0000],
                                         [27.5762, 23.1824, 20.7190, 19.1079],
                                         [30.7752, 25.3677, 23.4918, 27.8888],
                                         [52.5427, 100.0000, 100.0000, 50.5221]],
                                        [['>', 'v', '>', '*'],
                                         ['>', '>', '^', '^'],
                                         ['>', '^', '^', '<'],
                                         ['^', ' ', ' ', '^']]),
                    'outputs_match_func': q5_check_output,
                    'output_to_str_func': format_value_and_policy_output},
                   {'function_name': 'q5_stochastic_motion',
                    'function_input': dict(grid=[[0, 0, 0, 0],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 0],
                                                 [0, 1, 1, 0]],
                                           goal=[0, 3],
                                           cost_step=1,
                                           collision_cost=100,
                                           success_prob=0.5),
                    'expected_output': ([[83.8546, 72.4637, 51.0000, 0.0000],
                                         [86.4912, 83.8546, 100.0000, 51.0000],
                                         [90.4010, 87.4561, 87.7143, 73.4286],
                                         [96.2005, 100.0000, 100.0000, 87.7143]],
                                        [['>', '>', '>', '*'],
                                         ['>', '^', ' ', '^'],
                                         ['>', '^', '>', '^'],
                                         ['^', ' ', ' ', '^']]),
                    'outputs_match_func': q5_check_output,
                    'output_to_str_func': format_value_and_policy_output}, )
                     
if __name__ == '__main__':
    checkutil.check( FILL_IN_TEST_CASES, CODE_TEST_CASES, locals() )
