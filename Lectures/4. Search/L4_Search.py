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

from Utilities import test
import numpy as np
import copy
# SEARCH LESSON MODULES
print("SEARCH LESSON MODULES", end="")

# --------------------------------------------------------------------
# 9. FIRST SEARCH PROGRAM
print("\n9. FIRST SEARCH PROGRAM")
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
#
#  Comment out any print statements used for debugging.

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0]]

heuristic= [[9, 8, 7, 6, 5, 4],
            [8, 7, 6, 5, 4, 3],
            [7, 6, 5, 4, 3, 2],
            [6, 5, 4, 3, 2, 1],
            [5, 4, 3, 2, 1, 0]]


init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):

    init = copy.deepcopy(init)
    init.insert(0,0)
    grid_array = np.array(grid)
    goal_array = np.array(goal)

    heuristic = [[9, 8, 7, 6, 5, 4],
                 [8, 7, 6, 5, 4, 3],
                 [7, 6, 5, 4, 3, 2],
                 [6, 5, 4, 3, 2, 1],
                 [5, 4, 3, 2, 1, 0]]

    delta = [[cost, -1, 0],  # go up
             [cost, 0, -1],  # go left
             [cost, 1, 0],  # go down
             [cost, 0, 1]]  # go right

    delta_array = np.array(delta)
    visited_nodes = np.empty((0, len(init)-1), int)
    possible_states = np.array([init])

    expanded_grid = copy.deepcopy(grid)
    expansion = 0
    expanded_grid[init[0]][init[1]] = expansion

    navigation_grid = copy.deepcopy(grid)
    navigation_grid = [[' ' for _ in row] for row in navigation_grid]

    while possible_states.size != 0:

        indices = np.argsort(possible_states[:, 2])

        # Sort A using these indices
        possible_states_sorted = possible_states[indices]
        possible_states = np.array([possible_states_sorted[-1]])

        if visited_nodes.size > 0:
            if possible_states[0][1] - prev_state_x > 0:
                navigation_grid[prev_state_x][prev_state_y] = "v"
            elif possible_states[0][1] - prev_state_x < 0:
                navigation_grid[prev_state_x][prev_state_y] = "^"
            elif possible_states[0][2] - prev_state_y > 0:
                navigation_grid[prev_state_x][prev_state_y] = ">"
            elif possible_states[0][2] - prev_state_y < 0:
                navigation_grid[prev_state_x][prev_state_y] = "<"


        expanded_grid[possible_states[0][1]][possible_states[0][2]] = expansion
        expansion = expansion + 1

        prev_state_x = possible_states[0][1]
        prev_state_y = possible_states[0][2]

        for state in possible_states:
            if visited_nodes.size == 0 or not any((state[1:] == visited_nodes).all(1)):
                visited_nodes = np.append(visited_nodes, [state[1:]], axis=0)

            elif np.array_equal(state[1:], goal_array):
                navigation_grid[state[1]][state[2]] = "*"
                return list(state), expanded_grid, navigation_grid

            else:
                index = np.where((possible_states == state).all(axis=1))[0]
                possible_states = np.delete(possible_states, index, axis=0)
                continue
            for movement in delta_array:
                #acount for movements outside of grid
                new_position = state + movement
                if any((new_position[1:] == visited_nodes).all(1)):
                    continue
                elif new_position[1] < 0 or new_position[2] < 0:
                    continue
                elif new_position[1] > len(grid)-1 or new_position[2] > len(grid[0])-1:
                    continue
                elif grid_array[new_position[1]][new_position[2]] == 1:
                    expanded_grid[new_position[1]][new_position[2]] = -1
                    continue
                else:
                    heuristic_value = heuristic[new_position[1]][new_position[2]]
                    new_position[0] = new_position[0] + heuristic_value
                    possible_states = np.append(possible_states, [new_position], axis=0)

    state = "fail"
    return state, expanded_grid, navigation_grid


path, expanded_grid, navigation_grid = search(grid, init, goal, cost)
exp_arr = np.array(expanded_grid)
nav = np.array(navigation_grid)

print(search(grid, init, goal, cost))



print("EXTRA TEST CASES (1):")
try:
    response = test.run_grader_1(search)
    print(response)
except Exception as err:
    print(str(err))

# --------------------------------------------------------------------
# 10. EXPANSION GRID
print("\n10. EXPANSION GRID")
# Modify the function search so that it returns
# a table of values called expand. This table
# will keep track of which step each node was
# expanded.
#
# Make sure that the initial cell in the grid
# you return has the value 0.
#
#  Comment out any print statements used for debugging.

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):
    path = None
    expand = []
    # TODO: ADD CODE HERE

    return path, expand


path, expand = search(grid, init, goal, cost)
print('Path:', path)
print('Expand:')
for i in range(len(expand)):
    print(expand[i])


print("EXTRA TEST CASES (2):")
try:
    response = test.run_grader_2(search)
    print(response)
except Exception as err:
    print(str(err))

# --------------------------------------------------------------------
# 11. PRINT PATH
print("\n11. PRINT PATH")
# Modify the the search function so that it returns
# a shortest path as follows:
#
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left,
# up, and down motions. Note that the 'v' should be
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):
    # Copy and paste your solution code from the previous exercise (#10)
    # TODO: CHANGE/UPDATE CODE
    path = None
    expand = []
    policy = []

    return path, expand, policy


path, expand, policy = search(grid, init, goal, cost)
for i in range(len(policy)):
    print(policy[i])

# --------------------------------------------------------------------
# 13. IMPLEMENT A*
print("\n13. IMPLEMENT A*")
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
#
# If there is no path from init to goal,
# the function should return the string 'fail'

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost, heuristic):
    # Copy and paste your solution code from the previous exercise (#11)
    # TODO: CHANGE/UPDATE CODE
    path = None
    expand = []
    policy = []

    return path, expand, policy


path, expand, policy = search(grid, init, goal, cost, heuristic)
for i in range(len(expand)):
    print(expand[i])

# --------------------------------------------------------------------
# 18. VALUE PROGRAM
print("\n18. VALUE PROGRAM")
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal.
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def compute_value(grid, goal, cost):

    goal = copy.deepcopy(goal)
    goal.insert(0,0)
    grid_array = np.array(grid)
    goal_array = np.array(goal)

    delta = [[cost, -1, 0],  # go up
             [cost, 0, -1],  # go left
             [cost, 1, 0],  # go down
             [cost, 0, 1]]  # go right

    delta_array = np.array(delta)
    visited_nodes = np.empty((0, len(goal)-1), int)
    possible_states = np.array([goal])

    value_grid = copy.deepcopy(grid)
    value = 0
    value_grid[goal[0]][goal[1]] = value

    while possible_states.size != 0:

        indices = np.argsort(possible_states[:, 2])

        # Sort A using these indices
        possible_states = possible_states[indices]

        # prev_state_x = possible_states[0][1]
        # prev_state_y = possible_states[0][2]

        for state in possible_states:

            if visited_nodes.size == 0 or not any((state[1:] == visited_nodes).all(1)):
                visited_nodes = np.append(visited_nodes, [state[1:]], axis=0)
            else:
                index = np.where((possible_states == state).all(axis=1))[0]
                possible_states = np.delete(possible_states, index, axis=0)
                continue

            for movement in delta_array:
                #acount for movements outside of grid
                new_position = state + movement
                if any((new_position[1:] == visited_nodes).all(1)):
                    continue
                elif new_position[1] < 0 or new_position[2] < 0:
                    continue
                elif new_position[1] > len(grid)-1 or new_position[2] > len(grid[0])-1:
                    continue
                elif grid_array[new_position[1]][new_position[2]] == 1:
                    value_grid[new_position[1]][new_position[2]] = 99
                    continue
                else:
                    value_grid[new_position[1]][new_position[2]] = new_position[0]
                    possible_states = np.append(possible_states, [new_position], axis=0)

    return value_grid


value = compute_value(grid, goal, cost)

value_grid = np.array(value)

for i in range(len(value)):
    print(value[i])

# --------------------------------------------------------------------
# 19. OPTIMUM POLICY
print("\n19. OPTIMUM POLICY")
# Write a function optimum_policy that returns
# a grid which shows the optimum policy for robot
# motion. This means there should be an optimum
# direction associated with each navigable cell from
# which the goal can be reached.
#
# Unnavigable cells as well as cells from which
# the goal cannot be reached should have a string
# containing a single space (' '), as shown in the
# previous video. The goal cell should have '*'.

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def compute_value(grid, goal, cost):

    goal = copy.deepcopy(goal)
    goal.insert(0,0)
    grid_array = np.array(grid)

    policy_grid = copy.deepcopy(grid)
    policy_grid = [[' ' for _ in row] for row in policy_grid]
    policy_grid[goal[1]][goal[2]] = '*'
    delta = [[cost, -1, 0],  # go up
             [cost, 0, -1],  # go left
             [cost, 1, 0],  # go down
             [cost, 0, 1]]  # go right

    delta_array = np.array(delta)
    visited_nodes = np.empty((0, len(goal)-1), int)
    possible_states = np.array([goal])

    value_grid = copy.deepcopy(grid)
    value = 0
    value_grid[goal[0]][goal[1]] = value

    while possible_states.size != 0:

        indices = np.argsort(possible_states[:, 2])

        # Sort A using these indices
        possible_states = possible_states[indices]

        for state in possible_states:

            if visited_nodes.size == 0 or not any((state[1:] == visited_nodes).all(1)):
                visited_nodes = np.append(visited_nodes, [state[1:]], axis=0)
            else:
                index = np.where((possible_states == state).all(axis=1))[0]
                possible_states = np.delete(possible_states, index, axis=0)
                continue

            for movement in delta_array:
                #acount for movements outside of grid
                new_position = state + movement
                if any((new_position[1:] == visited_nodes).all(1)):
                    continue
                elif new_position[1] < 0 or new_position[2] < 0:
                    continue
                elif new_position[1] > len(grid)-1 or new_position[2] > len(grid[0])-1:
                    continue
                elif grid_array[new_position[1]][new_position[2]] == 1:
                    value_grid[new_position[1]][new_position[2]] = 99
                    continue
                else:
                    value_grid[new_position[1]][new_position[2]] = new_position[0]
                    possible_states = np.append(possible_states, [new_position], axis=0)


                    if new_position[1] - state[1] > 0:
                        policy_grid[new_position[1]][new_position[2]] = "^"
                    elif new_position[1] - state[1] < 0:
                        policy_grid[new_position[1]][new_position[2]] = "v"
                    elif new_position[2] - state[2] < 0:
                        policy_grid[new_position[1]][new_position[2]] = ">"
                    elif new_position[2] - state[2] > 0:
                        policy_grid[new_position[1]][new_position[2]] = "<"

    return value_grid, policy_grid

value, policy = compute_value(grid, goal, cost)
for i in range(len(policy)):
    print(policy[i])

# --------------------------------------------------------------------
# 20. LEFT TURN POLICY
print("\n20. LEFT TURN POLICY")
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's
# optimal path to the position specified in goal;
# the costs for each motion are as defined in cost.

# grid format:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0]  # given in the form [row,col,direction]

goal = [2, 0]  # given in the form [row,col]

cost = [2, 1, 20]  # cost has 3 values, corresponding to making a right turn, no turn, and a left turn

# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a
# right turn.

forward = [[-1,  0],  # go up
            [0, -1],  # go left
            [1,  0],  # go down
            [0,  1]]  # go right

forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']


# def optimum_policy2D(grid,init,goal,cost):
#     value = [[[999 for col in range(len(grid[0]))] for row in range(len(grid))],
#              [[999 for col in range(len(grid[0]))] for row in range(len(grid))],
#              [[999 for col in range(len(grid[0]))] for row in range(len(grid))],
#              [[999 for col in range(len(grid[0]))] for row in range(len(grid))]]
#     # TODO: ADD CODE HERE
#     # Work backwards from goal
#     # find the Value function, which can then be used as Heuristics for the optimal path
#     # the four dimensions of the value function represent the directions
#
#     value_grid = np.array(value)
#     goal = copy.deepcopy(goal)
#
#     grid_array = np.array(grid)
#
#     policy_grid = copy.deepcopy(grid)
#     policy_grid = [[' ' for _ in row] for row in policy_grid]
#     policy_grid[goal[0]][goal[1]] = '*'
#
#     forward = [[-1, 0],  # go up
#                [0, -1],  # go left
#                [1, 0],  # go down
#                [0, 1]]  # go right
#
#     actions = [[2, -1],[1, 0],[20,1]]
#
#     visited_nodes = np.empty(( 0, (len(goal)+1)), int)
#
#     goal_array = np.array(goal)
#
#     initial_states = [np.append([x, 0], goal_array) for x in range(len(forward))]
#     possible_states = np.array(initial_states)
#
#     value = 0
#     value_grid[:, goal[0],goal[1]] = value
#
#     while possible_states.size != 0:
#
#         indices = np.argsort(possible_states[:, 0])
#
#         # Sort A using these indices
#         possible_states = possible_states[indices]
#
#  #TODO fix dimensions
#
#
#         for state in possible_states:
#
#             if visited_nodes.size == 0 or not any((state[:-1] == visited_nodes).all(1)):
#                 visited_nodes = np.append(visited_nodes, [state[:-1]], axis=0)
#             else:
#                 index = np.where((possible_states == state).all(axis=1))[0]
#                 possible_states = np.delete(possible_states, index, axis=0)
#                 continue
#
#             for idx, action in enumerate(actions):
#                 direction = (action[1] + state[1]) % len(forward)
#                 movement = np.append(direction,forward[direction])
#
#                 #acount for movements outside of grid
#                 new_position = state[1:] + np.append(movement, action[0])
#                 new_position[1] = new_position[1] % len(forward)
#                 new_position = np.append(action[1]+1, new_position) #append action
#
#                 if any((new_position[1:] == visited_nodes).all(1)):
#                     continue
#                 elif new_position[2] < 0 or new_position[3] < 0:
#                     continue
#                 elif new_position[2] > len(grid)-1 or new_position[3] > len(grid[0])-1:
#                     continue
#                 elif grid_array[new_position[2]][new_position[3]] == 1:
#                     value_grid[action[1]+1,direction, new_position[2],new_position[3]] = 999
#                     continue
#                 else:
#                     value_grid[action[1]+1,direction, new_position[2],new_position[3]] = new_position[0]
#                     possible_states = np.append(possible_states, [new_position], axis=0)
#
#
#     return value_grid
#
#
# policy2D = optimum_policy2D(grid, init, goal, cost)



# for i in range(len(policy2D)):
#     print(policy2D[i])
