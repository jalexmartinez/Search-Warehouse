import numpy as np


def search(grid, init, goal, cost):
    # TODO: ADD CODE HERE
    init.insert(0, 0)
    grid_array = np.array(grid)
    goal_array = np.array(goal)

    delta = [[cost, -1, 0],  # go up
             [cost, 0, -1],  # go left
             [cost, 1, 0],  # go down
             [cost, 0, 1]]  # go right
    delta_array = np.array(delta)
    visited_nodes = np.empty((0, len(init) - 1), int)
    possible_states = np.array([init])
    expanded_grid = grid.copy()
    expansion = 0
    expanded_grid[init[0]][init[1]] = expansion

    while possible_states.size != 0:
        # Get the indices that would sort A by the third value in each row


        for state in possible_states:
            if visited_nodes.size == 0 or not any((state[1:] == visited_nodes).all(1)):
                visited_nodes = np.append(visited_nodes, [state[1:]], axis=0)

            elif np.array_equal(state[1:], goal_array):
                return list(state), list(expanded_grid)
            else:
                index = np.where((possible_states == state).all(axis=1))[0]
                possible_states = np.delete(possible_states, index, axis=0)
                continue
            for movement in delta_array:
                # acount for movements outside of grid
                new_position = state + movement
                if any((new_position[1:] == visited_nodes).all(1)):
                    continue
                elif new_position[1] < 0 or new_position[2] < 0:
                    continue
                elif new_position[1] > len(grid) - 1 or new_position[2] > len(grid[0]) - 1:
                    continue
                elif grid_array[new_position[1]][new_position[2]] == 1:
                    expanded_grid[new_position[1]][new_position[2]] = -1
                    continue
                else:
                    possible_states = np.append(possible_states, [new_position], axis=0)

    state = "fail"
    return state, list(expanded_grid)