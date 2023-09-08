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
import copy
import math
import collections
import heapq
import numpy as np

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class DeliveryPlanner_PartA:
    """
    Required methods in this class are:

      plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

      __init__: which is required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partA.py.  You may choose to use this starter code
        or modify and replace it based on your own solution.
        You should't change the signature, however.

    The following methods are starter code you may use for part A.
    However, they are not required and can be replaced with your
    own methods.

      _search(self, debug=False): Where the bulk of the A* search algorithm
          could reside.  It should find an optimal path from the robot
          location to a goal.
          Hint:  you may want to structure this based
          on whether looking for a box or delivering a box.

    """

    ## Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, robot_position, todo, box_locations):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self.warehouse_viewer = warehouse
        self.box_locations = box_locations
        self.dropzone = self.robot_position = robot_position

        self.delta = [[-1, 0],  # north
                      [0, -1],  # west
                      [1, 0],  # south
                      [0, 1],  # east
                      [-1, -1],  # northwest (diag)
                      [-1, 1],  # northeast (diag)
                      [1, 1],  # southeast (diag)
                      [1, -1]]  # southwest (diag)

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        # Can use this for a visual debug
        self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        # self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]

        self.delta_to_command = {
            (-1, 0): 'move n',
            (0, -1): 'move w',
            (1, 0): 'move s',
            (0, 1): 'move e',
            (-1, -1): 'move nw',
            (-1, 1): 'move ne',
            (1, 1): 'move se',
            (1, -1): 'move sw'
        }

        self.command_to_delta = {v: k for k, v in self.delta_to_command.items()}
        self.around_dropzone = [(self.dropzone[0] - 1, self.dropzone[1] + 1), (self.dropzone[0] - 1, self.dropzone[1]), (self.dropzone[0], self.dropzone[1] - 1), (self.dropzone[0] - 1, self.dropzone[1] - 1),
                 (self.dropzone[0] + 1, self.dropzone[1] - 1), (self.dropzone[0] + 1, self.dropzone[1]), (self.dropzone[0], self.dropzone[1] + 1), (self.dropzone[0] + 1, self.dropzone[1] + 1)]
    def heuristic(self, current, goal):
        dx = abs(current[0] - goal[0])+self.ORTHOGONAL_MOVE_COST
        dy = abs(current[1] - goal[1])+self.ORTHOGONAL_MOVE_COST
        heuristic_cost = max(dx, dy)
        return heuristic_cost

    def reconstruct_path(self, came_from, start, goal, box, action):
        current = goal
        path = []


        if len(came_from) == 1:

            delta = (self.robot_position[0] - start[0], self.robot_position[1] - start[1])
            if delta == (0,0):
                path.insert(0, f'lift {box}')
            else:
                path.insert(0, self.delta_to_command[delta])
                path.insert(0, f'lift {box}')


        else:
            path.insert(0, f'lift {box}')

            while current != start:
                previous = came_from[current]
                # Compute the difference (i.e., delta) between the current and previous cell
                delta = (current[0] - previous[0], current[1] - previous[1])
                # Map the delta to a move command and prepend it to the path
                path.insert(0, self.delta_to_command[delta])
                current = previous

        #self.robot_position = (current[0]+self.command_to_delta[path[0]][0], current[1]+self.command_to_delta[path[0]][1])
        return path

    def retrace_path(self, moves, box):
        reverse_actions = {
            'move n': 'move s',
            'move s': 'move n',
            'move e': 'move w',
            'move w': 'move e',
            'move ne': 'move sw',
            'move sw': 'move ne',
            'move se': 'move nw',
            'move nw': 'move se'
        }
        moves_retrace = copy.deepcopy(moves)
        moves_retrace.remove(f"lift {box}")
        path = [reverse_actions[move] for move in moves_retrace]
        path = path[::-1]
        old_robot_position = self.robot_position

        if self.robot_position not in self.around_dropzone:
            new_robot_position = self.robot_position

            for idx, move in enumerate(path):
                delta = self.command_to_delta[move]
                new_robot_position = (new_robot_position[0] + delta[0], new_robot_position[1] + delta[1])

                # Check if the new position is in the dropzone and is a valid position
                if new_robot_position in self.around_dropzone:
                    self.robot_position = new_robot_position
                    path = path[:idx+1]
                    break

        direction_goal = (self.dropzone[0]-self.robot_position[0], self.dropzone[1]-self.robot_position[1])
        heading = self.delta_to_command[direction_goal]
        if old_robot_position in self.around_dropzone:
            path = ["down" + heading[4:]]
        else:
            path.append("down" + heading[4:])
        return path

    def _search(self, goal, box, walls, debug=False):
        """
        This method should be based on lesson modules for A*, see Search, Section 12-14.
        The bulk of the search logic should reside here, should you choose to use this starter code.
        Please condition any printout on the debug flag provided in the argument.
        You may change this function signature (i.e. add arguments) as
        necessary, except for the debug argument which must remain with a default of False
        """

        cost_grid = collections.defaultdict(lambda: float('inf'))
        cost_grid[self.robot_position] = 0

        open_list = []
        heapq.heappush(open_list, (0, self.robot_position))

        came_from = {}
        came_from[self.robot_position] = None

        goals = [(goal[0] - 1, goal[1] + 1), (goal[0] - 1, goal[1]), (goal[0], goal[1] - 1), (goal[0] - 1, goal[1] - 1),
                 (goal[0] + 1, goal[1] - 1), (goal[0] + 1, goal[1]), (goal[0], goal[1] + 1), (goal[0] + 1, goal[1] + 1)]
        initial = self.robot_position
        while open_list:

            _, current = heapq.heappop(open_list)


            for i, direction in enumerate(self.delta):
                if current in goals:
                    if self.robot_position == self.dropzone:
                        self.robot_position = goal
                        prelim_goal = goal
                    else:

                        prelim_goal = goal

                    self.warehouse_viewer[goal[0]][goal[1]] = '.'
                    open_list = []
                    break
                next_node = (current[0] + direction[0], current[1] + direction[1])
                if (next_node[0], next_node[1]) in walls:
                    continue

                elif self.warehouse_viewer[next_node[0]][next_node[1]] != ".":

                    if self.warehouse_viewer[next_node[0]][next_node[1]] == "#":
                        walls.append((next_node[0], next_node[1]))
                        continue
                    if self.warehouse_viewer[next_node[0]][next_node[1]].isalnum():
                        if next_node not in goals:
                            walls.append((next_node[0], next_node[1]))
                            continue

                if next_node in goals:
                    came_from[next_node] = current
                    prelim_goal = next_node
                    open_list =[]
                    self.robot_position = next_node
                    self.warehouse_viewer[goal[0]][goal[1]] = '.'
                    break

                new_cost = cost_grid[current] + self.delta_cost[i] + self.heuristic(next_node, goal)
                if new_cost < cost_grid[next_node]:
                    cost_grid[next_node] = new_cost
                    priority = new_cost
                    heapq.heappush(open_list, (priority, next_node))
                    came_from[next_node] = current


        moves_to = self.reconstruct_path(came_from, initial, prelim_goal, box, action="Pick Up")
        moves_from = self.retrace_path(moves_to, box)
        moves_list = [moves_to, moves_from]
        moves = [move for direction in moves_list for move in direction]

        return moves, walls

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the moves.  You may use the starter code provided
        in any way you choose, but please condition any printouts on the debug flag
        """
        walls = []
        total_moves = []
        for box in list(self.box_locations.keys()):
            moves = self._search(self.box_locations[box], box, walls)
            total_moves.append(moves[0])
        final_moves = [move for task in total_moves for move in task]
        to_dropzone = "move" + final_moves[-1][4:]
        final_moves.append(to_dropzone)

        if debug:
            for i in range(len(total_moves)):
                print(total_moves[i])

        return final_moves

class DeliveryPlanner_PartB:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partB.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part B.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, goal, pickup_box=True, debug=False): Where the bulk
            of the dynamic programming (DP) search algorithm could reside.
            It should find an optimal path from the robot location to a goal.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost

        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # up left (diag)
                      [-1, 1],  # up right (diag)
                      [1, 1],  # dn right (diag)
                      [1, -1]]  # dn left (diag)

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        # Use this for a visual debug
        self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        # self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]
        self.delta_to_command = {
            (-1, 0): 'move n',
            (0, -1): 'move w',
            (1, 0): 'move s',
            (0, 1): 'move e',
            (-1, -1): 'move nw',
            (-1, 1): 'move ne',
            (1, 1): 'move se',
            (1, -1): 'move sw'
        }

        self.reverse_actions = {
            'move n': 'move s',
            'move s': 'move n',
            'move e': 'move w',
            'move w': 'move e',
            'move ne': 'move sw',
            'move sw': 'move ne',
            'move se': 'move nw',
            'move nw': 'move se'
        }

        self.command_to_delta = {v: k for k, v in self.delta_to_command.items()}
        self.around_dropzone = [(self.dropzone[0] - 1, self.dropzone[1] + 1), (self.dropzone[0] - 1, self.dropzone[1]),
                                (self.dropzone[0], self.dropzone[1] - 1), (self.dropzone[0] - 1, self.dropzone[1] - 1),
                                (self.dropzone[0] + 1, self.dropzone[1] - 1), (self.dropzone[0] + 1, self.dropzone[1]),
                                (self.dropzone[0], self.dropzone[1] + 1), (self.dropzone[0] + 1, self.dropzone[1] + 1)]
    # state parsing and initialization function
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def _find_policy(self, goal, pickup_box=True, debug=False):
        """
        This method should be based on lesson modules for Dynamic Programming,
        see Search, Section 15-19 and Problem Set 4, Question 5.  The bulk of
        the logic for finding the policy should reside here should you choose to
        use this starter code.  Please condition any printout on the debug flag
        provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """
        goals = [(goal[0] - 1, goal[1] + 1), (goal[0] - 1, goal[1]), (goal[0], goal[1] - 1), (goal[0] - 1, goal[1] - 1),
                 (goal[0] + 1, goal[1] - 1), (goal[0] + 1, goal[1]), (goal[0], goal[1] + 1), (goal[0] + 1, goal[1] + 1)]

        goal = np.array(copy.deepcopy(goal))
        goal = np.insert(goal,0,self.warehouse_cost[goal[0]][goal[1]])
        grid_array = np.array(self.warehouse_cost)

        policy_grid = copy.deepcopy(self.warehouse_cost)
        policy_grid = [[' ' for _ in row] for row in policy_grid]
        value_grid = [[999999 for _ in row] for row in policy_grid]
        if pickup_box == True:
            policy_grid[goal[1]][goal[2]] = '1'
        else:
            policy_grid[goal[1]][goal[2]] = '@'

        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # up left (diag)
                      [-1, 1],  # up right (diag)
                      [1, 1],  # dn right (diag)
                      [1, -1]]  # dn left (diag)

        self.cost_delta = [np.insert(self.delta[idx],0, self.delta_cost[idx]) for idx,x in enumerate(self.delta)]

        delta_array = np.array(self.cost_delta)
        visited_nodes = np.empty((0, len(goal) - 1), int)
        possible_states = np.array([goal])

        walls = []
        while possible_states.size != 0:

            indices = np.argsort(possible_states[:, 2])

            # Sort A using these indices
            possible_states = possible_states[indices]
            possible_states = sorted(possible_states, key=lambda state: state[0])

            for state in possible_states:

                if visited_nodes.size == 0 or not any((state[1:] == visited_nodes).all(1)):
                    visited_nodes = np.append(visited_nodes, [state[1:]], axis=0)
                else:
                    index = np.where((possible_states == state).all(axis=1))[0]
                    possible_states = np.delete(possible_states, index, axis=0)
                    continue

                for movement in delta_array:
                    # account for movements outside of grid
                    new_position = state + movement

                    if any((new_position[1:] == visited_nodes).all(1)):
                        continue
                    elif new_position[1] < 0 or new_position[2] < 0:
                        continue
                    elif new_position[1] > len(grid_array) - 1 or new_position[2] > len(grid_array[0]) - 1:
                        continue
                    elif self.warehouse_state[new_position[1]][new_position[2]] == '#':
                        value_grid[new_position[1]][new_position[2]] = 999
                        policy_grid[new_position[1]][new_position[2]] = -1
                        if (new_position[1], new_position[2]) not in walls:
                            walls.append((new_position[1], new_position[2]))
                        continue

                    else:
                        new_position[0] += self.warehouse_cost[new_position[1]][new_position[2]]
                        if value_grid[new_position[1]][new_position[2]] > new_position[0]:

                            value_grid[new_position[1]][new_position[2]] = new_position[0]

                            possible_states = np.append(possible_states, [new_position], axis=0)

                            direction = self.delta_to_command[(movement[1],movement[2])]
                            reverse = self.reverse_actions[direction]
                            policy_grid[new_position[1]][new_position[2]] = reverse

        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])

        tuple_list = [(x,y) for x in range(rows) for y in range(cols)]

        if pickup_box == True:
            for cell in goals:
                if cell in tuple_list and cell not in walls:
                    policy_grid[cell[0]][cell[1]] = 'lift 1'
        else:
            value = 999999
            for cell in goals:
                if cell in tuple_list and cell not in walls:
                    direction = (self.dropzone[0] - cell[0], self.dropzone[1] - cell[1])
                    direction_command = self.delta_to_command[direction]
                    policy_grid[cell[0]][cell[1]] = 'down' + direction_command[4:]
                    if value_grid[cell[0]][cell[1]] < value:
                        value = value_grid[cell[0]][cell[1]]
                        direction_dropzone = self.reverse_actions[direction_command]
                        policy_grid[goal[1]][goal[2]] = direction_dropzone

        test = 90
        test = 69

        return policy_grid

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """

        goal = self.boxes['1']
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)


        goal = self.dropzone
        deliver_policy = self._find_policy(goal, pickup_box=False, debug=debug)


        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(deliver_policy)):
                print(deliver_policy[i])

        return (to_box_policy, deliver_policy)


class DeliveryPlanner_PartC:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partC.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part C.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, goal, pickup_box=True, debug=False):
            Where the bulk of your algorithm could reside.
            It should find an optimal policy to a goal.
            Remember that actions are stochastic rather than deterministic.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):

        self.todo = todo
        self.boxes_delivered = []
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.stochastic_probabilities = stochastic_probabilities

        self.delta = [
            [-1, 0],  # go up
            [-1, -1],  # up left (diag)
            [0, -1],  # go left
            [1, -1],  # dn left (diag)
            [1, 0],  # go down
            [1, 1],  # dn right (diag)
            [0, 1],  # go right
            [-1, 1],  # up right (diag)]
        ]

        self.delta_directions = ["n", "nw", "w", "sw", "s", "se", "e", "ne"]

        # Use this for a visual debug
        self.delta_name = ['🡑', '🡔', '🡐', '🡗', '🡓', '🡖', '🡒', '🡕']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST, ]

        self.delta_to_command = {
            (-1, 0): 'move n',
            (0, -1): 'move w',
            (1, 0): 'move s',
            (0, 1): 'move e',
            (-1, -1): 'move nw',
            (-1, 1): 'move ne',
            (1, 1): 'move se',
            (1, -1): 'move sw'
        }

        self.reverse_actions = {
            'move n': 'move s',
            'move s': 'move n',
            'move e': 'move w',
            'move w': 'move e',
            'move ne': 'move sw',
            'move sw': 'move ne',
            'move se': 'move nw',
            'move nw': 'move se'
        }

        self.command_to_delta = {v: k for k, v in self.delta_to_command.items()}
        self.around_dropzone = [(self.dropzone[0] - 1, self.dropzone[1] + 1), (self.dropzone[0] - 1, self.dropzone[1]),
                                (self.dropzone[0], self.dropzone[1] - 1), (self.dropzone[0] - 1, self.dropzone[1] - 1),
                                (self.dropzone[0] + 1, self.dropzone[1] - 1), (self.dropzone[0] + 1, self.dropzone[1]),
                                (self.dropzone[0], self.dropzone[1] + 1), (self.dropzone[0] + 1, self.dropzone[1] + 1)]

    # state parsing and initialization function

    # state parsing and initialization function
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def get_surrounding_positions(self, position, direction, type):
        # Calculate the side and diagonal cells' positions.
        y, x = position
        dy, dx = direction

        if type > 3:  # For diagonal directions
            diagonal1 = (y + dy, x)  # One side cell
            diagonal2 = (y, x + dx)  # Other side cell
            side1 = (y + dy, x + dx)  # One diagonal cell
            side2 = (y - dy, x - dx)  # Other diagonal cell
        else:  # For cardinal directions
            side1 = (y - dx, x + dy)  # One side cell
            side2 = (y + dx, x - dy)  # Other side cell
            diagonal1 = (y + dy - dx, x + dx + dy)  # One diagonal cell
            diagonal2 = (y + dy + dx, x + dx - dy)  # Other diagonal cell

        return side1, side2, diagonal1, diagonal2

    def _find_policy(self, goal, pickup_box=True, debug=False):
        """
        You are free to use any algorithm necessary to complete this task.
        Some algorithms may be more well suited than others, but deciding on the
        algorithm will allow you to think about the problem and understand what
        tools are (in)adequate to solve it. Please condition any printout on the
        debug flag provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """
        goals = [(goal[0] - 1, goal[1] + 1), (goal[0] - 1, goal[1]), (goal[0], goal[1] - 1), (goal[0] - 1, goal[1] - 1),
                 (goal[0] + 1, goal[1] - 1), (goal[0] + 1, goal[1]), (goal[0], goal[1] + 1), (goal[0] + 1, goal[1] + 1)]

        goal = np.array(copy.deepcopy(goal))
        goal = np.insert(goal, 0, self.warehouse_cost[goal[0]][goal[1]])
        grid_array = np.array(self.warehouse_cost)

        policy_grid = copy.deepcopy(self.warehouse_cost)
        policy_grid = [[' ' for _ in row] for row in policy_grid]
        value_grid_list = [[10000 for _ in row] for row in policy_grid]
        value_grid = np.array(value_grid_list)

        if pickup_box == True:
            policy_grid[goal[1]][goal[2]] = '1'
        else:
            policy_grid[goal[1]][goal[2]] = '@'

        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # up left (diag)
                      [-1, 1],  # up right (diag)
                      [1, 1],  # dn right (diag)
                      [1, -1]]  # dn left (diag)
        self.delta_ortho = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1]]  # go right
        self.delta_diag = [[-1, -1],  # up left (diag)
                      [-1, 1],  # up right (diag)
                      [1, 1],  # dn right (diag)
                      [1, -1]]  # dn left (diag)


        self.cost_delta = [np.insert(self.delta[idx], 0, self.delta_cost[idx]) for idx, x in enumerate(self.delta)]

        delta_array = np.array(self.cost_delta)
        visited_nodes = np.empty((0, len(goal) - 1), int)
        possible_states = np.array([goal])

        walls = []
        value_grid[goal[1]][goal[2]] = -1
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
                    elif new_position[1] > len(grid_array) - 1 or new_position[2] > len(grid_array[0]) - 1:
                        continue
                    elif self.warehouse_state[new_position[1]][new_position[2]] == '#':
                        value_grid[new_position[1]][new_position[2]] = self.ILLEGAL_MOVE_PENALTY
                        policy_grid[new_position[1]][new_position[2]] = -1
                        if (new_position[1], new_position[2]) not in walls:
                            walls.append((new_position[1], new_position[2]))
                        continue
                    elif (new_position[1], new_position[2]) in goals:
                        value_grid[new_position[1]][new_position[2]] = 0
                        policy_grid[new_position[1]][new_position[2]] = 'lift 1'
                        if any((new_position[1:] == x[1:]).all() for x in possible_states):
                            continue
                        possible_states = np.append(possible_states, [new_position], axis=0)
                        continue
                    elif (new_position[1], new_position[2]) == -1:
                        continue
                    else:

                        if any(state[1:] != goal[1:]):

                            left_loc, right_loc, slantL_loc, slantR_loc = self.get_surrounding_positions((state[1],state[2]), (movement[1], movement[2]), idx)
                            if idx < 4:
                                move_cost_side = self.ORTHOGONAL_MOVE_COST
                                move_cost_diagonal = self.DIAGONAL_MOVE_COST
                            else:
                                move_cost_diagonal = self.ORTHOGONAL_MOVE_COST
                                move_cost_side = self.DIAGONAL_MOVE_COST

                            #test = 69
                            if right_loc[0] < 0 or right_loc[1] < 0:
                                right_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            elif right_loc[0] > len(grid_array) - 1 or right_loc[1] > len(grid_array[0]) - 1:
                                right_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]]
                            elif self.warehouse_state[right_loc[0]][right_loc[1]] == "#":
                                right_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            else:
                                right_cost = self.warehouse_cost[right_loc[0]][right_loc[1]] + value_grid[state[1], state[2]] \
                                            + move_cost_side

                            if left_loc[0] < 0 or left_loc[1] < 0:
                                left_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            elif left_loc[0] > len(grid_array) - 1 or left_loc[1] > len(grid_array[0]) - 1:
                                left_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            elif self.warehouse_state[left_loc[0]][left_loc[1]] == "#":
                                left_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            else:
                                left_cost =  self.warehouse_cost[left_loc[0]][ left_loc[1]] + value_grid[state[1]][state[2]] \
                                             + move_cost_side

                            if slantR_loc[0] < 0 or slantR_loc[1] < 0:
                                slantR_cost  = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            elif slantR_loc[0] > len(grid_array) - 1 or slantR_loc[1] > len(grid_array[0]) - 1:
                                slantR_cost  = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            elif self.warehouse_state[slantR_loc[0]][slantR_loc[1]] == "#":
                                slantR_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            else:
                                slantR_cost  = self.warehouse_cost[slantR_loc[0]][slantR_loc[1]] + value_grid[state[1]][state[2]] \
                                                + move_cost_diagonal

                            if slantL_loc[0] < 0 or slantL_loc[1] < 0:
                                slantL_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            elif slantL_loc[0] > len(grid_array) - 1 or slantL_loc[1] > len(grid_array[0]) - 1:
                                slantL_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            elif self.warehouse_state[slantL_loc[0]][slantL_loc[1]] == "#":
                                slantL_cost = self.ILLEGAL_MOVE_PENALTY + self.warehouse_cost[state[1]][state[2]] + movement[0]
                            else:
                                slantL_cost = self.warehouse_cost[slantL_loc[0]][slantL_loc[1]] + value_grid[state[1]][state[2]] \
                                              + move_cost_diagonal

                            if idx < 4:
                                new_cost = (self.stochastic_probabilities["as_intended"] * (new_position[0] + self.warehouse_cost[new_position[1]][new_position[2]])) + \
                                           (self.stochastic_probabilities["sideways"] * right_cost) + (self.stochastic_probabilities["sideways"] * \
                                            left_cost ) + (self.stochastic_probabilities["slanted"] * slantR_cost) + (self.stochastic_probabilities["slanted"] * \
                                            slantL_cost)
                            else:
                                new_cost = (self.stochastic_probabilities["as_intended"] * (new_position[0] + self.warehouse_cost[new_position[1]][new_position[2]])) + \
                                           (self.stochastic_probabilities["slanted"] * right_cost) + (self.stochastic_probabilities["slanted"] * \
                                            left_cost ) + (self.stochastic_probabilities["sideways"] * slantR_cost) + (self.stochastic_probabilities["sideways"] * \
                                            slantL_cost)

                            if new_cost < value_grid[new_position[1]][new_position[2]]: #maybe try state
                                new_position[0] = new_cost
                                value_grid[new_position[1]][new_position[2]] = new_position[0]
                                possible_states = np.append(possible_states, [new_position], axis=0)
                                direction = self.delta_to_command[(movement[1], movement[2])]
                                reverse = self.reverse_actions[direction]
                                policy_grid[new_position[1]][new_position[2]] = reverse

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

        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])

        tuple_list = [(x, y) for x in range(rows) for y in range(cols)]

        if pickup_box == True:
            for cell in goals:
                if cell in tuple_list and cell not in walls:
                    policy_grid[cell[0]][cell[1]] = 'lift 1'
        else:
            value = 999999
            for cell in goals:
                if cell in tuple_list and cell not in walls:
                    direction = (self.dropzone[0] - cell[0], self.dropzone[1] - cell[1])
                    direction_command = self.delta_to_command[direction]
                    policy_grid[cell[0]][cell[1]] = 'down' + direction_command[4:]
                    if value_grid[cell[0]][cell[1]] < value:
                        value = value_grid[cell[0]][cell[1]]
                        direction_dropzone = self.reverse_actions[direction_command]
                        policy_grid[goal[1]][goal[2]] = direction_dropzone

        return policy_grid, value_grid

    def plan_delivery(self, debug=True):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """

        goal = self.boxes['1']
        to_box_policy, to_box_value = self._find_policy(goal, pickup_box=True, debug=debug)

        goal = self.dropzone
        to_zone_policy, to_zone_value = self._find_policy(goal, pickup_box=False, debug=debug)

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nTo Zone Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        test = 69

        return (to_box_policy, to_zone_policy, list(to_box_value), list(to_zone_value))


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'jmartinez348'
    return whoami


if __name__ == "__main__":
    """ 
    You may execute this file to develop and test the search algorithm prior to running 
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will NOT be called by the autograder
    # This section is just a provided as a convenience to help in your development/debugging process

    # Testing for Part A
    # testcase 1
    print('\nTesting for part A:')

    from testing_suite_partA import wrap_warehouse_object, Counter

    # test case data starts here
    warehouse = [
        '######',
        '#....#',
        '#.1#2#',
        '#..#.#',
        '#...@#',
        '######',
    ]
    todo = list('12')
    benchmark_cost = 23
    viewed_cell_count_threshold = 20
    robot_position = (4, 4)
    box_locations = {
        '1': (2, 2),
        '2': (2, 4),
    }
    # test case data ends here

    viewed_cells = Counter()
    warehouse_access = wrap_warehouse_object(warehouse, viewed_cells)
    partA = DeliveryPlanner_PartA(warehouse_access, robot_position, todo, box_locations)
    partA.plan_delivery(debug=True)
    # Note that the viewed cells for the hard coded solution provided
    # in the initial template code will be 0 because no actual search
    # process took place that accessed the warehouse
    print('Viewed Cells:', len(viewed_cells))
    print('Viewed Cell Count Threshold:', viewed_cell_count_threshold)

    # Testing for Part B
    # testcase 1
    print('\nTesting for part B:')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[3, 5, 2],
                      [10, math.inf, 2],
                      [2, 10, 2]]

    todo = ['1']

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    partB.plan_delivery(debug=True)

    # Testing for Part C
    # testcase 1
    print('\nTesting for part C:')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[13, 5, 6],
                      [10, math.inf, 2],
                      [2, 11, 2]]

    todo = ['1']

    stochastic_probabilities = {
        'as_intended': .70,
        'slanted': .1,
        'sideways': .05,
    }

    partC = DeliveryPlanner_PartC(warehouse, warehouse_cost, todo, stochastic_probabilities)
    partC.plan_delivery(debug=True)
