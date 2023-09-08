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

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) * self.ORTHOGONAL_MOVE_COST

    def reconstruct_path(self, came_from, start, goal, action, box=None):
        current = goal
        path = []
        action = action

        # Create a mapping from delta to move commands
        delta_to_command = {
            (-1, 0): 'move n',
            (0, -1): 'move w',
            (1, 0): 'move s',
            (0, 1): 'move e',
            (-1, -1): 'move nw',
            (-1, 1): 'move ne',
            (1, 1): 'move se',
            (1, -1): 'move sw'
        }

        while current != start:
            previous = came_from[current]
            # Compute the difference (i.e., delta) between the current and previous cell
            delta = (current[0] - previous[0], current[1] - previous[1])
            # Map the delta to a move command and prepend it to the path
            path.insert(0, delta_to_command[delta])
            current = previous
            last_key = current
        # if action == "Pick Up":
        #     path_final = path[:-1]
        #     path_final.append(f'lift {box}')
        # else:
        #     direction = (goal[0] - last_key[0], goal[1] - last_key[1])
        #     command = delta_to_command[direction]
        #     command[0:4] = 'down'
        #     path.append(command)
        #     path_final = path

        return path

    def retrace_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)  # optional
        path.reverse()  # optional
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

        while open_list:
            _, current = heapq.heappop(open_list)

            if current in goals:
                came_from[goal] = current
                break

            for i, direction in enumerate(self.delta):
                next_node = (current[0] + direction[0], current[1] + direction[1])
                if (next_node[0], next_node[1]) in walls:
                    continue
                if self.warehouse_viewer[next_node[0]][next_node[1]] == ".":
                    new_cost = cost_grid[current] + self.delta_cost[i] + self.heuristic(next_node, goal)
                    if new_cost < cost_grid[next_node]:
                        cost_grid[next_node] = new_cost
                        priority = new_cost
                        heapq.heappush(open_list, (priority, next_node))
                        came_from[next_node] = current
                elif self.warehouse_viewer[next_node[0]][next_node[1]] == "#":
                    walls.append((next_node[0], next_node[1]))

        moves_to = self.reconstruct_path(came_from, self.robot_position, goal, box)
        moves_from = self.retrace_path(came_from, current, self.dropzone)
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
            total_moves.append(moves)

        if debug:
            for i in range(len(total_moves)):
                print(total_moves[i])

        return total_moves


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

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################

        # get a shortcut variable for the warehouse (note this is just a view it does not make a copy)
        grid = self.warehouse_state
        grid_costs = self.warehouse_cost

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1
        if pickup_box:
            # To box policy
            policy = [['B', 'lift 1', 'move w'],
                      ['lift 1', '-1', 'move nw'],
                      ['move n', 'move nw', 'move n']]

        else:
            # Deliver policy
            policy = [['move e', 'move se', 'move s'],
                      ['move ne', '-1', 'down s'],
                      ['move e', 'down e', 'move n']]

        return policy

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################

        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        goal = self.boxes['1']
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
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
        You are free to use any algorithm necessary to complete this task.
        Some algorithms may be more well suited than others, but deciding on the
        algorithm will allow you to think about the problem and understand what
        tools are (in)adequate to solve it. Please condition any printout on the
        debug flag provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################

        # get a shortcut variable for the warehouse (note this is just a view it does not make a copy)
        grid = self.warehouse_state
        grid_costs = self.warehouse_cost

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1
        if pickup_box:
            # To-box policy
            # the below policy is hard coded to work for test case 1
            policy = [
                ['B', 'lift 1', 'move w'],
                ['lift 1', -1, 'move nw'],
                ['move n', 'move nw', 'move n'],
            ]

        else:
            # to-zone policy
            # the below policy is hard coded to work for test case 1
            policy = [
                ['move e', 'move se', 'move s'],
                ['move se', -1, 'down s'],
                ['move e', 'down e', 'move n'],
            ]

        return policy

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################

        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        goal = self.boxes['1']
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
        goal = self.dropzone
        to_zone_policy = self._find_policy(goal, pickup_box=False, debug=debug)

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nTo Zone Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        to_box_values = None
        to_zone_values = None
        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)


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
