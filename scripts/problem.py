#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Chirav Dave", "Ketan Patil", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import sys
import rospy
from search.srv import *
import collections

class State:
    """
    This class defines the state of the TurtleBot.

    """
    
    def __init__(self,x,y,orientation):
        """
        :param x: current x-cordinate of turtlebot
        :type x: float
        :param y: current x-cordinate of turtlebot
        :type y: float   
        :param orientation: current orientation of turtlebot, can be either NORTH, SOUTH, EAST, WEST
        :type orientation: float

        """    
        self.x  = x 
        self.y = y
        self.orientation = orientation

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.orientation == other.orientation:
            return True
        else:
            return False

    def __repr__(self):
        return "({}, {}, {})".format(str(self.x), str(self.y), str(self.orientation))



class Helper:
    """
    This class provides the methods used to control TurtleBot.
        
    Example:
        .. code-block:: python

            from problem import Helper

            h = Helper()
            init_state = h.get_init_state()

      """
    '''
    def check_is_edge(self, edge,valueFlag):
        rospy.wait_for_service("check_is_edge")
        try:
    ''' 


    def execute_move_can_action(self, can_name, state):
        rospy.wait_for_service('execute_move_can_action')
        try:
        
            move_can_action = rospy.ServiceProxy('execute_move_can_action',MoveCan)
            response = move_can_action(can_name,state[0],state[1])

            if response.result == -1:
                return Fals
            elif response.result == 1:
                return True

        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def get_successor (self, curr_state):
        """
        This function calls get_successor service with current state as input and receives a dictionary as output. Possible actions are key to this dictionary and the value of each action is the state that can be reached by applying that action and the corresponding cost.

        :param curr_state: current state of the TurtleBot
        :type curr_state: State

        :returns: An **ordered** dictionary of actions {action_i : (state_i, cost_i)} such that applying action_i on curr_state results in state_i and it incurs a cost cost_i.

        :rtype: OrderedDict{str:(tuple(State,float))}

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                initial_state = h.get_initial_state()
                possible_successors_dict = h.get_successor(initial_state)
            
        .. warning::
            Process the output of this function in the same order it is returned. If you need to break ties between multiple nodes, use the tuple that appears earlier in the list returned by get_successor().

        """
        rospy.wait_for_service('get_successor')

        try:
            get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
            response = get_successor(curr_state.x, curr_state.y, curr_state.orientation)
            states = collections.OrderedDict()

            for i in range(6):  # Jie: That's total 6 successor state 
                states[response.action[i]] = (State(response.x[i], response.y[i], response.direction[i]), response.g_cost[i])
            return states
        
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_initial_state(self):
        """
        This function calls get_initial_state service to recive the initial state of the turtlebot.

        :returns: Initial state of the TurtleBot
        :rtype: State
                    
        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                initial_state = h.get_initial_state()

        """
        rospy.wait_for_service('get_initial_state')
        try:
            get_initial_state = rospy.ServiceProxy('get_initial_state', GetInitialState)
            response = get_initial_state()
            return State(response.x, response.y, response.direction)

        except rospy.ServiceException, e:
             print "Service call failed: %s" % e

    def is_goal_state(self, state):
        """
        This function calls is_goal_state service to check if the current state is the goal state or not.

        :param state: current state of the TurtleBot
        :type state: State

        :returns: 1 if input state is the goal state, 0 otherwise
        :rtype: bool

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                initial_state = h.get_initial_state()

                if h.is_goal_state(initial_state)
                    print "TurtleBot is already at Goal"

        """
        rospy.wait_for_service('is_goal_state')
        try:
            is_goal_state_client = rospy.ServiceProxy('is_goal_state', IsGoalState)
            response = is_goal_state_client(state.x, state.y)
            return response.is_goal
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_goal_state(self):
        """
        This function calls returns the goal state.

        :returns: Goal state
        :rtype: State

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                initial_state = h.get_initial_state()
                goal_state = h.get_goal_state()

                if initial_state == goal_state:
                    print "TurtleBot is already at Goal"

        .. note::
            The default orientation returnded is EAST. Any piece of code using this method should ignore the orientation to check for goal state.

        """
        rospy.wait_for_service('get_goal_state')
        try:
            get_goal_state = rospy.ServiceProxy('get_goal_state', GetGoalState)
            response = get_goal_state()
            return State(response.x, response.y, "EAST")
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_actions(self):
        """
        This function returns the list of all actions that a TurtleBot can perform.

        :returns: List of actions
        :rtype: list(str)

        Example:
            .. code-block:: python

                from problem import Helper

                h = Helper()
                possible_actions = h.get_actions()

        .. note::
            The code calling this API should check if the actions are applicable from the current state or not.

        """

        return ["TurnHalfCW", "TurnCW", "TurnHalfCCW", "TurnCCW", "MoveF", "MoveB"] # Jie turn 45 degree

    def usage(self):
        return "%s [x y]" % sys.argv[0]
