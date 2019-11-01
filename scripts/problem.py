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
from cse571_project.srv import *
import collections

class State:
    """
    This class defines the state of the TurtleBot.

    """
    
    def __init__(self,x,y,orientation,battery):
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
        self.battery = battery

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.orientation == other.orientation and self.battery == other.battery:
            return True
        else:
            return False

    def __repr__(self):
        return "({}, {}, {}, {})".format(str(self.x), str(self.y), str(self.orientation), str(self.battery))



class Helper:
    """
    This class provides the methods used to control TurtleBot.
        
    Example:
        .. code-block:: python

            from problem import Helper

            h = Helper()
            init_state = h.get_init_state()

    """
    def get_successor(self,curr_state):
        rospy.wait_for_service('get_successor')
        try:
            get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
            response = get_successor(curr_state.x, curr_state.y, curr_state.orientation,curr_state.battery)
            states = collections.OrderedDict()
                      
            for i in range(len(response.action)):
                states[response.action[i]]=[State(response.x[i],response.y[i],response.orientation[i],response.battery[i]),response.g_cost[i]]

            return states
        except rospy.ServiceException, e:
            print "Service call failed get_successor: %s" % e


    def get_possible_actions(self,curr_state):
        rospy.wait_for_service('get_successor')
        try:
            get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
            response = get_successor(curr_state.x, curr_state.y, curr_state.orientation,curr_state.battery)
            states = collections.OrderedDict()
            
            print response
            '''
            for i in range(len(response.action)):
                print(response.action[i])
                print str(response.x[i]) + " " + str(response.y[i]) + " "+ str(response.orientation[i])+ " " + str(response.battery[i])
            '''
            return response.action
        
        except rospy.ServiceException, e:
            print "Service call failed get_possible_actions: %s" % e




    def get_initial_state(self):
        """
        This function calls get_initial_state service to recive the initial state of the turtlebot.

        return:  x_cord - initial x-cordinate of turtlebot           
                 y_cord - initial y-cordinate of turtlebot
                 direction - initial orientation
        """
        rospy.wait_for_service('get_initial_state')
        try:
            get_initial_state = rospy.ServiceProxy('get_initial_state', GetInitialState)
            response = get_initial_state()
            return response
        except rospy.ServiceException, e:
            print "Service call failed get_current_state: %s"%e


    def is_goal_state(self,state):
        rospy.wait_for_service('is_goal_state')
        try:
            is_goal_state = rospy.ServiceProxy('is_goal_state', IsGoalState)
            response = is_goal_state(state.x,state.y)
            return response.is_goal == 1
        except rospy.ServiceException, e:
            print "Service call failed is_terminal_state: %s"%e

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

        return ["TurnCW", "TurnCCW", "MoveF", "MoveB","REFUEL"]

    def get_goal_state(self):
        rospy.wait_for_service('get_goal_state')
        try:
            get_goal_state = rospy.ServiceProxy('get_goal_state',GetGoalState)
            response = get_goal_state()
            return response
        except rospy.ServiceException, e:
            print "Service call failed get_goal_state: %s" % e



    def usage(self):
        return "%s [x y]" % sys.argv[0]
