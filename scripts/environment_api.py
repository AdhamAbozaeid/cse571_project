#!/usr/bin/env python
import sys
import rospy
from cse571_project.srv import *
from std_msgs.msg import String
import json
import problem
import collections
rospy.init_node("environment_checker")

def get_current_state():
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


def is_terminal_state(state):
    rospy.wait_for_service('is_goal_state')
    try:
        is_goal_state = rospy.ServiceProxy('is_goal_state', IsGoalState)
        response = is_goal_state(state.x,state.y)
        return response.is_goal == 1
    except rospy.ServiceException, e:
        print "Service call failed is_terminal_state: %s"%e


def get_all_actions():
    """
    This function calls is_goal_state service to check if the current state is the goal state or not.

    parameters:  x_cord - current x-cordinate of turtlebot           return:   1 : if current state is the goal state
                 y_cord - current y-cordinate of turtlebot                     0 : if current state is not the goal state
    """
    rospy.wait_for_service('get_all_actions')
    try:
        all_actions = rospy.ServiceProxy('get_all_actions', GetActions)
        response = all_actions()
        return response.actions.split(",")
    except rospy.ServiceException, e:
        print "Service call failed get_all_actions: %s"%e


def get_possible_actions(curr_state):
    rospy.wait_for_service('get_successor')
    try:
        get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
        response = get_successor(curr_state.x, curr_state.y, curr_state.orientation,curr_state.battery)
        states = collections.OrderedDict()
        '''
        print response
        for i in range(len(response.action)):
            print(response.action[i])
            print str(response.x[i]) + " " + str(response.y[i]) + " "+ str(response.direction[i])+ " " + str(response.battery[i])
        '''
        return response.action
    
    except rospy.ServiceException, e:
        print "Service call failed get_possible_actions: %s" % e

def get_terminal_state():
    rospy.wait_for_service('get_goal_state')
    try:
        get_goal_state = rospy.ServiceProxy('get_goal_state',GetGoalState)
        response = get_goal_state()
        return response
    except rospy.ServiceException, e:
        print "Service call failed get_goal_state: %s" % e



def get_possible_states(curr_state, action):
    rospy.wait_for_service('get_successor')
    try:
        get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
        response = get_successor(curr_state.x, curr_state.y, curr_state.orientation,curr_state.battery)
        states = collections.OrderedDict()
        possible_state =[]
        if action not in response.action:
            print "Action not applicable in this state"
            return False
        for i in range(len(response.action)):
            if response.action[i] == action:
                return problem.State(response.x[i],response.y[i],response.direction[i],response.battery[i])

    except rospy.ServiceException, e:
            print "Service call failed get_possible_states: %s" % e


if __name__ == "__main__":
    state_to_check = problem.State(1.0,1.5,'EAST',5)
    action_to_check = 'REFUEL'
    
    initial_state = get_current_state()
    print "INITIAL STATE: " +str(initial_state)
    
    is_goal = is_terminal_state(state_to_check)
    print "IS TERMINAL STATE = " + str(is_goal)
    
    all_actions = get_all_actions()
    print "ALL ACTIONS: " + str(all_actions)


    possible_actions = get_possible_actions(state_to_check)
    print "POSSIBLE ACTIONS: " + str(possible_actions)

    
    goal_state = get_terminal_state()
    print "GOAL STATE: " + str(goal_state)

    possible_state = get_possible_states(state_to_check,action_to_check)
    print "POSSIBLE STATES: "+str(possible_state)