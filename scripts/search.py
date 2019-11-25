#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
from collections import deque
import math
import time 
import numpy as np
import pprint

rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Possible arguments = {bfs, ucs, gbfs, astar}. Default value is bfs.", metavar='bfs', action='store', dest='algorithm', default="bfs", type=str)
parser.add_argument('-c', help="Use custom heuristic function", action='store_true', dest='custom_heuristic')
parser.add_argument('-d',help = "Use debug action sequence",action = 'store_true',dest = 'debug_mode')
parser.add_argument('-s',help = "print states",action='store',default=1,type=int,dest = 'print_states')
pp = pprint.PrettyPrinter(indent=4)

def states_from_action_list(action_list):
    helper = problem.Helper()
    state = helper.get_initial_state()
    states = []
    for action in action_list:
        states.append(state)
        sucs = helper.get_successor(state)
        state,child_cost = sucs[action]
    pp.pprint(states)

def manhattan(state1,state2):
    return abs(state1.x-state2.x)+abs(state1.y-state2.y)

def custom_heuristic2(state,goal):
    # use square of euclidian distance for efficiency
    eucl_dist_sqr = math.sqrt(((state.x - goal.x)**2) + ((state.y - goal.y)**2))

    theta = math.degrees(math.atan2(abs(state.y-goal.y), abs(state.x-goal.x)))
    if theta > 0 and theta <= 45:
        eucl_dist_sqr += 2
    elif theta > 45 and theta <= 90:
        eucl_dist_sqr += 4

    # Penalize if the battery isn't enough to reach the goal
    # Number of steps is at minimum the euclidian distance / 0.5sqrt(2)
    no_steps_sqr = eucl_dist_sqr/0.707
    if no_steps_sqr > (state.battery):
        return eucl_dist_sqr + 1000
    return eucl_dist_sqr

def bfs(use_custom_heuristic,use_debug_mode):

    global debug_sequence
    if use_debug_mode:
        return debug_sequence


    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []
    counter = 0 #For breaking ties

    state_dictionary = helper.get_successor(init_state)
    if helper.is_goal_state(init_state):
        print "Found Goal"
        return action_list

    root = init_state
    
    frontier = deque()
    actions_frontier = deque()
    
    frontier.append(root)
    actions_frontier.append([])

    explored = set()
    counter+=1
    while True:
        if len(frontier)==0:
            print "No path possible!"
            return action_list
        
        node = frontier.popleft()
        node_actions = actions_frontier.popleft()

        explored.add(str(node))
        sucs = helper.get_successor(node)

        for action in sucs:
            counter+=1
            actions = node_actions[:]
            child_node,child_cost = sucs[action]
            if ((str(child_node) not in explored) and (child_node not in frontier)):# and (child_node not in frontier):
                actions.append(action)
                if helper.is_goal_state(child_node):
                    print "Found goal! : "+str(child_node)
                    return actions
                    
                frontier.append(child_node)
                actions_frontier.append(actions)

    return action_list

def findnode(q,item):
    for i in range(len(q)):
        if q[i][-1] == item:
            return i
    return -1

def ucs(use_custom_heuristic,use_debug_mode):
    '''
    Perform UCS to find sequence of actions from initial state to goal state
    '''

    global debug_mode
    global debug_sequence
    if use_debug_mode:
        return debug_sequence


    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []
    counter = 0 #For breaking ties

    # to get the possible action->(state,cost) mapping from current state
    if helper.is_goal_state(init_state):
        print "Found Goal"
        return action_list

    '''
    Use priority queue to keep track of path and Path Cost
    Each item in frontier is of the form[Cumulative Cost,counter,action1,action2,action3......actionN,(LastNode)]
    '''
    root = init_state
    frontier = []
    explored = set()
    
    item = [0.0,counter,[],root]
    heapq.heappush(frontier,item)
    counter+=1
    
    while True:
        if len(frontier)==0:
            print "No path possible!"
            return action_list
        
        item = heapq.heappop(frontier)
        node = item.pop()
        node_actions = item.pop()
        node_cum_cost = item.pop(0)
        if helper.is_goal_state(node):
            return node_actions


        explored.add(str(node))
        sucs = helper.get_successor(node)

        for action in sucs:
            counter+=1
            actions = node_actions[:]
            child_node,child_cost = sucs[action]
            child_cost+=node_cum_cost
            
            if (str(child_node) not in explored):# and (child_node not in frontier):
                index = findnode(frontier,child_node)
                if index!=-1:
                    if frontier[index][0]>child_cost:
                        del frontier[index]
                actions.append(action)
                item = [child_cost,counter,actions,child_node]
                heapq.heappush(frontier,item)

    return action_list

def gbfs(use_custom_heuristic,use_debug_mode):
    '''
    Perform A* search to find sequence of actions from initial state to goal state
    '''
    global debug_mode
    global debug_sequence
    if use_debug_mode:
        return debug_sequence



    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []
    counter = 0
    state_dictionary = helper.get_successor(init_state)
    if helper.is_goal_state(init_state):
        print "Found Goal"
        return action_list

    '''
    Use priority queue to keep track of path and Path Cost
    Each item in frontier is of the form[Heuristic,counter,cumulative_cost,action1,action2,action3......actionN,(LastNode)]
    '''
    root = init_state
    frontier = []
    explored = set()

    if use_custom_heuristic:
        heuristic = custom_heuristic2(root,goal_state)
    else:
        heuristic = manhattan(root,goal_state)

    item = [heuristic,counter,[],root]
    heapq.heappush(frontier,item)
    counter+=1

    while True:
        if len(frontier)==0:
            print "No path possible!"
            return action_list
        
        item = heapq.heappop(frontier)
        node = item.pop()
        node_actions = item.pop()
        
        if helper.is_goal_state(node):
            return node_actions

        explored.add(str(node))
        sucs = helper.get_successor(node)
        
        for action in sucs:
            counter+=1
            actions = node_actions[:]
            child_node,child_cost = sucs[action]
            
            if use_custom_heuristic:
                child_heuristic = custom_heuristic2(child_node,goal_state)
            else:   
                child_heuristic = manhattan(child_node,goal_state)

            if (str(child_node) not in explored):
                actions.append(action)
                item = [child_heuristic,counter,actions,child_node]
                heapq.heappush(frontier,item)

    return action_list
def astar(use_custom_heuristic,use_debug_mode):
    '''
    Perform A* search to find sequence of actions from initial state to goal state
    '''
    global debug_mode
    global debug_sequence
    if use_debug_mode:
        return debug_sequence



    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []
    counter = 0
    
    if helper.is_goal_state(init_state):
        print "Found Goal"
        return action_list

    '''
    Use priority queue to keep track of path and Path Cost
    '''
    root = init_state
    frontier = []
    explored = set()
    
    if use_custom_heuristic:
        heuristic = custom_heuristic2(root,goal_state)
    else:
        heuristic = manhattan(root,goal_state)

    node_cum_cost = 0.0
    item = [heuristic,counter,node_cum_cost,[],root]
    heapq.heappush(frontier,item)
    counter+=1
    
    while True:
        if len(frontier)==0:
            print "No path possible!"
            return action_list
    
        item = heapq.heappop(frontier)
        node = item.pop()
        node_actions = item.pop()
        node_cum_cost = item.pop()
        if helper.is_goal_state(node):
            return node_actions

        explored.add(str(node))
        sucs = helper.get_successor(node)
        for action in sucs:
            counter+=1
            actions = node_actions[:]
            child_node,child_cost = sucs[action]
            child_cost+=node_cum_cost
            
            if use_custom_heuristic:
                child_heuristic = custom_heuristic2(child_node,goal_state)
            else:
                child_heuristic = manhattan(child_node,goal_state)

            if (str(child_node) not in explored):
                actions.append(action)
                total_cost = child_heuristic+child_cost
                item = [total_cost,counter,child_cost,actions,child_node]
                heapq.heappush(frontier,item)

    return action_list


def exec_action_list(action_list):
    '''
    publishes the list of actions to the publisher topic
    action_list: list of actions to execute
    '''
    global print_state_path

    if print_state_path!=0:
        states_from_action_list(action_list)

    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

debug_sequence = ['MoveF', 'REFUEL', 'TurnCCW', 'MoveF', 'MoveF', 'MoveF', 'MoveF', 'TurnCCW', 'MoveF']

if __name__ == "__main__":


    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    print_state_path = args.print_states
    r = rospy.Rate(10)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    if args.algorithm in ["bfs", "ucs"] and args.custom_heuristic == True:
        print ("Error: "+args.algorithm+" called with heuristic")
        exit(1)

    start_time = time.time()
    
    actions = algorithm(args.custom_heuristic,args.debug_mode)
    
    time_taken = time.time() - start_time
    print("Algorithm : "+str(args.algorithm))
    print("Time Taken = " + str(time_taken))
    print("Plan = " + str(actions))
    exec_action_list(actions)

