#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Ketan Patil"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from cse571_project.srv import *
import rospy
from gen_maze import *
import sys
import argparse
import time
import numpy as np


mazeInfo = None
parser = argparse.ArgumentParser()
parser.add_argument('-d', help='for providing dimension of the grid', metavar='5', action='store', dest='grid_dimension', default=5, type=int)
parser.add_argument('-n', help='for providing no. of obstacles to be added in the grid', metavar='15', action='store', dest='n_obstacles', default=15, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
parser.add_argument('-f', help='for providing no. of fuel stations', metavar='3', action='store', dest='num_fuel_stations', default=3, type=int)
parser.add_argument('-b', help ='for providing initial battery level', metavar='15', action='store', dest='battery_input_value', default=10, type=int)


def manhattanDistance(x1, y1, x2, y2):
	"""
	This function returns manhattan distance between two points.
	"""
	return abs(x1-x2) + abs(y1-y2)

def check_is_edge(edge, valueFlag):
	"""
	This function checks if two points are connected via edge or not.
	"""
	global mazeInfo
	invalid_edges = mazeInfo[1]
	if valueFlag == "changedValuesLater":
		if edge[2] < mazeInfo[0][0] or edge[2] > mazeInfo[0][1]*0.5 or edge[3] < mazeInfo[0][0] or edge[3] > mazeInfo[0][1]*0.5:
			return False
	elif valueFlag == "changedValuesBefore":
		if edge[0] < mazeInfo[0][0] or edge[0] > mazeInfo[0][1]*0.5 or edge[1] < mazeInfo[0][0] or edge[1] > mazeInfo[0][1]*0.5:
			return False

	if edge in invalid_edges:
		return False
	else:
		return True

def handle_get_successor(req):
	"""
		This function returns all successors of a given state 
				
		parameters:	x_cord - current x-cordinate of turtlebot
				    y_cord - current y-cordinate of turtlebot
				    direction - current orientation

		output:   
			GetSuccessorResponse (search/srv/GetSuccessor.srv)
	"""
	global mazeInfo,num_fuel_stations
	action_list = []
	direction_list = ["NORTH", "NORTH_EAST", "EAST", "SOUTH_EAST", "SOUTH", "SOUTH_WEST", "WEST", "NORTH_WEST"]
	state_x = []
	state_y = []
	state_direction = []
	state_cost = []
	state_battery = []
	x_cord, y_cord, direction,battery = req.x, req.y, req.orientation,req.battery
	nearby_clearance = 0.1
	refueling_cost = 1
	fuel_stations = mazeInfo[-1]
	FULL_BATTERY_CAPACITY = 10

	print battery
	if battery>0:
		action_list = ["TurnCW", "TurnCCW", "MoveB", "MoveF"]
		
	if num_fuel_stations!=len(fuel_stations):
		print("Something is wrong!")
	##Check if state is eligible for refueling station
	for (fx,fy) in fuel_stations:
		if manhattanDistance(fx,fy,req.x,req.y)<=nearby_clearance:
			print("Robot is near fuelling station!")
			action_list.append("REFUEL")
			break



	for action in action_list:
		#Checking requested action and making changes in states
		x_cord, y_cord, direction,battery = req.x, req.y, req.orientation,req.battery
		if action == 'REFUEL':
			x_cord = req.x
			y_cord = req.y
			g_cost = refueling_cost
			battery=FULL_BATTERY_CAPACITY

		elif action == 'TurnCW':
			index = direction_list.index(req.orientation)
			direction = direction_list[(index+1)%len(action_list)]
			g_cost = 2
			battery-=1

		elif action == 'TurnCCW':
			index = direction_list.index(req.orientation)
			direction = direction_list[(index-1)%len(action_list)]
			g_cost = 2
			battery-=1

		elif action == 'MoveF':
			if direction == "NORTH" or direction == "NORTH_EAST" or direction == "NORTH_WEST":
				y_cord += 0.5
			if direction == "EAST" or direction == "NORTH_EAST" or direction == "SOUTH_EAST":
				x_cord += 0.5
			if direction == "SOUTH" or direction == "SOUTH_EAST" or direction == "SOUTH_WEST":
				y_cord -= 0.5
			if direction == "WEST" or direction == "NORTH_WEST" or direction == "SOUTH_WEST":
				x_cord -= 0.5
			g_cost = 1
			battery-=1

		elif action == 'MoveB':
			if direction == "NORTH" or direction == "NORTH_EAST" or direction == "NORTH_WEST":
				y_cord -= 0.5
			if direction == "EAST" or direction == "NORTH_EAST" or direction == "SOUTH_EAST":
				x_cord -= 0.5
			if direction == "SOUTH" or direction == "SOUTH_EAST" or direction == "SOUTH_WEST":
				y_cord += 0.5
			if direction == "WEST" or direction == "NORTH_WEST" or direction == "SOUTH_WEST":
				x_cord += 0.5
			g_cost = 3
			battery-=1

		if req.x <= x_cord and req.y <= y_cord:
			isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord), "changedValuesLater")
		else:
			isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")

		if not isValidEdge:
			state_x.append(-1)
			state_y.append(-1)
			state_direction.append(direction)
			state_cost.append(-1)
			state_battery.append(battery) #Since bot can't move on invalid edge, assume that no battery is exhausted.
		else:
			state_x.append(x_cord)
			state_y.append(y_cord)
			state_battery.append(battery)
			state_direction.append(direction)
			state_cost.append(g_cost)
			#print g_cost
		if len(action_list) == 0:
			state_x = []
			state_y = []
			state_direction = []
			state_battery = []
			state_cost.append(1000000)


	return GetSuccessorResponse(state_x, state_y, state_direction, state_battery, state_cost, action_list)

def handle_get_initial_state(req):
	"""
	This function will return initial state of turtlebot3.
	"""
	global mazeInfo

	initial_state = mazeInfo[0]
	return GetInitialStateResponse(initial_state[0],initial_state[0],initial_state[2],initial_state[3])

def handle_is_goal_state(req):
	"""
    This function will return True if turtlebot3 is at goal state otherwise it will return False.
	"""
	global mazeInfo
	goal_state = mazeInfo[0][1]*0.5
	if req.x == req.y and req.x == goal_state:
		return IsGoalStateResponse(1)

	return IsGoalStateResponse(0)

def handle_get_goal_state(req):
	global mazeInfo
	goal_state = mazeInfo[0][1]*0.5
	return GetGoalStateResponse(goal_state,goal_state)

def handle_get_actions(req):
	return "TurnCW,TurnCCW,MoveB,MoveF,REFUEL"

def server():
    rospy.init_node('get_successor_server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('get_initial_state', GetInitialState, handle_get_initial_state)
    rospy.Service('is_goal_state', IsGoalState, handle_is_goal_state)
    rospy.Service('get_goal_state',GetGoalState,handle_get_goal_state)
    rospy.Service('get_all_actions',GetActions,handle_get_actions)

    print "Ready!"
    rospy.spin()

if __name__ == "__main__":
    args = parser.parse_args()
    possible_n_obstacles =  args.grid_dimension*(args.grid_dimension + 1)*2
    if args.n_obstacles > possible_n_obstacles:
    	print('Maximum no. of obstacles that could be added to the grid is {} but provided value is {}'.format(possible_n_obstacles, args.n_obstacles))
    	exit()
    my_maze = Maze()
    mazeInfo = my_maze.generate_maze(args.grid_dimension, args.n_obstacles, args.seed,args.num_fuel_stations,battery = args.battery_input_value)
    num_fuel_stations = args.num_fuel_stations
    server()	