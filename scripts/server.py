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

from search.srv import *
import rospy
from gen_maze import *
import sys
import argparse
import time
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
import problem
import math

mazeInfo = None
parser = argparse.ArgumentParser()
parser.add_argument('-d', help='for providing dimension of the grid', metavar='5', action='store', dest='grid_dimension', default=5, type=int)
parser.add_argument('-n', help='for providing no. of obstacles to be added in the grid', metavar='15', action='store', dest='n_obstacles', default=15, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)

status = String(data='Idle')
model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)
action_publisher = rospy.Publisher("/actions",String,queue_size= 10)
status_publisher = rospy.Publisher("/status",String,queue_size=10)

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
	
	global mazeInfo
	global grid
	action_list = ["TurnHalfCW", "TurnCW", "TurnCCW", "TurnHalfCCW", "MoveB", "MoveF"]
	direction_list = ["NORTH", "NORTHEAST", "EAST", "SOUTHEAST","SOUTH", "SOUTHWEST", "WEST", "NORTHWEST"]
	state_x = []
	state_y = []
	state_direction = []
	state_cost = []

	
	for action in action_list:
		#Checking requested action and making changes in states
		x_cord, y_cord, direction = req.x, req.y, req.direction
		if action == 'TurnHalfCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index+1)%8]
			g_cost = 2

		elif action == 'TurnHalfCCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index-1)%8]
			g_cost = 2
		elif action == 'TurnCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index+1)%4]
			g_cost = 2

		elif action == 'TurnCCW':
			index = direction_list.index(req.direction)
			direction = direction_list[(index-1)%4]
			g_cost = 2
		elif action == 'MoveF':
			if direction == "NORTH":
				y_cord += 0.5
			elif direction == "EAST":
				x_cord += 0.5
			elif direction == "SOUTH":
				y_cord -= 0.5
			elif direction == "WEST":
				x_cord -= 0.5
			elif direction == "NORTHEAST":
				x_cord += 0.5
				y_cord += 0.5
			elif direction == "SOUTHEAST":
				x_cord += 0.5
				y_cord -= 0.5
			elif direction == "SOUTHWEST":
				x_cord -= 0.5
				y_cord -= 0.5
			elif direction == "NORTHWEST":
				x_cord += 0.5
				y_cord -= 0.5
			g_cost = 1

		elif action == 'MoveB':
			if direction == "NORTH":
				y_cord -= 0.5
			elif direction == "EAST":
				x_cord -= 0.5
			elif direction == "SOUTH":
				y_cord += 0.5
			elif direction == "WEST":
				x_cord += 0.5
			elif direction == "NORTHEAST":
				x_cord -= 0.5
				y_cord -= 0.5
			elif direction == "SOUTHEAST":
				x_cord -= 0.5
				y_cord += 0.5
			elif direction == "SOUTHWEST":
				x_cord += 0.5
				y_cord += 0.5
			elif direction == "NORTHWEST":
				x_cord -= 0.5
				y_cord += 0.5
			g_cost = 3
		
		if req.x <= x_cord and req.y <= y_cord:
			isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord), "changedValuesLater")
		else:
			isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")

		if not isValidEdge:
			state_x.append(-1)
			state_y.append(-1)
			state_direction.append(direction)
			state_cost.append(-1)
		else:
			state_x.append(x_cord)
			state_y.append(y_cord)
			state_direction.append(direction)
			state_cost.append(g_cost)

	return GetSuccessorResponse(state_x, state_y, state_direction, state_cost, action_list)
  

def handle_get_initial_state(req):
	"""
	This function will return initial state of turtlebot3.
	"""
	global mazeInfo

	initial_state = mazeInfo[0]
	return GetInitialStateResponse(initial_state[0],initial_state[0],initial_state[2])


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

def change_state(can_name,target_transform):
	model_state_msg = ModelState()
	model_state_msg.model_name = can_name
	if can_name == 'turtlebot3_burger':
		print('calling bot change')
		print(target_transform)
		model_state_msg.pose.position.x = target_transform[0]
		model_state_msg.pose.position.y = target_transform[1]
		model_state_msg.pose.position.z = 0

		model_state_publisher.publish(model_state_msg)
		return
		
	else:
		# change can location
		model_state_msg.pose.position.x = target_transform[0]
		model_state_msg.pose.position.y = target_transform[1]
		model_state_msg.pose.position.z = 0
		model_state_publisher.publish(model_state_msg)

def move_can(req, bot_loc):
	helper = problem.Helper()
	goal_state = helper.get_goal_state()
	can_name = req
	print('updating: ',can_name)
	scale = 0.5
	target = []
	grid = goal_state.x
	with open('/home/cse-571/catkin_ws/src/search/cans.json', "r") as read_json:
		can_json = json.load(read_json)

	x = scale * np.random.randint(1, grid*2+1)
	y = scale * np.random.randint(1, grid*2+1)
	
	flag = np.random.randint(0,3)
	if x == 0 and y == 0:
		print('cannot update as its initial state')
		return
	elif x == bot_loc[0] and y == bot_loc[1] or (((bot_loc[0] - x)**2 + (bot_loc[1] - y)**2)<=(.2*.2)):
		print('cannot updata near the bot')
		return
	elif x == goal_state.x and y == goal_state.y:
		print('cannot update as its goal state')
		return 
	offset = np.random.uniform(0, 0.07*scale)
	# renew can location
	if flag == 0:	
		target = [x-0.25,y-0.25,0]
	
	elif flag == 1:
		target = [x-0.25+offset,y,0]
	
	else:
		target = [x,y-0.25+offset,0]

	if target[:2] in can_json.values():
		print('cannot updata at a duplicate location')
		return
	elif target[0] == 0 and target[1] == 0:
		print('cannot update at the initial state')
		return
	elif (target[0] == bot_loc[0] and target[1] == bot_loc[1]) or (((bot_loc[0] - target[0])**2 + (bot_loc[1] - target[1])**2)<=(0.2*0.2)):
		print('cannot updata near the bot')
		
		return
	elif target[0] == goal_state.x and target[1] == goal_state.y:
		print('cannot update as its goal state')
		return 
	else:
		a = target[0]
		b = target[1]
		can_json[can_name] = target[:2]
		for i in can_json:
			if i == can_name:
				continue
			if (can_json[i][0]-a)**2 + (can_json[i][1]-b)**2 <= (0.2)**2:
				print('cannot update as its near ',i)
				return
		with open('/home/cse-571/catkin_ws/src/search/cans.json', "w") as write_json:
			json.dump(can_json,write_json)
		change_state(can_name,target)
		time.sleep(1)

def server():
    rospy.init_node('get_successor_server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('get_initial_state', GetInitialState, handle_get_initial_state)
    rospy.Service('is_goal_state', IsGoalState, handle_is_goal_state)
    rospy.Service('get_goal_state',GetGoalState,handle_get_goal_state)
    rospy.Service('execute_move_can_action',MoveCan, move_can)
    print "Ready!"
    #print('--',check_is_edge((0,1.5, 0 ,2),'changedValuesLater'))
    rospy.spin()

if __name__ == "__main__":
    args = parser.parse_args()
    grid = args.grid_dimension
    possible_n_obstacles =  args.grid_dimension*(args.grid_dimension + 1)*2
    if args.n_obstacles > possible_n_obstacles:
        print('Maximum no. of obstacles that could be added to the grid is {} but provided value is {}'.format(possible_n_obstacles, args.n_obstacles))
        exit()
    my_maze = Maze()
    mazeInfo = my_maze.generate_maze(args.grid_dimension, args.n_obstacles, args.seed)
    print('----------',mazeInfo)

    server()