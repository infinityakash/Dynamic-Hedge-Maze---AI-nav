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
import time
import Queue
import server
import math
from problem import Helper, State
import json
import random



rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',
                    help="Please mention algorithm to use. Possible arguments = {bfs, ucs, gbfs, astar}. Default value is bfs.",
                    metavar='bfs', action='store', dest='algorithm', default="bfs", type=str)
parser.add_argument('-c', help="Use custom heuristic function. No value needed.", action='store_true',
                    dest='custom_heuristic')


#Eucidean Distance
def E_heuristic(x1,y1,x2,y2):

    return math.sqrt((x1 - x2)*(x1-x2) + (y1 - y2)*(y1-y2))


'''
    check if it's a blocked edge
    yes: return False
    no: return True
'''
def our_check_edge(old_x,old_y,new_x,new_y):
    with open('/home/cse-571/catkin_ws/src/search/cans.json', "r") as read_json:
        can_json = json.load(read_json)
    print(old_x,new_x,old_y,new_y)
    for can_name, can_loc in can_json.items():
        if new_x == old_x and new_y != old_y:
            if can_loc[0] ==new_x and can_loc[1]<=max(old_y,new_y) and can_loc[1]>=min(old_y,new_y):
            
                return False

        elif new_y == old_y:
            if can_loc[1] == new_y and can_loc[0]<=max(old_x,new_x) and can_loc[0]>=min(old_x,new_x):
            
                return False

        else:
            slope =(new_y - old_y)/(new_x -old_x)
            intercept = ((new_x * old_y) -(old_x* new_y)) /(new_x-old_x) 
            if can_loc[1] == slope*can_loc[0] + intercept and can_loc[0]<=max(old_x,new_x) and can_loc[0]>=min(old_x,new_x):
            
                return False
               
    return True

def get_successor(curr_state,next_state_list):  
    
    
    helper = problem.Helper()
    goal_state = helper.get_goal_state()
    x_dic = []
    # Chech if all possible new direction states are valid edges
    l_hue = []
    
    x_cord = curr_state[0]
    y_cord = curr_state[1]
    req_y = y_cord
    req_x = x_cord + 0.5
    min_hue = 100000000
    hue = 0

    #East
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        if our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('1')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x, req_y,'EAST',min_hue] 

    #Northeast
    req_y = y_cord + 0.5
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        print(x_cord,y_cord,req_x,req_y,'abcdcdc')        
        if our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('2')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x,req_y,'NORTHEAST',min_hue]
                
                
    #Southeast
    req_y = y_cord - 0.5
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        if  our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('3')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x,req_y,'SOUTHEAST', min_hue] 
    #North
    req_x = x_cord 
    req_y = y_cord + 0.5
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        if our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('4')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                next_state_list[str([req_x,req_y])] += 1
                #print('initial hue',hue)
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
                #print('now',hue)
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x,req_y,'NORTH', min_hue] 
    #South
    req_y = y_cord - 0.5  
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        if our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('5')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                next_state_list[str([req_x,req_y])] += 1
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x,req_y,'SOUTH', min_hue] 
    #Southwest
    req_x = x_cord - 0.5
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        if our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('6')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                next_state_list[str([req_x,req_y])] += 1
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x,req_y,'SOUTHWEST', min_hue]

    #West
    req_y = y_cord 
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        if our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('7')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                next_state_list[str([req_x,req_y])] += 1
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x,req_y,'WEST', min_hue]
    #Northwest
    req_y = y_cord + 0.5
    if (req_x>=0 and req_x <= goal_state.x) and (req_y>=0 and req_y <= goal_state.y):
        if our_check_edge(x_cord, y_cord, req_x, req_y) :
            print('8')
            hue = E_heuristic(goal_state.x,goal_state.y,req_x,req_y)
            if next_state_list.has_key(str([req_x,req_y])):
                next_state_list[str([req_x,req_y])] += 1
                hue += (0.5 * (next_state_list[str([req_x,req_y])] + 1))
                next_state_list[str([req_x,req_y])] += 1
            l_hue.append(hue)
            if min_hue >= hue:
                min_hue = hue
                x_dic = [req_x,req_y,'NORTHWEST', min_hue] 
    if x_dic == []:
        return []

    min_x_dic = x_dic[:3]
    next_state_orientation = min_x_dic[2]
    robot_orientation = curr_state[2]

    dict_radians = {"EAST": 0.0 , "NORTHEAST": math.pi / 4 ,"NORTH":  2 * (math.pi / 4),"NORTHWEST":  3 * (math.pi / 4),"WEST": 4 * (math.pi / 4),"SOUTHWEST": 5 * (math.pi / 4),"SOUTH": 6 * (math.pi / 4),"SOUTHEAST": 7 * (math.pi / 4)}
    
    robot_yaw = dict_radians[robot_orientation]
    target_yaw = dict_radians[next_state_orientation]
    action_list_one_step = []
    
    if abs(target_yaw - robot_yaw) == math.pi : # In the reverse direction
        # TurnCW,TurnCW,MoveF or TurnCCW,TurnCCW,MoveF
        action_list_one_step = ["TurnCW", "TurnCW","MoveF"]
    
    elif target_yaw - robot_yaw == (math.pi / 4) or target_yaw - robot_yaw == -7 * (math.pi/4): # 45 degrees CCW
        # TurnHalfCCW, MoveF
        action_list_one_step = ["TurnHalfCCW","MoveF"]
    
    elif target_yaw - robot_yaw == -3 * (math.pi / 2) or target_yaw - robot_yaw == math.pi/2: # 90 degrees CCW
        # TurnCCW,MoveF
        action_list_one_step = ["TurnCCW","MoveF"]
    
    elif target_yaw - robot_yaw == 3 * (math.pi / 4) or target_yaw - robot_yaw == -5 * (math.pi/4): # 135 degrees CCW
        # TurnCW, TurnHalfCCW, MoveF
        action_list_one_step = ["TurnCCW","TurnHalfCCW","MoveF"]
    
    elif target_yaw - robot_yaw == 5 * (math.pi / 4) or target_yaw - robot_yaw == -3 * (math.pi / 4): # 135 degrees CW
        # TurnCW, TurnHalfCW, MoveF
        action_list_one_step = ["TurnCW","TurnHalfCW","MoveF"]
       
    elif target_yaw - robot_yaw == 3 * (math.pi / 2) or target_yaw - robot_yaw == -(math.pi / 2): # 90 degrees CW
        # TurnCw,MoveF
        action_list_one_step = ["TurnCW","MoveF"]
       
    elif target_yaw - robot_yaw ==  - (math.pi / 4) or target_yaw - robot_yaw == 7 * (math.pi/4): # 45 degrees CW
        # TurnHalfCW, MoveF
        action_list_one_step = ["TurnHalfCW","MoveF"]
    
    elif target_yaw - robot_yaw == 0 : # No rotation
        # Move F
        action_list_one_step = ["MoveF"]
    
    else:
        #print(target_yaw - robot_yaw)
        print("ERROR!")

    succ_state = []
    succ_state.append(action_list_one_step)
    succ_state.append(min_x_dic)


    return succ_state 

# use move_can function to change can's location
def change_can(can_json,bot_loc):
    l = len(can_json)
    a = random.randint(1,l)
    for i in range(1,a):
        b = random.randint(1,l)
        can_name = 'can'+str(b)
        server.move_can(can_name,bot_loc)    




def local_search():

    helper = problem.Helper()
    init_state = helper.get_initial_state()
    curr_state = []
    curr_state.append(init_state.x)
    curr_state.append(init_state.y)
    curr_state.append(init_state.orientation)
    
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions()
    min_action = None
    min_next_state = None
    min_hue = 0
    next_state_list = {}
    while not ((curr_state[0] == goal_state.x) and (curr_state[1] == goal_state.y)): 
        if not next_state_list.has_key(str(curr_state[:2])):
            next_state_list[str(curr_state[:2])] = 0   
        else:
            next_state_list[str(curr_state[:2])] += 1
        state_dictionary = get_successor(curr_state,next_state_list)
        if state_dictionary == []:
            with open('/home/cse-571/catkin_ws/src/search/cans.json', "r") as read_json:
                can_json = json.load(read_json)         
            change_can(can_json,curr_state[:2])
            continue

        action = state_dictionary[0]
        next_state = state_dictionary[1]
        print('calling bot change')       
        server.change_state('turtlebot3_burger',[next_state[0],next_state[1],next_state[2]])
        print('next_state',next_state)        
        time.sleep(3)
        curr_state = next_state
        if(goal_state.x == curr_state[0] and goal_state.y == curr_state[1]):
            print('goal reached!')
        else:
            with open('/home/cse-571/catkin_ws/src/search/cans.json', "r") as read_json:
                can_json = json.load(read_json)          
            change_can(can_json,curr_state[:2]) 
             
        

if __name__ == "__main__":
    local_search()