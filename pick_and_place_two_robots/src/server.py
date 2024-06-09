#!/usr/bin/env python3
import time
import math

from pick_and_place_two_robots.action import Communication

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import yaml

actions = [
    "Initial position",         #0
    "Above piece",              #1
    "Move down to pick up",     #2
    "Close Gripper",            #3
    "Move up with piece",       #4
    "Workspace exit",           #5
    "Depot area",               #6
    "Move down to drop",        #7
    "Open Gripper",             #8
    "Move up to depot area",    #9
    "Workspace exit 2",         #10
]

class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('action_server')

        self._action_server = ActionServer(
            self,
            Communication,
            'Communication',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        with open("/home/victor/ur_rtde/TFG/src/pick_and_place_two_robots/launch/config/args_for_launch.yaml", 'r') as ymlfile:
            cfg = yaml.safe_load(ymlfile)
        
        self.number_of_pieces = cfg["MinimalActionServer"]["ros__parameters"]["number_of_pieces"]
        self.positions = cfg["MinimalActionServer"]["ros__parameters"]["positions"]
        self.orientation = cfg["MinimalActionServer"]["ros__parameters"]["orientation"]
        self.type_list = cfg["MinimalActionServer"]["ros__parameters"]["type"]

        self.get_logger().info("Number of pieces: {0}".format(self.number_of_pieces))

        self.pieces = []
        for i in range(1,self.number_of_pieces+1):
            piece = self.Piece(self.type_list[i-1], [self.positions[2*i-2], self.positions[2*i-1]], self.orientation[i-1])
            self.pieces.append(piece)
            self.get_logger().info("i-1: {0}".format(i-1))
            self.get_logger().info("Pieza: {0}".format(piece.position))
            self.get_logger().info("Pieza: {0}".format(piece.classification))
            self.get_logger().info("Pieza: {0}".format(piece.orientation))
        
        self.umbral = 0.001

        self.robot1_pos = [0,0]
        self.robot1_detained = False
        self.robot1_action = actions[0]
        self.robot1_goal_pos = [20,20]
        self.robot1_gripper_status = 0
        self.robot1_piece = -10
        self.robot1_completed = False
        self.robot1_need_help = False
        self.robot2_pos = [10,10]
        self.robot2_detained = False
        self.robot2_action = actions[0]
        self.robot2_goal_pos = [20,20]
        self.robot2_gripper_status = 0
        self.robot2_piece = -10
        self.robot2_completed = False
        self.robot2_need_help = False

    class Piece:
            def __init__(self,classification,position,orientation):
                self.classification = classification
                self.position = position
                self.orientation = orientation
                self.completed = False

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')       
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Handle executing the action."""
        self.get_logger().info('Executing goal...')

        feedback_msg = Communication.Feedback()

        for i in range(1, 5):
            feedback_msg.actual_action = self.switch_action(goal_handle, feedback_msg)
            self.set_feedback(goal_handle, feedback_msg)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        result = Communication.Result()           
        result.goal_pos = feedback_msg.goal_pos
        result.movement_type = feedback_msg.movement_type
        result.actual_action = feedback_msg.actual_action
        result.action_type = feedback_msg.action_type
        result.detain = False
        
        self.get_logger().info("Goal Pos: {0}".format(result.goal_pos))
        self.get_logger().info("Actual action: {0}".format(result.actual_action))
        self.get_logger().info("Detained: {0}".format(result.detain))
        goal_handle.succeed()
        return result

    def near_goal(self, goal_handle, feedback_msg):
        """Compares the actual position with the goal position."""
        if ((abs(feedback_msg.goal_pos[0] - goal_handle.request.tcp_pos[0]) < self.umbral) 
            and (abs(feedback_msg.goal_pos[1] - goal_handle.request.tcp_pos[1]) < self.umbral) 
            and (abs(feedback_msg.goal_pos[2] - goal_handle.request.tcp_pos[2]) < self.umbral)):
            self.get_logger().info("CERCA DEL OBJETIVO")

            return True
        else:
            return False

    def set_feedback(self, goal_handle, feedback_msg):
        if (goal_handle.request.identification == 1):       
            if feedback_msg.actual_action == actions[0]:
                self.robot1_piece = self.select_piece(goal_handle)
                self.get_logger().info("self.robot1_piece: {0}".format(self.robot1_piece))
                coord = self.coord_general_to_robot1([20,35])
                feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
                feedback_msg.movement_type = "moveJ"
                feedback_msg.action_type = "TCP"
                self.robot1_need_help = self.out_of_range(1,self.pieces[self.robot1_piece].position[0])
            
            self.get_logger().info("self.robot1_piece: {0}".format(self.robot1_piece))

            if(self.robot1_piece >= 0 and self.robot1_need_help == False):
                self.first_state_machine_robot1(feedback_msg, goal_handle)

            elif ((self.robot1_piece == -1 or self.robot1_need_help == True) and (self.robot2_need_help == True)):
                self.second_state_machine_robot1(feedback_msg,goal_handle)

            else:
                self.get_logger().info("self.robot1_piece: {0}".format(self.robot1_piece))
                coord = self.coord_general_to_robot1([20,35])
                feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
                feedback_msg.movement_type = "moveJ"
                feedback_msg.action_type = "TCP"
                feedback_msg.actual_action = actions[0]

            feedback_msg.detain = self.set_detain(goal_handle)
            gp = self.coord_robot1_to_general(feedback_msg.goal_pos[:2])
            self.robot1_goal_pos[0] = gp[0]
            self.robot1_goal_pos[1] = gp[1]
            p = self.coord_robot1_to_general(goal_handle.request.tcp_pos[:2])
            self.robot1_pos[0] = p[0]
            self.robot1_pos[1] = p[1]
            self.robot1_gripper_status = goal_handle.request.gripper_status
            self.robot1_action = feedback_msg.actual_action   
        if (goal_handle.request.identification == 2):
            if feedback_msg.actual_action == actions[0]:
                self.robot2_piece = self.select_piece(goal_handle)
                self.get_logger().info("self.robot2_piece: {0}".format(self.robot2_piece))
                coord = self.coord_general_to_robot2([-14,22])
                feedback_msg.goal_pos = [coord[0],coord[1],0.1,0.0,3.139,0.0]
                feedback_msg.movement_type = "moveJ"
                feedback_msg.action_type = "TCP"
                self.robot2_need_help = self.out_of_range(2,self.pieces[self.robot2_piece].position[0])

            if(self.robot2_piece >= 0 and self.robot2_need_help == False):
                self.first_state_machine_robot2(feedback_msg,goal_handle)
            
            elif (self.robot2_piece < 0 and self.robot1_need_help == True):
                self.second_state_machine_robot2(feedback_msg,goal_handle)
            
            else:
                self.get_logger().info("self.robot2_piece: {0}".format(self.robot2_piece))
                coord = self.coord_general_to_robot2([-14,22])
                feedback_msg.goal_pos = [coord[0],coord[1],0.1,0.0,3.139,0.0]
                feedback_msg.movement_type = "moveJ"
                feedback_msg.action_type = "TCP"
                feedback_msg.actual_action = actions[0]

            feedback_msg.detain = self.set_detain(goal_handle)
            gp = self.coord_robot2_to_general(feedback_msg.goal_pos[:2])
            self.robot2_goal_pos[0] = gp[0]
            self.robot2_goal_pos[1] = gp[1]
            p = self.coord_robot2_to_general(goal_handle.request.tcp_pos[:2])
            self.robot2_pos[0] = p[0]
            self.robot2_pos[1] = p[1]
            self.robot2_action = feedback_msg.actual_action
            self.robot2_gripper_status = goal_handle.request.gripper_status   

    def switch_action(self, goal_handle, feedback_msg):
        self.get_logger().info("Gripper Status: {0}".format(goal_handle.request.gripper_status))
        self.get_logger().info("goal_handle.request.last_action: {0}".format(goal_handle.request.last_action))
        if ((goal_handle.request.last_action == actions[2]) or (goal_handle.request.last_action == actions[3])):
            if(goal_handle.request.gripper_status == 1):
                return actions[4]
            else:
                return actions[3]
        elif ((goal_handle.request.last_action == actions[7]) or (goal_handle.request.last_action == actions[8])):
            if(goal_handle.request.gripper_status == 0):
                return actions[9]
            else: 
                return actions[8]
        else:
            if (self.near_goal(goal_handle, feedback_msg) and (goal_handle.request.last_action != ("Close Gripper" and "Open Gripper"))):
                current_index = actions.index(goal_handle.request.last_action)
                next_index = (current_index + 1) % len(actions)
                return actions[next_index]
            else:
                current_index = actions.index(goal_handle.request.last_action)
                return actions[current_index]

    
    def set_detain(self,goal_handle):
        distance = self.calculate_distance(self.robot1_pos,self.robot2_pos)
        self.get_logger().info("Distancia: {0}".format(distance))      
        self.get_logger().info("Pos 1: {0}".format(self.robot1_pos))
        self.get_logger().info("Pos 2: {0}".format(self.robot2_pos))
        if distance < 15    :
            if ((self.robot1_action == actions[1]) and (self.robot2_action == actions[1])):  
                distance1 = self.calculate_distance(self.robot1_pos,self.robot1_goal_pos)
                distance2 = self.calculate_distance(self.robot2_pos,self.robot2_goal_pos)
                if (distance1 <= distance2):
                    self.robot1_detained = False
                    self.robot2_detained = True
                elif (distance1 > distance2):
                    self.robot1_detained = True
                    self.robot2_detained = False
            elif ((self.robot1_action == actions[1]) and (self.robot2_action == (actions[2] or actions[3] or actions[4] or actions[5]))):
                self.robot1_detained = True
                self.robot2_detained = False
            elif ((self.robot2_action == actions[1]) and (self.robot1_action == (actions[2] or actions[3] or actions[4] or actions[5]))):
                self.robot1_detained = False
                self.robot2_detained = True
            elif ((self.robot1_action == actions[5]) and (self.robot2_action == (actions[5]))):
                if (self.robot1_pos[1] <= self.robot2_pos[1]):
                    self.robot1_detained = True
                    self.robot2_detained = False                   
                elif (self.robot1_pos[1] > self.robot2_pos[1]):
                    self.robot1_detained = False
                    self.robot2_detained = True
            elif ((self.robot1_action == actions[5]) and (self.robot2_action == (actions[6] or actions[7] or actions[8] or actions[9]))):
                self.robot1_detained = True
                self.robot2_detained = False
            elif ((self.robot2_action == actions[5]) and (self.robot1_action == (actions[6] or actions[7] or actions[8] or actions[9]))):
                self.robot1_detained = False
                self.robot2_detained = True
        else:
            self.robot1_detained = False
            self.robot2_detained = False

        self.get_logger().info("Robot1 detain: {0}".format(self.robot1_detained))
        self.get_logger().info("Robot2 detain: {0}".format(self.robot2_detained))
        
        if (goal_handle.request.identification == 1):
            return self.robot1_detained
        elif (goal_handle.request.identification == 2):
            return self.robot2_detained
    
    def calculate_distance(self,pos1,pos2):
        distance = math.sqrt((pos1[0] - pos2[0])** 2 + (pos1[1] - pos2[1])** 2)
        return distance

    def coord_robot1_to_general(self,pos):
        result = [0.0,0.0]
        result[0] = 100 * pos[0] + 28
        result[1] = 100 * pos[1] + 50.3
        return result
    
    def coord_robot2_to_general(self,pos):
        result = [0.0,0.0]
        result[0] = (-100) * pos[0] - 30.5
        result[1] = (-100) * pos[1] + 2.5
        return result
    
    def coord_general_to_robot1(self,pos):
        result = [0.0,0.0]
        result[0] = 0.01 * pos[0] - 0.28
        result[1] = 0.01 * pos[1] - 0.503
        return result
    
    def coord_general_to_robot2(self,pos):
        result = [0.0,0.0]
        result[0] = (-0.01) * pos[0] - 0.305
        result[1] = (-0.01) * pos[1] + 0.025
        return result
    
    def select_piece(self,goal_handle):
        if (goal_handle.request.identification == 1 and self.robot1_completed == True and self.robot1_action == actions[0]):
            self.pieces[self.robot1_piece].completed = True
            self.robot1_completed = False
            self.get_logger().info("Piece completed: {0}".format(self.robot1_piece))
        if (goal_handle.request.identification == 2 and self.robot2_completed == True and self.robot2_action == actions[0]):
            self.pieces[self.robot2_piece].completed = True
            self.robot2_completed = False
            self.get_logger().info("Piece completed: {0}".format(self.robot2_piece))

        for i in range(1,self.number_of_pieces+1):
            self.get_logger().info("pieza: {0}".format(i-1))
            self.get_logger().info("Completed:{0}".format(self.pieces[i-1].completed))
            if((self.pieces[i-1].classification == goal_handle.request.identification) and self.pieces[i-1].completed == False):
                return i-1
                break
        return -1
    
    def organize_pieces(self):
        return 0

    def set_orientation(self,number):
        if(self.pieces[number].orientation == "vertical"):
            return [2.065,2.345]
        elif(self.pieces[number].orientation == "horizontal"):
            return [-0.23,3.1057]
    
    def first_state_machine_robot1(self,feedback_msg,goal_handle):
        if feedback_msg.actual_action == actions[1]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[2]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.02,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[3]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "Close Gripper"
            feedback_msg.action_type = "Gripper"
        elif feedback_msg.actual_action == actions[4]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[5]:
            coord = self.coord_general_to_robot1([12,46.7])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"   
        elif feedback_msg.actual_action == actions[6]:
            coord = self.coord_general_to_robot1([4,46.7])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"  
        elif feedback_msg.actual_action == actions[7]:
            coord = self.coord_general_to_robot1([4,46.7])
            feedback_msg.goal_pos = [coord[0],coord[1],0.08,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"            
        elif feedback_msg.actual_action == actions[8]:
            coord = self.coord_general_to_robot1([4,46.7])
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,2.065,2.345,0.0]
            feedback_msg.movement_type = "Open Gripper"
            feedback_msg.action_type = "Gripper"      
        elif feedback_msg.actual_action == actions[9]:
            coord = self.coord_general_to_robot1([4,46.7])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[9]:
            coord = self.coord_general_to_robot1([4,46.7])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[10]:          
            coord = self.coord_general_to_robot1([12,46.7])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP" 
            self.robot1_completed = True

    def first_state_machine_robot2(self,feedback_msg,goal_handle):
        if feedback_msg.actual_action == actions[1]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[2]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.02,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[3]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "Close Gripper"
            feedback_msg.action_type = "Gripper"
        elif feedback_msg.actual_action == actions[4]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[5]:
            coord = self.coord_general_to_robot2([-8,37.15])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"   
        elif feedback_msg.actual_action == actions[6]:
            coord = self.coord_general_to_robot2([4,37.15])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"  
        elif feedback_msg.actual_action == actions[7]:
            coord = self.coord_general_to_robot2([4,37.15])
            feedback_msg.goal_pos = [coord[0],coord[1],0.08,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"            
        elif feedback_msg.actual_action == actions[8]:
            coord = self.coord_general_to_robot2([4,37.15])
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,2.065,2.345,0.0]
            feedback_msg.movement_type = "Open Gripper"
            feedback_msg.action_type = "Gripper"      
        elif feedback_msg.actual_action == actions[9]:
            coord = self.coord_general_to_robot2([4,37.15])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[10]:
            coord = self.coord_general_to_robot2([4,37.15])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP" 
            #self.pieces[self.robot2_piece].completed = True
            self.robot2_completed = True

    def second_state_machine_robot1(self,feedback_msg,goal_handle):
        if feedback_msg.actual_action == actions[1]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[2]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.02,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[3]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "Close Gripper"
            feedback_msg.action_type = "Gripper"
        elif feedback_msg.actual_action == actions[4]:
            coord = self.coord_general_to_robot1(self.pieces[self.robot2_piece].position)
            orientation = self.set_orientation(self.robot2_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[5]:
            coord = self.coord_general_to_robot1([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"   
        elif feedback_msg.actual_action == actions[6]:
            coord = self.coord_general_to_robot1([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"  
        elif feedback_msg.actual_action == actions[7]:
            coord = self.coord_general_to_robot1([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.02,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"            
        elif feedback_msg.actual_action == actions[8]:
            coord = self.coord_general_to_robot1([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,2.065,2.345,0.0]
            feedback_msg.movement_type = "Open Gripper"
            feedback_msg.action_type = "Gripper"      
        elif feedback_msg.actual_action == actions[9]:
            coord = self.coord_general_to_robot1([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[9]:
            coord = self.coord_general_to_robot1([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[10]:          
            coord = self.coord_general_to_robot1([10,15])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP" 
            self.pieces[self.robot2_piece].position = [10.0,10.0]
            self.pieces[self.robot2_piece].orientation = "vertical"
            self.robot2_need_help = False

    def second_state_machine_robot2(self,feedback_msg,goal_handle):
        if feedback_msg.actual_action == actions[1]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[2]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.02,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[3]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "Close Gripper"
            feedback_msg.action_type = "Gripper"
        elif feedback_msg.actual_action == actions[4]:
            coord = self.coord_general_to_robot2(self.pieces[self.robot1_piece].position)
            orientation = self.set_orientation(self.robot1_piece)
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,orientation[0],orientation[1],0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[5]:
            coord = self.coord_general_to_robot2([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"   
        elif feedback_msg.actual_action == actions[6]:
            coord = self.coord_general_to_robot2([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"  
        elif feedback_msg.actual_action == actions[7]:
            coord = self.coord_general_to_robot2([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.02,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"            
        elif feedback_msg.actual_action == actions[8]:
            coord = self.coord_general_to_robot2([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],10.0,2.065,2.345,0.0]
            feedback_msg.movement_type = "Open Gripper"
            feedback_msg.action_type = "Gripper"      
        elif feedback_msg.actual_action == actions[9]:
            coord = self.coord_general_to_robot2([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[9]:
            coord = self.coord_general_to_robot2([10,10])
            feedback_msg.goal_pos = [coord[0],coord[1],0.1,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP"
        elif feedback_msg.actual_action == actions[10]:          
            coord = self.coord_general_to_robot2([10,15])
            feedback_msg.goal_pos = [coord[0],coord[1],0.15,2.065,2.345,0.0]
            feedback_msg.movement_type = "moveJ"
            feedback_msg.action_type = "TCP" 
            self.pieces[self.robot1_piece].position = [10.0,10.0]
            self.pieces[self.robot1_piece].orientation = "vertical"
            self.robot1_need_help = False

    def out_of_range(self,classification,position):
        if(classification == 1):
            if (position <= 5):
                return True
            else:
                return False
        elif (classification == 2):
            if (position >= 20):
                return True
            else:
                return False

def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = MinimalActionServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_action_server, executor=executor)

    minimal_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


 