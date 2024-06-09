#!/usr/bin/env python3
#ROS2 includes
from action_msgs.msg import GoalStatus
from pick_and_place_two_robots.action import Communication

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

#UR RTDE includes
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEInterface
import os
import psutil
import sys
import time
import numpy as np


class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('action_client_1')
        self._action_client = ActionClient(self, Communication, 'Communication')
        self._send_goal_future = None
        self._goal_count = 0
        
        
        # FALTA DEFINIR UNA POSICION INICIAL Y MOVER A LOS ROBOTS A ESA POSICION 
        
        self.prev_goal = [3.0,3.0,3.0,3.0,3.0,3.0]
        self.prev_action = "Initial position"
        self.prev_detained = False
        self.prev_gripper_status = 0

        # Parámetros
        self.vel = 1.0  # Velocidad de movimiento
        self.acc = 0.1  # Aceleración de movimiento
        rtde_frequency = 500.0  # Frecuencia de RTDE
        dt = 1.0 / rtde_frequency  # Paso de tiempo
        flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
        ur_cap_port = 50002  # Puerto de capacidad de UR
        robot_ip = "192.168.10.35"  # Dirección IP del robot

        # Prioridades en tiempo real de ur_rtde
        rt_receive_priority = 90
        rt_control_priority = 85
        rt_interface_priority = 80

        # Interfaces RTDE
        self.rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
        self.rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
        self.rtde_i = RTDEInterface(robot_ip,rtde_frequency,True)
        print("Good")

        # Prioridad en tiempo real de la aplicación
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (ya sea de 32 bits o de 64 bits)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)

        # Iniciar el temporizador para enviar metas cada 0.7 segundos
        self.timer = self.create_timer(0.7, self.send_goal)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Goal position feedback: {0}'.format(feedback.feedback.goal_pos))
        self.get_logger().info('action feedback: {0}'.format(feedback.feedback.actual_action))

        self.execute_action(feedback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Goal position Result: {0}'.format(result.goal_pos))
            self.get_logger().info('Detain Result: {0}'.format(result.goal_pos))
            self.get_logger().info('action type Result: {0}'.format(result.action_type))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def execute_action(self, feedback):
        if (feedback.feedback.detain == True):
            self.get_logger().info('Detaining the action...')
            self.detain(feedback)
        else:
            if np.array_equal(feedback.feedback.goal_pos, self.prev_goal):
                self.restart_movement(feedback)
                self.get_logger().info('Continuing the action...')
            else:
               self.prev_goal = feedback.feedback.goal_pos
               self.action_planner(feedback)
        self.prev_goal = feedback.feedback.goal_pos
        self.prev_action = feedback.feedback.actual_action
        
        self.prev_detained = feedback.feedback.detain
        # hacer que se obtenga el valor del gripper
        #self.prev_gripper_status = 0

    def action_planner(self, feedback):
        if (feedback.feedback.action_type == "Gripper"):
            self.move_gripper(feedback)
        elif (feedback.feedback.action_type == "TCP"):
            self.move(feedback)

    def move_gripper(self, feedback):
        self.rtde_i.setToolDigitalOut(0,False)
        self.rtde_i.setToolDigitalOut(1,False)
        time.sleep(0.5)
        if (feedback.feedback.movement_type == "Close Gripper"):
            self.get_logger().info('Closing...')
            self.close_gripper()
        elif (feedback.feedback.movement_type == "Open Gripper"):
            self.get_logger().info('Opening...')
            self.open_gripper()

    def move(self, feedback):
        if (feedback.feedback.movement_type == "moveJ"):
            self.rtde_c.moveJ_IK(feedback.feedback.goal_pos, self.vel, self.acc,True)
        elif (feedback.feedback.movement_type == "moveL"):
            self.rtde_c.moveL_IK(feedback.feedback.goal_pos, self.vel, self.acc,True)
        elif (feedback.feedback.movement_type == "moveC"):
            self.rtde_c.moveC_IK(feedback.feedback.goal_pos, self.vel, self.acc,True)

    def detain(self,feedback):
        self.rtde_c.stopJ()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Communication.Goal()
        goal_msg.identification = 1
        goal_msg.actual_pos = self.get_joint_position()
        goal_msg.tcp_pos = self.get_tool_position()
        goal_msg.gripper_status = self.prev_gripper_status
        goal_msg.last_action = self.prev_action
        goal_msg.goal_pos = self.prev_goal
        goal_msg.detain = self.prev_detained

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    #Función para imprimir la posición de la herramienta cada segundo
    def get_tool_position(self):
        actual_tcp_pose = self.rtde_r.getActualTCPPose()
        return actual_tcp_pose
        
    def get_joint_position(self):
        actual_pose = self.rtde_r.getJointControlOutput()
        return actual_pose

    def close_gripper(self):
        # Cerrar el gripper
        time.sleep(2.5)
        self.rtde_i.setToolDigitalOut(1,True)
        time.sleep(1)
        self.rtde_i.setToolDigitalOut(1,False)
        time.sleep(1) 
        self.prev_gripper_status = 1

    def open_gripper(self):
        # Abrir el gripper
        time.sleep(2.5)
        self.rtde_i.setToolDigitalOut(0, True)
        time.sleep(1)
        self.rtde_i.setToolDigitalOut(0, False)
        time.sleep(1)
        self.prev_gripper_status = 0
    
    def restart_movement(self,feedback):
        if ((self.prev_detained == True) and (feedback.feedback.detain == False)):
            self.action_planner(feedback)

    
def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
