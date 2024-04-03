import sys
import os

from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    try:
        get_package_share_directory("my_robot_bringup_ms")
    except PackageNotFoundError:
        print(
            "ERROR:"
            "Could not find the package my_robot_bringup_ms" 
        )
        sys.exit(1)
    
    config_parameters = PathJoinSubstitution(
        [FindPackageShare("my_robot_bringup_ms"), "/home/victor/workspace/ros_ur_driver/src/Pick-and-Place-with-ROS2/my_robot_bringup_ms/config", "param_bringup.yaml"]
    )
    
    try:
        get_package_share_directory("my_func_nodes")
    except PackageNotFoundError:
        print(
            "ERROR:"
            "Could not find the package my_func_nodes" 
        )
        sys.exit(1)

    try:
        get_package_share_directory("ur_bringup")
    except PackageNotFoundError:
        print(
            "ERROR:"
            "Could not find the package ur_bringup" 
        )
        sys.exit(1)

    #Ejecucion del archivo de lanzamiento de lo controladores de UR
    drivers = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_bringup'),
                    "/home/victor/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/launch",'ur_control.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur3e',
                'robot_ip': '192.168.20.35',
                'use_fake_hardware':'false',
                'launch_rviz':'false'#,
                #'initial_joint_controller':'joint_trajectory_controller'
            }.items()
        )
    #Ejecución del archivo de lanzamiento de Moveit
    moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_bringup'),
                    "/home/victor/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/launch",'ur_moveit.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur3e',
                'robot_ip': '168.20.35',
                'launch_rviz':'true'
            }.items()
        )
        
    #Ejecucion del nodo creador de imagenes, camera 
    camera = Node(
                package="my_func_nodes",
                executable="camera_exec",
                name="camera",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )

    
    #Ejecucion del nodo de control del robot, control_robot_node
    control_robot_node = Node(
                package="my_func_nodes",
                executable="control_robot_exec",
                name="control_robot_master",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )

    #Ejecucion del nodo de deteccion de objetos, camera_detection
    camera_detection = Node(
                package="my_func_nodes",
                executable="camera_detection",
                name="camera_pub_pos",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
    
    #Ejecucion del nodo de la GUI, interfaz
    interfaz = Node(
                package="my_func_nodes",
                executable="interfaz_exec",
                name="interfaz_menu",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )


    nodes_to_start = [
    	
        drivers,
        moveit,
        control_robot_node,
        camera,
        camera_detection,
        interfaz,
    	
    ]   

    return LaunchDescription(nodes_to_start)
