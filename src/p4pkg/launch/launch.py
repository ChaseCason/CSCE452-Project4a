from launch import LaunchDescription
from launch_ros.actions import *
import os
from p4pkg.disc_robot import *
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handler import *
from launch.event import *

def generate_launch_description():

    #input_bag_arg = DeclareLaunchArgument('bag_in')
    #output_bag_arg = DeclareLaunchArgument('bag_out')

    #robot = DeclareLaunchArgument('robot')

    launchBot = load_disc_robot('normal.robot')
    node = Node(package='p4pkg',
                executable='simulator')

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description' : launchBot['urdf']}])

    #ep = ExecuteProcess(cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_in')
    #record = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_out'), '/scan', '/map'])
    # event_handler = OnProcessExit(target_action=ep, on_exit[EmitEvent(event=Shutdown())])
    # terminate_at_end = RegisterEventHandler(event_handler)
    #ed, record, terminate_at_end
    ld = LaunchDescription([ robot_state_publisher_node, node])

    return ld