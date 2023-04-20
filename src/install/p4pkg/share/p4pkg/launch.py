from launch import LaunchDescription
from launch_ros.actions import *

def generate_launch_description():
    node = Node(package='p4pkg',
                executable='simulator')
    ld = LaunchDescription([ node ])
    return ld
