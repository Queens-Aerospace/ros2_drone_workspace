from launch import LaunchDescription
from launch_ros.actions import Node

# https://youtu.be/xJ3WAs8GndA?si=5c28M9s_awHMZwev

# Go to drone_workspace then run 
# colcon build
# source install/setup.bash

# Function must have this exact name
def generate_launch_description():
        ld = LaunchDescription()
        drone_node = Node(
                package="my_drone_ctrl",
                executable="GPSNode"
        )
        ld.add_action(drone_node)
        return ld
"""
        talker_node = Node(
                package="demo_nodes_cpp",
                executable="talker"
        )

        listener_node = Node(
                package="demo_nodes_py",
                executable="listener"
        )
        
        # Add node so that it is launched
        ld.add_action(talker_node)
        ld.add_action(listener_node)
"""
