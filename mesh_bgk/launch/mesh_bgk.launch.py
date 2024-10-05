### import  
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

### variables pkgs's name  
pkg_name = 'mesh_bgk'      

### launch description function
def generate_launch_description():
    ## read config params
    params_file = os.path.join(get_package_share_directory(
        pkg_name), 'config/params.yaml')  
    print(params_file)  
    
    ld = LaunchDescription()  
    ## nodes, actions, etc...
    lwuep_node = Node(
        package = pkg_name,
        executable= pkg_name + '_node',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )
    ## ld.add_xxx
    ld.add_action(lwuep_node)
    
    return ld  