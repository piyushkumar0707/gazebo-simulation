import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('simple_3dof_arm')
    world_file = os.path.join(pkg_dir, 'worlds', 'simple_table_world.world')
    model_path = os.path.join(pkg_dir, 'models')
    
    return LaunchDescription([
        # Set Gazebo model path
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        
        # Launch Gazebo with world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),
    ])
