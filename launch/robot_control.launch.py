import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('simple_3dof_arm')
    world_file = os.path.join(pkg_dir, 'worlds', 'simple_table_world.world')
    
    # Set Gazebo model path
    gazebo_model_path = os.path.join(pkg_dir, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    
    return LaunchDescription([
        gazebo,
    ])
