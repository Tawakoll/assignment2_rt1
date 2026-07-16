import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')

    # Bring up the 3D simulation (Gazebo + RViz + robot), which provides /scan and /cmd_vel
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py'),
        )
    )

    laser_status_node = Node(
        package='assignment2_rt1',
        executable='laser_status.py',
        name='laser_status',
        output='screen',
    )

    controller_node = Node(
        package='assignment2_rt1',
        executable='controller.py',
        name='controller',
        output='screen',
    )

    return LaunchDescription([
        simulation_launch,
        laser_status_node,
        controller_node,
    ])
