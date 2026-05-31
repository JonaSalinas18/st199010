from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    launchDescription = LaunchDescription()

    useRvizArg = LaunchConfiguration('useRviz')

    config = os.path.join(
        get_package_share_directory('st199010'),
        'config',
        'params.yaml'
    )

    declareLaunchArg = DeclareLaunchArgument(
        'useRviz',
        default_value='false',
        description='Launch RViz or not'
    )

    pointCloudProcessor = Node(
        package="st199010",
        executable="point_cloud_processor_node",
        parameters=[config, {'use_sim_time': True}]
    )

    rVizNode = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(useRvizArg),
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    launchDescription.add_action(declareLaunchArg)
    launchDescription.add_action(pointCloudProcessor)
    launchDescription.add_action(rVizNode)

    return launchDescription
