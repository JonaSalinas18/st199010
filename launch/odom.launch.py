from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    launchDescription = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('3d-lidar-odometry'),
        'config',
        'params.yaml'
    )

    pointCloudProcessor = Node(
        package = "3d-lidar-odometry",
        executable = "point_cloud_processor_node",
        parameters = [config]
    )

    launchDescription.add_action(pointCloudProcessor)

    return launchDescription