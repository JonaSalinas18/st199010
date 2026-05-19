from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	launchDescription = LaunchDescription()
	
	pointCloudProcessor = Node(
    	package="3d-lidar-odometry",
    	executable="point_cloud_processor_node",
	)

	#cargar parametros desde un archivo yaml
	

	#iniciar RViz2

	launchDescription.add_action(pointCloudProcessor)

	return launchDescription