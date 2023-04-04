import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument(
        name='port',
        default_value='this_is_a_port',
        description='Filters Configuration'))
    
    config = os.path.join(
      get_package_share_directory('oculus_ros2'),
      'cfg',
      'default.yaml'
      )
   
    oculus_sonar_node = Node(
         package='oculus_ros2',
         executable='oculus_sonar_node',
         name='oculus_sonar',
        #  parameters=[config],
         remappings=[
                 ('ping', '/oculus_sonar/ping'), # Topic name where ping messages are published (cf Oculus.h).
                 ('status', '/oculus_sonar/status') # Topic name where status messages are published (cf Oculus.h).
             ],
        arguments=['-port', LaunchConfiguration('port')],
        output='screen'
      )   
    
    image_publisher_node = Node(
         package='oculus_ros2',
         executable='oculus_subscriber_to_image.py',
         name='oculus_subscriber_to_image',
        output='screen'
      )
    
    rqt_reconfigure_node = Node(
         package='rqt_reconfigure',
         executable='rqt_reconfigure',
         name='rqt_reconfigure',
        output='screen'
      )
    
    
    ld.add_action(oculus_sonar_node)
    ld.add_action(image_publisher_node)
    # ld.add_action(rqt_reconfigure_node)

    return ld
