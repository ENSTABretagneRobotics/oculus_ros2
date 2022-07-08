import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_oculus = get_package_share_directory('oculus_ros2')
    
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument(
        name='port',
        default_value='this_is_a_port',
        description='Filters Configuration'))
    
    # config = os.path.join(
    #   get_package_share_directory('oculus_ros2'),
    #   'cfg',
    #   'oculus_sonar.yaml'
    #   )
   
    oculus_sonar_node = Node(
         package='oculus_ros2',
         executable='oculus_sonar_node',
         name='oculus_sonar',
        #  parameters=[config],
        parameters=[{
          'ping_topic': 'ping', 
          'status_topic': 'status'
        }],
        arguments=['-port', LaunchConfiguration('port')],
        output='screen'
      )
    ld.add_action(oculus_sonar_node)

    return ld
