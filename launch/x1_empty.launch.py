import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg = get_package_share_directory('ign_simulation')
    
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
      launch_arguments={'ign_args': os.path.join(pkg, 'worlds', 'empty.sdf')}.items(),
     )
  
  #  # Parse robot description from xacro
  #   robot_description_file =  os.path.join(pkg, 'models', 'turtlebot3_description', 'urdf', 'turtlebot3_burger.urdf')
  #   robot_description_config = xacro.process_file(
  #       robot_description_file
  #   )
  #   robot_description = {"robot_description": robot_description_config.toxml()}

    # # Robot state publisher
    # robot_state_publisher = Node(
    #   package='robot_state_publisher',
    #   executable='robot_state_publisher',
    #   name='robot_state_publisher',
    #   output='both',
    #   parameters=[robot_description],)


    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'x1_ugv',
                    #'-topic', 'robot_description',
                    '-file',  os.path.join(pkg, 'models', 'X1',
                                          'model.sdf')
                    ],
                output='screen',
                )
    
    # bridge = Node(
    #   package='ros_ign_bridge',
    #   executable='parameter_bridge',
    #   arguments=[
    #             # Joint states (IGN -> ROS2)
    #            '/world/empty/model/turtlebot3_burger/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
    #            '/world/empty/model/turtlebot3_burger/link/base_footprint/sensor/imu/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
    #             ],
    #   remappings=[
    #         ('/world/empty/model/turtlebot3_burger/joint_state', 'joint_states'),
    #         ('/world/empty/model/turtlebot3_burger/link/base_footprint/sensor/imu/imu', 'imu'),
    #     ],
    #     output='screen')

    # rviz = Node(
    #   package='rviz2',
    #   executable='rviz2',
    #   arguments=['-d', os.path.join(pkg,'models', 'turtlebot3_description', 'rviz', 'model.rviz')]
    # )
            
    return LaunchDescription([
        ign_gazebo,
       # robot_state_publisher,
        spawn,
        # bridge,
       # rviz,
    ])

