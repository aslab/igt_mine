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
      #launch_arguments={'ign_args': os.path.join(pkg, 'worlds', 'empty.sdf')}.items(),
     )

    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'x1_ugv',
                    '-file',  os.path.join(pkg, 'models', 'x1', 'model.sdf')
                    ],
                output='screen',
                )
    
    bridge = Node(
      package='ros_ign_bridge',
      executable='parameter_bridge',
      arguments=[
      # Mappings (IGN -> ROS2)
      # The ROS message type is followed by an @, [, or ] symbol where:

      # @ is a bidirectional bridge.
      # [ is a bridge from Ignition to ROS.
      # ] is a bridge from ROS to Ignition.
      # https://github.com/ignitionrobotics/ros_ign/tree/ros2/ros_ign_bridge

      '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
      '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

      '/model/x1_ugv/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
      '/model/x1_ugv/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
      '/model/x1_ugv/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

      '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
      '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',          

      '/camera_front/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
      '/camera_front/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
      '/camera_front/image@sensor_msgs/msg/Image[ignition.msgs.Image',

      '/camera_left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
      '/camera_left/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
      '/camera_left/image@sensor_msgs/msg/Image[ignition.msgs.Image',

      '/camera_rear/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
      '/camera_rear/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
      '/camera_rear/image@sensor_msgs/msg/Image[ignition.msgs.Image',

      '/camera_right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
      '/camera_right/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
      '/camera_right/image@sensor_msgs/msg/Image[ignition.msgs.Image',               
             
                ],
      remappings=[
            ('/imu', 'imu'),
        ],
        output='screen')
   
    return LaunchDescription([
      DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg, 'worlds', 'empty.sdf')]),
        ign_gazebo,
        spawn,
        bridge,
    ])

