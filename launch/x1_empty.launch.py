import os
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg = get_package_share_directory('x1_simulation')
    urdf_path = pkg + '/models/x1/urdf/x1_from_sdf.urdf'
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = os.path.join(pkg, 'rviz', 'urdf_config.rviz')
    
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
      #launch_arguments={'ign_args': os.path.join(pkg, 'worlds', 'empty.sdf')}.items(),
     )

    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'X1',
                    '-file',  os.path.join(pkg, 'models', 'x1', 'model.sdf'),
                    '-x', '-16.857300',
                    '-y', '-13.665100',
                    '-z', '-0.183275',
                    '-R', '0.114033',
                    '-P', '0.0275442',
                    '-Y', '-0.904919',],
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

      '/model/X1/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
      '/model/X1/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
      '/model/X1/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
      '/world/empty/model/X1/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',

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
            ('/cmd_vel', 'cmd_vel'),
            ('/world/empty/model/X1/joint_state', 'joint_states'),
            ('/model/X1/tf', 'tf'),
            ('/model/X1/odometry', 'odom'),
        ],
        output='screen')


    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
				output='screen',
				parameters = [
					{'ignore_timestamp': False},
                                        {'use_sim_time': use_sim_time},
					{'use_tf_static': True},
					{'robot_description': open(urdf_path).read()}],
				arguments = [urdf_path])	

    rviz2 = Node(package='rviz2', executable='rviz2',
					name='rviz2',
					arguments=['-d', rviz_config_file],
					parameters=[{'use_sim_time': True}],
					output='screen',)

   
    return launch.LaunchDescription([
      DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg, 'worlds', 'cave_world.sdf')]),
      DeclareLaunchArgument('use_sim_time', default_value=['true'],
                    description='Enable sim time from /clock'),
        ign_gazebo,
        spawn,
        bridge,
        state_publisher,
        rviz2,
    ])

