# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    package_name='diffdrive_arduino'
    
    #nav2_params_file = os.path.join(get_package_share_directory('nav2_bringup'),'params','nav2_params.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    ekf_params = os.path.join(get_package_share_directory('robot_localization'),'params','ekf.yaml')
    ukf_params = os.path.join(get_package_share_directory('robot_localization'),'params','ukf.yaml')
    imu_params = os.path.join(get_package_share_directory('ros2_mpu6050'),'config','params.yaml')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffdrive_arduino"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffdrive_arduino"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diffdrive_arduino"), "rviz", "diffbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    #launching LiDAR
    lidar = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource([os.path.join(
    	    get_package_share_directory(package_name),'launch','rp_lidar.launch.py'
    	)])
    ) 
    
    delayed_lidar = TimerAction(period = 2.0, actions = [lidar])  
    
    
    #SLAM toolboxv - are we still using this??
    online_async = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource([os.path.join(
    	    get_package_share_directory(package_name),'launch','online_async_launch.py'
    	)])
    )
    
    #launch the costmaps and all that nav2 stuff
    #nav2 = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #	    get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py')]),
    #	    launch_arguments={'use_sim_time':'false','namespace':'','params_file':nav2_params_file}.items()
    #)
    
    #delayed_nav2 = TimerAction(period = 10.0, actions = [nav2])
    
    #Convert LiDAR data to odometry
    lidar_odom = Node(
	package='lidar_odometry',
	executable='lidar_odometry_node',
	)
    
    #launch the MPU   
    imu_mpu6050 = IncludeLaunchDescription(
 	PythonLaunchDescriptionSource([os.path.join(
 	    get_package_share_directory('ros2_mpu6050'),'launch','ros2_mpu6050.launch.py')]),
	    launch_arguments={'params_file':imu_params}.items()
	)

    #twist_mux launch
    twist_mux = Node(
    	package='twist_mux',
    	executable='twist_mux',
    	remappings=[
    	     ('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')
    	     ],
    	parameters=[twist_mux_params]	
    	
    )
    
    delay_twist = TimerAction(period = 2.0, actions = [twist_mux])
    
    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
    	    get_package_share_directory('robot_localization'),'launch','ukf.launch.py')]),
    	    launch_arguments={'params_file':ukf_params}.items()
    )
   
    
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        #delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
	delayed_lidar,
	delay_twist,	
	#lidar_odom,
	imu_mpu6050,
	robot_localization

    ]

    return LaunchDescription(nodes)
