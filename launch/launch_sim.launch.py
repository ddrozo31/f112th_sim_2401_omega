# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='f112th_sim_2401_omega' #<--- CHANGE ME
    robot_name = 'carlikebot.urdf.xacro'
        # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description',robot_name)
    robot_description_config = xacro.process_file(xacro_file)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    

    # Create a robot_state_publisher node
    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}

    robot_state_pub_bicycle_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    robot_controllers = os.path.join(get_package_share_directory(package_name),'config','carlikebot_controllers.yaml')

    control_node_remapped = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/bicycle_steering_controller/tf_odometry", "/tf")])

    	# Launch the Diff_Controller
    diff_drive_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['bicycle_steering_controller',"--controller-manager", "/controller_manager"])
		
		# Launch the Joint_Broadcaster
    joint_broad_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['joint_state_broadcaster',"--controller-manager", "/controller_manager"])
    
    ekf_param_file = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_param_file,{'use_sim_time': True}]
)
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )



    # Launch them all!
    return LaunchDescription([
        robot_state_pub_bicycle_node,
        joystick,
        twist_mux_node,
        gazebo,
        spawn_entity,
        control_node_remapped,
        diff_drive_spawner,
        joint_broad_spawner,
        robot_localization_node,
    ])