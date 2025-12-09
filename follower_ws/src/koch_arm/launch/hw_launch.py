import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuration and Argument Setup
    package_name = 'koch_arm'
    package_share_path = get_package_share_directory(package_name)
    config_dir = os.path.join(package_share_path, 'config')

    # Define paths to the controller and MoveIt configuration files
    controllers_yaml_path = os.path.join(config_dir, 'ros2_controllers.yaml')
    moveit_controllers_yaml_path = os.path.join(config_dir, 'moveit_controllers.yaml')
    kinematics_yaml_path = os.path.join(config_dir, 'kinematics.yaml')
    joint_limits_yaml_path = os.path.join(config_dir, 'joint_limits.yaml')
    
    # Argument to specify the initial positions file
    initial_positions_file = LaunchConfiguration('initial_positions_file', 
        default=os.path.join(config_dir, 'initial_positions.yaml'))
        
    # 2. Robot Description (URDF + ros2_control)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([config_dir, 'follower.urdf.xacro']),
        ' ',
        'initial_positions_file:=', initial_positions_file,
    ])
    robot_description = {'robot_description': robot_description_content}

    # 3. Hardware and State Nodes

    # Robot State Publisher Node: Publishes the URDF.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ROS 2 Control Node: Loads the DynamixelHardwareInterface and Controller Manager.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml_path],
        output="screen",
    )

    # 4. Controller Spawners (Sequenced with Event Handlers)

    # 4a. Joint State Broadcaster: Starts first to publish joint positions.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # 4b. Arm Controller Spawner: The controller MoveIt will command.
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # Handler to start the Arm Controller AFTER the Joint State Broadcaster finishes spawning.
    start_arm_controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # 5. MoveIt 2 Planning Node

    # Move Group Node: The core planning node.
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            {'moveit_controller_manager': moveit_controllers_yaml_path},
            {'kinematics_solver': kinematics_yaml_path},
            {'joint_limits': joint_limits_yaml_path},
        ],
    )

    # Handler to start the Move Group Node AFTER the Arm Controller finishes spawning.
    start_move_group_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    return LaunchDescription([
        # Declare the initial positions file argument
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value=initial_positions_file,
            description='YAML file with initial joint positions',
        ),

        # Start necessary nodes
        robot_state_publisher_node,
        control_node,
        
        # Start the sequence
        joint_state_broadcaster_spawner,
        start_arm_controller_handler,
        start_move_group_handler,
    ])