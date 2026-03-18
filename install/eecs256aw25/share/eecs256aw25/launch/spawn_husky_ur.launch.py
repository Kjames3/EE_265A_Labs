import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_eecs256aw25 = get_package_share_directory('eecs256aw25')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    husky_urdf_path = os.path.join(pkg_eecs256aw25, 'models', 'husky_description', 'urdf', 'husky.urdf.xacro')
    husky_ur_extras_path = os.path.join(pkg_eecs256aw25, 'models', 'husky_ur_extras.urdf.xacro')
    
    # Process Xacro
    # We substitute $(find eecs256aw25) with the actual path because xacro needs it.
    # However, since we patched the xacro files to use $(find eecs256aw25), we rely on Command finding the package.
    # Note: 'eecs256aw25' must be in the package path.
    
    # Set Gazebo Sim resource path
    # We need to add the parent of the package share directory to allow 'package://eecs256aw25' resolution
    # AND the models directory for direct model lookups if needed.
    install_dir = get_package_share_directory('eecs256aw25')
    install_share_parent = os.path.dirname(install_dir)
    models_dir = os.path.join(install_dir, 'models')

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + install_share_parent + ':' + models_dir
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = install_share_parent + ':' + models_dir

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        husky_urdf_path,
        ' ',
        'urdf_extras:=', husky_ur_extras_path,
        ' ',
        'name:=husky',
        ' ',
        'prefix:=',
        ' ',
        'gazebo_controllers:=', PathJoinSubstitution([install_dir, 'models', 'husky_ur_control.yaml']),
        ' ',
        'is_sim:=true',
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'husky',
            '-topic', 'robot_description',
            '-z', '0.5',
            '-J', 'ur_arm_shoulder_pan_joint', '0.0',
            '-J', 'ur_arm_shoulder_lift_joint', '-1.57',
            '-J', 'ur_arm_elbow_joint', '0.0'
        ],
        output='screen',
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawners
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # spawn_ur_manipulator_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['ur_manipulator_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )

    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        spawn_joint_state_broadcaster,
        spawn_husky_velocity_controller,
        # spawn_ur_manipulator_controller
    ])
