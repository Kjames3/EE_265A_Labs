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
        'prefix:='
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Spawn Entity
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'husky',
            '-topic', 'robot_description',
            '-z', '0.5'
        ],
        output='screen',
    )
    
    # Bridge (Optional: for clock and basic transforms if needed, but RSP handles TF)
    # Usually we need a bridge for /clock if we want identifying simulation time.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        bridge
    ])
