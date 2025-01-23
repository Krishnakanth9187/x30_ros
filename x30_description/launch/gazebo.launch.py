from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to true to enable Gazebo GUI'
    )
    package_arg = DeclareLaunchArgument(
        name='urdf_package',
        description='The package where the robot description is located',
        default_value='x30_description',
    )
    model_arg = DeclareLaunchArgument(
        name='urdf_package_path',
        description='The path to the robot description relative to the package root',
        default_value='urdf/X30.urdf',
    )

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
        }.items(),
    )

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path'),
        }.items()
    )

    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-z', '1'],
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        empty_world_launch,
        description_launch_py,
        urdf_spawner_node,
    ])
