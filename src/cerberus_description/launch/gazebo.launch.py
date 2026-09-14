import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = "cerberus_description"


    urdf_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value="cerberus_final.urdf",
        description="URDF filename inside <pkg>/urdf/",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.world",
        description="Gazebo world file to load (must be findable by gazebo_ros, "
                     "or give an absolute path)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock published by Gazebo instead of wall-clock time. "
                     "Keep this true for anything simulation-related - controllers, "
                     "TF, sensor timestamps all need to agree with Gazebo's clock.",
    )

    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Spawn X position (m)")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y position (m)")
    z_arg = DeclareLaunchArgument("z", default_value="0.40", description="Spawn Z position (m) - "
                                  "keep this above standing height so the robot drops onto the "
                                  "ground plane instead of spawning inside it")


    gazebo_model_path_env = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            PathJoinSubstitution([FindPackageShare(pkg_name), ".."]),
            ":",
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ],
    )


    urdf_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", LaunchConfiguration("urdf_file")]
    )


    robot_description_content = Command(["sed '1{/<?xml/d}' ", urdf_path])
    robot_description = {"robot_description": robot_description_content}


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )


    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_cerberus",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "cerberus",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
    )

    return LaunchDescription(
        [
            gazebo_model_path_env,
            urdf_arg,
            world_arg,
            use_sim_time_arg,
            x_arg,
            y_arg,
            z_arg,
            robot_state_publisher_node,
            gazebo,
            spawn_entity_node,
        ]
    )
