import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import TimerAction


def generate_launch_description():

    pkg_name = "cerberus_description"


    urdf_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value="cerberus_final.urdf",
        description="URDF filename inside <pkg>/urdf/",
    )

    world_arg = DeclareLaunchArgument(
    "world",
    default_value=PathJoinSubstitution(
    [FindPackageShare(pkg_name), "worlds", "cerberus.world"]
    ),
    description="Gazebo world file to load",
)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock published by Gazebo instead of wall-clock time. "
                     "Keep this true for anything simulation-related - controllers, "
                     "TF, sensor timestamps all need to agree with Gazebo's clock.",
    )

    start_paused_arg = DeclareLaunchArgument(
        "start_paused",
        default_value="false",
        description="Pause Gazebo at startup (leave false so controllers can activate).",
    )

    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Spawn X position (m)")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y position (m)")
    z_arg = DeclareLaunchArgument("z", default_value="0.205", description="Spawn Z position (m) - "
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

    # controller_manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     name="controller_manager",
    #     output="screen",
    #     parameters=[
    #         PathJoinSubstitution([FindPackageShare(pkg_name), "config", "controller_manager.yaml"]),
    #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
    #     ],
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    stand_pose_node = Node(
        package=pkg_name,
        executable="stand_pose_node.py",
        name="stand_pose",
        output="screen",
    )

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
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "pause": LaunchConfiguration("start_paused"),
        }.items(),
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

    delayed_spawn_entity = TimerAction(
        period=5.0,
        actions=[spawn_entity_node],
    )

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                TimerAction(period=0.5, actions=[joint_trajectory_controller]),
                TimerAction(period=1.5, actions=[stand_pose_node]),
            ],
        )
    )
    start_jsb_after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn_entity_node,on_exit=[joint_state_broadcaster_spawner]))
    start_jtc_after_jsb = RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner,on_exit=[joint_trajectory_controller]))
    start_stand_after_jtc = RegisterEventHandler(OnProcessExit(target_action=joint_trajectory_controller,on_exit=[stand_pose_node]))
            


    return LaunchDescription(
        [
            gazebo_model_path_env,
            urdf_arg,
            world_arg,
            use_sim_time_arg,
            start_paused_arg,
            x_arg,
            y_arg,
            z_arg,
            gazebo,
            delayed_spawn_entity,
            robot_state_publisher_node,
            start_jsb_after_spawn,
            start_jtc_after_jsb,
            start_stand_after_jtc,
            ]
    )
