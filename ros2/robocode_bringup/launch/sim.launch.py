"""KR10 milling cell: Gazebo Harmonic (or mock hardware) + ros2_control + MoveIt 2 + RViz.

Replaces gazebo_moveit.launch.py of ClaBirk/ROS2_Gazebo_Moveit_Simulation (Humble + Gazebo Classic).

  ros2 launch robocode_bringup sim.launch.py                       # Gazebo + MoveIt + RViz
  ros2 launch robocode_bringup sim.launch.py sim:=mock             # without physics (fast, e.g. WSL without GPU)
  ros2 launch robocode_bringup sim.launch.py moveit:=false trajectory_file:=/path/Machining.ros_trajectory.json
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARM_CONTROLLER = "kr10_arm_controller"


def _setup(context):
    sim = LaunchConfiguration("sim").perform(context)
    if sim not in ("gz", "mock"):
        raise RuntimeError(f"sim:={sim} – allowed: gz, mock")
    use_moveit = LaunchConfiguration("moveit").perform(context).lower() == "true"
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)
    trajectory_file = LaunchConfiguration("trajectory_file").perform(context)
    time_scale = LaunchConfiguration("time_scale").perform(context)
    use_sim_time = sim == "gz"

    desc_share = get_package_share_directory("robocode_kr10_description")
    xacro_file = os.path.join(desc_share, "urdf", "kr10_cell.urdf.xacro")
    controllers_file = os.path.join(desc_share, "config", "ros2_controllers.yaml")

    from moveit_configs_utils import MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("kr10_cell", package_name="robocode_kr10_moveit_config")
        .robot_description(file_path=xacro_file, mappings={"sim_mode": sim, "controllers_file": controllers_file})
        .robot_description_semantic(file_path="config/kr10_cell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    sim_time = {"use_sim_time": use_sim_time}
    actions = []

    actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                        parameters=[moveit_config.robot_description, sim_time]))

    spawn_controllers = Node(package="controller_manager", executable="spawner", output="screen",
                             arguments=["joint_state_broadcaster", ARM_CONTROLLER,
                                        "--controller-manager", "/controller_manager"],
                             parameters=[sim_time])

    if sim == "gz":
        # meshes are referenced as package://robocode_kr10_description/...; Gazebo resolves them via this path
        actions.append(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(desc_share)))
        gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                                launch_arguments={"gz_args": f"-r {world}"}.items()))
        create = Node(package="ros_gz_sim", executable="create", output="screen",
                      arguments=["-topic", "robot_description", "-name", "kr10_cell", "-allow_renaming", "true"])
        actions.append(create)
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]))
        # controllers exist only after gz_ros2_control has loaded the spawned model
        actions.append(RegisterEventHandler(OnProcessExit(target_action=create, on_exit=[spawn_controllers])))
    else:
        actions.append(Node(package="controller_manager", executable="ros2_control_node", output="screen",
                            parameters=[controllers_file],
                            remappings=[("~/robot_description", "/robot_description")]))
        actions.append(spawn_controllers)

    if use_moveit:
        actions.append(Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                            parameters=[moveit_config.to_dict(), sim_time]))

    if use_rviz:
        if use_moveit:
            rviz_cfg = os.path.join(get_package_share_directory("robocode_kr10_moveit_config"), "config", "moveit.rviz")
            rviz_params = [moveit_config.robot_description, moveit_config.robot_description_semantic,
                           moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                           moveit_config.joint_limits, sim_time]
        else:
            rviz_cfg = os.path.join(desc_share, "rviz", "view.rviz")
            rviz_params = [sim_time]
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_cfg], parameters=rviz_params))

    if trajectory_file:
        actions.append(Node(package="robocode_ros", executable="trajectory_player", output="screen",
                            parameters=[{"trajectory_file": trajectory_file, "time_scale": float(time_scale),
                                         "use_sim_time": use_sim_time}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="gz", description="gz (Gazebo Harmonic) or mock (no physics)"),
        DeclareLaunchArgument("moveit", default_value="true", description="start move_group (OMPL + Pilz)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value="empty.sdf", description="Gazebo world file"),
        DeclareLaunchArgument("trajectory_file", default_value="",
                              description="roboCODE <name>.ros_trajectory.json to play after start"),
        DeclareLaunchArgument("time_scale", default_value="1.0",
                              description=">1 slows the roboCODE trajectory down (e.g. 2.0 = half speed)"),
        OpaqueFunction(function=_setup),
    ])
