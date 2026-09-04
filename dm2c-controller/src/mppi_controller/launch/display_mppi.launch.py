import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    urdf_pkg = get_package_share_directory("shensuo_robot_urdf")

    xacro_file = os.path.join(urdf_pkg, "urdf", "shensuo_robot.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={"use_gazebo": "false"})
    robot_desc = doc.toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="shensuo_robot",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    mppi_pkg = get_package_share_directory("mppi_controller")
    yaml_path = os.path.join(mppi_pkg, "mppi_controllers.yaml")

    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="shensuo_robot",
        parameters=[{"robot_description": robot_desc}],
        arguments=["--ros-args", "--params-file", yaml_path],
        output="screen",
    )

    spawn_jsb = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "-c", "/shensuo_robot/controller_manager",
        ],
        output="screen",
    )

    spawn_ctrl = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "mppi_controller",
            "-c", "/shensuo_robot/controller_manager",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="shensuo_robot",
        arguments=["-d", os.path.join(urdf_pkg, "rviz", "display.rviz")],
        output="screen",
    )

    return LaunchDescription([
        rsp,
        cm,
        TimerAction(period=3.0, actions=[spawn_jsb]),
        TimerAction(period=5.0, actions=[spawn_ctrl]),
        rviz,
    ])
