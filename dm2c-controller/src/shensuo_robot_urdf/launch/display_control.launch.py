import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("shensuo_robot_urdf")

    xacro_file = os.path.join(pkg, "urdf", "shensuo_robot.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_gazebo': 'false'})
    robot_desc = doc.toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        output="screen",
    )

    controllers_yaml = os.path.join(pkg, "config", "ros2_controllers.yaml")

    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        arguments=["--ros-args", "--params-file", controllers_yaml],
        output="screen",
    )

    spawn_jsb = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
             "-c", "/controller_manager", "--controller-manager-timeout", "30"],
        output="screen",
    )
    spawn_controllers = []
    for ctrl in ["left_arm_controller", "right_arm_controller",
                 "left_gripper_controller", "right_gripper_controller"]:
        spawn_controllers.append(ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", ctrl,
                 "-c", "/controller_manager", "--controller-manager-timeout", "30"],
            output="screen",
        ))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg, "rviz", "display.rviz")],
        output="screen",
    )

    IKNode = Node(
        package="arm_ik_solver",
        executable="dual_arm_node",
    )

    return LaunchDescription([
        rsp,
        cm,
        TimerAction(period=3.0, actions=[spawn_jsb]),
        TimerAction(period=5.0, actions=spawn_controllers),
        rviz,
        IKNode
    ])
