import os
from launch import LaunchDescription
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
        namespace="shensuo_robot",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        output="screen",
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="shensuo_robot",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="shensuo_robot",
        arguments=["-d", os.path.join(pkg, "rviz", "display.rviz")],
        output="screen",
    )

    return LaunchDescription([rsp, jsp, rviz])
