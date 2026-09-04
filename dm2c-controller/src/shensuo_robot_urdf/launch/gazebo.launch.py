import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = get_package_share_directory('shensuo_robot_urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    xacro_file = os.path.join(pkg_share, 'urdf', 'shensuo_robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_gazebo': 'true'})
    robot_desc = doc.toxml()

    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'models') + ':' + os.path.dirname(pkg_share)
    )

    cyclone_dds = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value=os.path.expanduser('~/.ros/cyclonedds.xml')
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'shensuo_robot',
                   '-allow_renaming', 'true', '-z', '0.05'],
    )

    load_controllers = []
    for ctrl in ['joint_state_broadcaster',
                 'left_arm_controller', 'right_arm_controller',
                 'left_gripper_controller', 'right_gripper_controller']:
        load_controllers.append(ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner', ctrl],
            output='screen',
        ))

    detach_left = ExecuteProcess(
        cmd=['ign', 'topic', '-t', '/left_gripper/detach',
             '-m', 'ignition.msgs.Empty', '-p', ''],
        output='screen',
    )
    detach_right = ExecuteProcess(
        cmd=['ign', 'topic', '-t', '/right_gripper/detach',
             '-m', 'ignition.msgs.Empty', '-p', ''],
        output='screen',
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[('gz_args', ['-r ', world_file])],
    )

    IKNode = Node(
        package="arm_ik_solver",
        executable="dual_arm_node",
    )

    return LaunchDescription([
        IKNode,
        cyclone_dds,
        gz_resource_path,
        gz,
        robot_state_publisher,
        spawn_entity,
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='If true, use simulated clock'),
    ] + load_controllers + [
        TimerAction(period=6.0, actions=[detach_left]),
        TimerAction(period=6.0, actions=[detach_right]),
    ])
