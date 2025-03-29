from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 獲取 roarm_description 的目錄
    roarm_description_dir = get_package_share_directory("roarm_description")

    # 定義 Launch 參數
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            roarm_description_dir, 'urdf', "roarm.urdf.xacro"
        ),
        description="Absolute path to robot URDF file"
    )

    # 設置 Gazebo 資源路徑
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(roarm_description_dir).parent.resolve())
        ]
    )

    # 加載 URDF 文件
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    # 啟動 robot_state_publisher 節點
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    # 啟動 Gazebo 模擬器
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py"
            )
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf"])
        ]
    )

    # 啟動 Gazebo 實體生成節點
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "roarm"]
    )

    # 啟動 Gazebo 與 ROS 2 的橋接節點
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_ros2_bridge,
        gz_spawn_entity
    ])
