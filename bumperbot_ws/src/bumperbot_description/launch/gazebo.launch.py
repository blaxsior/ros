import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
  bumperbot_description_dir = get_package_share_directory("bumperbot_description")
  # cli argument 정의
  model_arg = DeclareLaunchArgument(
    name="model",
    default_value=os.path.join(bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"),
    description="absolute path to robot URDF file"
  )
  # xacro => xacro to urdf. need model path 

  robot_description = ParameterValue(
    Command(["xacro ", LaunchConfiguration("model")]), #LaunchConfiguration: 런타임에 전달되는 값을 읽음
    value_type=str # 문자열 값으로 읽음
  )

  robot_state_publisher = Node(
    package="robot_state_publisher", 
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description}]
  )

  gazebo_resource_path = SetEnvironmentVariable(
    name="GZ_SIM_RESOURCE_PATH",
    value=[
      str(Path(bumperbot_description_dir).parent.resolve())
      ]
  )

  # 다른 launch 설정에서 가저오기
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"
      ]),
      # -r: 바로 시작 / -v 4 콘솔 출력 관련 / 파일 가져오기
      launch_arguments=[
        ("gz_args", [" -v 4", " -r", " empty.sdf"])
      ]
  )

  gz_spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    output="screen",
    arguments=["-topic", "robot_description",
               "-name", "bumperbot",
              ]
  )

  return LaunchDescription([
    model_arg,
    robot_state_publisher,
    gazebo_resource_path,
    gazebo,
    gz_spawn_entity
  ])