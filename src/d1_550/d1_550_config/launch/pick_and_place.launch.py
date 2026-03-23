from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("d1_550_description", package_name="d1_550_config").to_moveit_configs()

    # MTC Demo node
    pick_and_place = Node(
        package="d1_550_config",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([pick_and_place])