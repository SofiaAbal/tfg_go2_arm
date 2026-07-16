import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "d1_550_description", package_name="d1_550_config"
    ).to_moveit_configs()

    # Switch ALL ROS2 nodes to FastDDS so CycloneDDS is only used by the
    # Unitree SDK inside d1_bridge — zero conflict between the two DDS stacks.
    set_fastdds = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    rviz_config_file = os.path.join(
        get_package_share_directory("d1_550_config"),
        "launch",
        "mtc.rviz",
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("d1_550_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Python bridge: uses the cyclonedds Python package (bundled libddsc 11.x)
    # which has no iceoryx shared-memory conflict with the Unitree SDK.
    # Network interface is fixed in d1_550_driver/config/cyclonedds.xml,
    # pointed to via CYCLONEDDS_URI (no XML built in Python).
    cyclonedds_config = os.path.join(
        get_package_share_directory("d1_550_driver"),
        "config",
        "cyclonedds.xml",
    )
    d1_driver = Node(
        package="d1_550_driver",
        #executable="custom_d1_driver.py",
        executable="d1_driver.py",
        output="both",
        #additional_env={"D1_IFACE": "enx00e04c681034", "CYCLONEDDS_URI": f"file://{cyclonedds_config}"},
        additional_env={"D1_IFACE": "enx00e04c681034", "CYCLONEDDS_URI": f"file://{cyclonedds_config}"},
    )

    load_controllers = []
    for controller in [
        "d1_arm_controller",
        "d1_gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            set_fastdds,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            d1_driver,
        ]
        + load_controllers
    )
