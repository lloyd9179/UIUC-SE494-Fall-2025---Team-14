import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # --- Declare launch arguments ---
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur10e",
        description="Type of UR robot to use (e.g., ur3, ur5, ur10e)",
    )

    # --- Get paths to configuration files ---
    ur_moveit_config_pkg = get_package_share_directory("ur_moveit_config")
    ur_description_pkg = get_package_share_directory("ur_description")
    
    # --- Define path to the ros2_controllers.yaml ---
    ros2_controllers_yaml = os.path.join(ur_moveit_config_pkg, "config", "ros2_controllers.yaml")

    # --- Manually load robot description (URDF) ---
    # THIS IS THE FINAL FIX: We now pass ALL required arguments, including 'name'.
    robot_description_content = ParameterValue(
        Command([
            "xacro",
            " ",
            os.path.join(ur_description_pkg, "urdf", "ur.urdf.xacro"),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            LaunchConfiguration("ur_type"),
            " ",
            "ros2_control_params_file:=",
            ros2_controllers_yaml,
            " ",
            "use_fake_hardware:=true", # Critical for simulation
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # --- Manually load semantic robot description (SRDF) ---
    robot_description_semantic_content = ParameterValue(
        Command([
            "xacro",
            " ",
            os.path.join(ur_moveit_config_pkg, "config", "ur.srdf.xacro"),
            " ",
            "name:=", 
            "ur",
            " ",
            "ur_type:=", 
            LaunchConfiguration("ur_type"),
        ]),
        value_type=str,
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # --- Manually load kinematics configuration ---
    kinematics_yaml = os.path.join(ur_moveit_config_pkg, "config", "kinematics.yaml")

    # --- Manually load OMPL planning pipeline configuration ---
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planning_request_adapters/AddTimeOptimalParameterization default_planning_request_adapters/FixWorkspaceBounds default_planning_request_adapters/FixStartStateBounds default_planning_request_adapters/FixStartStateCollision default_planning_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = os.path.join(ur_moveit_config_pkg, "config", "ompl_planning.yaml")

    # --- Define parameters for the MoveGroup node ---
    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        ompl_planning_pipeline_config,
        ompl_planning_yaml,
        {"use_sim_time": True},
    ]

    # --- ros2_control setup ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml],
        output="screen",
    )

    # --- Spawn controllers ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # --- MoveGroup node ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
    )

    # --- RViz node ---
    rviz_config = os.path.join(ur_moveit_config_pkg, "launch", "view_robot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description, robot_description_semantic, ompl_planning_pipeline_config, kinematics_yaml],
    )
    
    # --- Static TF publisher ---
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    return LaunchDescription(
        [
            ur_type_arg,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            move_group_node,
            rviz_node,
            static_tf_node,
        ]
    )

