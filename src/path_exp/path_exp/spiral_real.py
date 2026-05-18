#### SE 494, Fall 2025
### Written by Junyang Guan
###  lloyd9179@gmail.com
##### University of Illinois Urbana Champaign

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
import numpy as np
import math

# MoveIt related messages, services, and actions
from moveit_msgs.msg import MoveItErrorCodes, Constraints, JointConstraint, RobotTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath # CORRECTED IMPORT
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration # ADDED FOR RESCALING

#########
# position unit: m
# angle unit: rad

# --------------------------------------------------------------------------------
# Helper function to translate MoveIt error codes to a readable string
def moveit_error_code_to_string(val):
    """Translates a MoveItErrorCodes message to a string for debugging."""
    if val == MoveItErrorCodes.SUCCESS:
        return "SUCCESS"
    if val == MoveItErrorCodes.FAILURE:
        return "FAILURE"
    error_dict = {k: v for k, v in MoveItErrorCodes.__dict__.items() if k.isupper()}
    for name, value in error_dict.items():
        if value == val:
            return name
    return "UNKNOWN_ERROR_CODE"

# --------------------------------------------------------------------------------
# ** NEW HELPER FUNCTION FOR TRAJECTORY RETIMING **
def rescale_trajectory_time(trajectory: RobotTrajectory, time_scaling_factor: float) -> RobotTrajectory:
    """
    Rescales a trajectory's timestamps and correctly updates velocities and accelerations.
    
    :param trajectory: The input RobotTrajectory to rescale.
    :param time_scaling_factor: The factor to scale the time by. > 1.0 slows down, < 1.0 speeds up.
    :return: A new RobotTrajectory with rescaled timing.
    """
    if time_scaling_factor == 1.0:
        return trajectory

    new_trajectory = RobotTrajectory()
    new_trajectory.joint_trajectory.header = trajectory.joint_trajectory.header
    new_trajectory.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names
    num_time = 0
    for point in trajectory.joint_trajectory.points:
        new_point = point
        

        # Rescale time
        original_time_from_start = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        new_time_from_start = original_time_from_start * time_scaling_factor # new time t' = t*s
        print("this is the og time",original_time_from_start)
        print("this is the new time",new_time_from_start)

        num_time = num_time+1
        new_point.time_from_start = Duration(
            sec=int(new_time_from_start),
            nanosec=int((new_time_from_start % 1) * 1e9)
        )


        # Rescale velocities (v' = v / s)
        # this is math: v = ds/dt, since t'=f*t, thus for the new scaled time t', ds/dt' = ds/(s*dt) --> v' = v/s
        if point.velocities:
            new_point.velocities = [v / time_scaling_factor for v in point.velocities]

        # Rescale accelerations (a' = a / s^2)
        #same
        if point.accelerations:
            new_point.accelerations = [a / (time_scaling_factor**2) for a in point.accelerations]
        
        new_trajectory.joint_trajectory.points.append(new_point)
    print(num_time)    
    return new_trajectory
# --------------------------------------------------------------------------------


class SpiralController(Node):

    def __init__(self):
        super().__init__('spiral_controller_node')

        # --- Parameters ---
        self.move_group_name = "ur_manipulator"
        self.end_effector_link = "wrist_3_link" # Make sure this is your end-effector link
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        # --- ROS 2 Communications ---
        # Action Client: For point-to-point joint moves (e.g., to go to start pose)
        self._move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        # Action Client: For executing a pre-planned trajectory
        self._execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        # Service Client: To request a Cartesian path plan from MoveIt
        self._cartesian_path_service_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        # Subscriber: To get the robot's current joint states
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # --- Internal State Variables ---
        self.current_joint_state = None
        self.get_logger().info("Node initialized.")

    def joint_state_callback(self, msg: JointState):
        """Callback function to continuously receive and store the robot's current joint state."""
        self.current_joint_state = msg

    def wait_for_ready(self):
        """Ensures that all required action and service servers are available before proceeding."""
        self.get_logger().info("Waiting for all services to be ready...")
        self._move_group_action_client.wait_for_server()
        self._execute_trajectory_action_client.wait_for_server()
        self._cartesian_path_service_client.wait_for_service()
        
        self.get_logger().info(" - Waiting to receive /joint_states message...")
        while self.current_joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("All services are ready! Ready to plan.")
        return True

    def plan_and_execute_joint_goal(self, goal_joint_positions):
        """
        [Reused from your code] Plans and executes a goal in joint space.
        We will use this to move the robot to a known starting position.
        """
        self.get_logger().info(f"Planning to joint goal: {goal_joint_positions}")

        if len(goal_joint_positions) != len(self.joint_names):
            self.get_logger().error("Number of goal positions does not match number of joints!")
            return False

        goal_msg = MoveGroup.Goal()
        request = goal_msg.request
        request.group_name = self.move_group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.start_state.joint_state = self.current_joint_state
        request.start_state.is_diff = True
        
        goal_constraints = Constraints()
        for i, name in enumerate(self.joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = goal_joint_positions[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0 ###################################
            goal_constraints.joint_constraints.append(joint_constraint)
        request.goal_constraints.append(goal_constraints)

        self.get_logger().info("Sending goal to MoveGroup Action Server...")
        future = self._move_group_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server!")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        error_string = moveit_error_code_to_string(result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"Motion succeeded! Result: {error_string}")
            return True
        else:
            self.get_logger().error(f"Motion failed! Result: {error_string}")
            return False

    def plan_and_execute_spiral(self):
        """
        [MODIFIED] Plans and executes the spiral trajectory.
        """
        # --- 1. Generate Waypoints for the Spiral ---
        self.get_logger().info("Generating spiral waypoints...")
        waypoints = []
        
        # --- MODIFICATION 3: SPIRAL ---#####################################################
        # The center of the spiral
        center_x, center_y, center_z = 0.3, 0.2, 0.8
        
        # Spiral parameters
        radius = 0.15         # Radius of the spiral circle
        num_points = 250      # Number of points to define the path
        num_revolutions = 5   # How many times the spiral goes around
        x_increment = 0.3     # Total linear motion along the X-axis
        # --- END OF MODIFICATION 3 ---#######################################################
###########################################################################################################
        for i in range(num_points + 1):
            angle = 2 * np.pi * num_revolutions * (i / num_points)
            
            p = Pose()
            # --- MODIFICATION 3: REORIENTED SPIRAL MATH ---
            # Spiral is now in the YZ plane
            p.position.y = center_y + radius * np.cos(angle)
            p.position.z = center_z + radius * np.sin(angle)
            # Linear motion is now along the X-axis
            p.position.x = center_x + (x_increment * i / num_points)
            ####################################################################################
            # Keep the end-effector orientation constant (pointing to yz plane)
            p.orientation.x = 0.0
            p.orientation.y = 0.70710678  # sin(90_degrees / 2)
            p.orientation.z = 0.0
            p.orientation.w = 0.70710678  # cos(90_degrees / 2)
            waypoints.append(p)
        self.get_logger().info(f"Generated {len(waypoints)} waypoints successfully.")

        # --- 2. Request a Cartesian Path Plan from MoveIt ---
        self.get_logger().info("Calling /compute_cartesian_path service...")
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.start_state.joint_state = self.current_joint_state
        request.group_name = self.move_group_name
        request.waypoints = waypoints
        request.max_step = 0.01     # Interpolation step size
        request.jump_threshold = 0.0 # Disable jump detection

        future = self._cartesian_path_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        fraction = response.fraction
        self.get_logger().info(f"Cartesian path computed with fraction: {fraction * 100.0:.2f}%")

        if fraction < 0.9:
            self.get_logger().error("Could not compute a complete Cartesian path. Cancelling motion.")
            return False
            
        # --- MODIFICATION 2: SPEED CONTROL (Corrected Method) ---
        self.get_logger().info("Rescaling trajectory time for slower motion...")

        ################################################################################################
        # A time_scaling_factor of 3.0 means the trajectory will take 3x longer to complete (1/3 speed).
        time_scaling_factor = 5.0 
        rescaled_trajectory = rescale_trajectory_time(response.solution, time_scaling_factor)
        # --- END OF MODIFICATION 2 ---

        # --- 3. Execute the (Rescaled) Trajectory ---
        self.get_logger().info("Planning successful. Preparing to execute trajectory...")
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = rescaled_trajectory # Use the rescaled trajectory
        
        execute_future = self._execute_trajectory_action_client.send_goal_async(execute_goal)
        rclpy.spin_until_future_complete(self, execute_future)
        
        goal_handle = execute_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution was rejected!")
            return False

        self.get_logger().info("Trajectory execution in progress...")
        execute_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, execute_result_future)
        
        final_result = execute_result_future.result().result
        if final_result.error_code.val == MoveItErrorCodes.SUCCESS:
             self.get_logger().info("Spiral trajectory execution successful!")
             return True
        else:
            error_string = moveit_error_code_to_string(final_result.error_code.val)
            self.get_logger().error(f"Spiral trajectory execution failed: {error_string}")
            return False

# --------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    spiral_controller = SpiralController()
    
    # Place the rclpy.spin() call in a separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(spiral_controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # --- Main Logic Execution ---
        if spiral_controller.wait_for_ready():
            # Define the starting joint configuration for the robot
            start_joint_goal = [math.radians(0), math.radians(-111), math.radians(124), 
                                math.radians(145), math.radians(-103), math.radians(0)]

            # 1. First, move to a known, safe starting joint position
            spiral_controller.get_logger().info("--- Step 1: Moving to Start Position ---")
            success_to_start = spiral_controller.plan_and_execute_joint_goal(start_joint_goal)
            
            if success_to_start:
                # 2. Then, execute the spiral motion from the current position
                spiral_controller.get_logger().info("\n--- Step 2: Executing Spiral Motion ---")
                success_spiral = spiral_controller.plan_and_execute_spiral()
                
                # --- MODIFICATION 1: RETURN TO START ---
                if success_spiral:
                    spiral_controller.get_logger().info("\n--- Step 3: Returning to Start Position ---")
                    spiral_controller.plan_and_execute_joint_goal(start_joint_goal)
                # --- END OF MODIFICATION 1 ---

            else:
                spiral_controller.get_logger().error("Failed to move to start position. Cancelling spiral motion.")

    except KeyboardInterrupt:
        spiral_controller.get_logger().info("Keyboard interrupt, shutting down.")
    
    spiral_controller.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()