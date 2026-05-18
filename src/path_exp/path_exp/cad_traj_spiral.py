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
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion
### NEW IMPORT ###
# Added the Duration message type needed for the new rescaling function
from builtin_interfaces.msg import Duration 

# In your ROS 2 environment, you can install scipy with:
# python3 -m pip install scipy
from scipy.spatial.transform import Rotation

# --------------------------------------------------------------------------------
# Helper function to translate MoveIt error codes to a readable string
def moveit_error_code_to_string(val):
    """Translates a MoveItErrorCodes message to a string for debugging."""
    # This function is unchanged.
    if val == MoveItErrorCodes.SUCCESS:
        return "SUCCESS"
    if val == MoveItErrorCodes.FAILURE:
        return "FAILURE"
    error_dict = {k: v for k, v in MoveItErrorCodes.__dict__.items() if k.isupper()}
    for name, value in error_dict.items():
        if value == val:
            return name
    return "UNKNOWN_ERROR_CODE"

### NEW HELPER FUNCTION ###
# This is the advanced time-scaling function you provided.
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
    
    for point in trajectory.joint_trajectory.points:
        new_point = point
        
        # Rescale time_from_start
        original_time_from_start = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        new_time_from_start = original_time_from_start * time_scaling_factor
        
        new_point.time_from_start = Duration(
            sec=int(new_time_from_start),
            nanosec=int((new_time_from_start % 1) * 1e9)
        )

        # Rescale velocities (v' = v / s)
        if point.velocities:
            new_point.velocities = [v / time_scaling_factor for v in point.velocities]

        # Rescale accelerations (a' = a / s^2)
        if point.accelerations:
            new_point.accelerations = [a / (time_scaling_factor**2) for a in point.accelerations]
        
        new_trajectory.joint_trajectory.points.append(new_point)
        
    return new_trajectory

def get_orientation(normal, tangent):
    """
    Calculates the quaternion to align the tool's Z-axis with the normal vector.
    Uses the path's tangent vector to stabilize the tool's "twist".
    """
    # This function is unchanged.
    z_axis = np.array(normal) / np.linalg.norm(normal)
    t_vec = np.array(tangent)
    if np.linalg.norm(t_vec) > 1e-6:
        t_vec = t_vec / np.linalg.norm(t_vec)
    #else: 
    #    t_vec = np.array([1.0, 0.0, 0.0])
        
    y_axis = np.cross(z_axis, t_vec)
    
    if np.linalg.norm(y_axis) < 1e-6:
        t_vec = np.array([1.0, 0.0, 0.0])
        y_axis = np.cross(z_axis, t_vec)
        if np.linalg.norm(y_axis) < 1e-6:
            t_vec = np.array([0.0, 1.0, 0.0])
            y_axis = np.cross(z_axis, t_vec)
            
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
    r = Rotation.from_matrix(rotation_matrix)
    quat = r.as_quat()

    q_msg = Quaternion()
    q_msg.x = quat[0]
    q_msg.y = quat[1]
    q_msg.z = quat[2]
    q_msg.w = quat[3]
    return q_msg

# --------------------------------------------------------------------------------
class SpiralController(Node):
    # The __init__, joint_state_callback, wait_for_ready, and 
    # plan_and_execute_joint_goal methods are all unchanged.
    def __init__(self):
        super().__init__('spiral_controller_node')
        self.move_group_name = "ur_manipulator"
        self.end_effector_link = "wrist_3_link"
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self._move_group_action_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._cartesian_path_service_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.current_joint_state = None
        self.get_logger().info("Node initialized.")

    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def wait_for_ready(self):
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
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        request.goal_constraints.append(goal_constraints)
        future = self._move_group_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server!")
            return False
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
        # This is the failing code version, now with the new speed control.
        # --- 1. DEFINE GEOMETRY & PATH PARAMETERS ---
        self.get_logger().info("Defining geometry based on precise model...")
        INCHES_TO_METERS = 0.0254

        # GEOMETRY (in METERS)
        h_c = (31.28 - 28.05) / 2 * INCHES_TO_METERS
        h_e = 28.05 / 2 * INCHES_TO_METERS
        R_cyl = (110.25 / 4) * INCHES_TO_METERS
        a = R_cyl
        b = h_e
        a_sq = a**2
        b_sq = b**2

        # POSITIONING
        OBJECT_CENTER_M = np.array([0.0, 0.5, a]) 
        
        # PATH PARAMETERS
        num_points = 300
        num_revolutions = 5
        
        # --- 2. GENERATE WAYPOINTS (Poses: Position + Orientation) ---
        self.get_logger().info("Generating waypoints for cylinder and ellipsoid...")
        waypoints = []
        raw_points = []

        total_depth = h_c + h_e
        for i in range(num_points):
            progress = i / (num_points - 1)
            y = progress * total_depth  ### ??????????????????????????????????????????
            theta = progress * 2 * math.pi * num_revolutions
            
            if y <= h_c:
                radius = R_cyl
            else:
                y_shifted = y - h_c
                sqrt_arg = 1 - (y_shifted**2 / b_sq)
                if sqrt_arg < 0: continue
                radius = a * math.sqrt(sqrt_arg)
            ############################################
            #spiral
            x = radius * math.cos(theta)
            z = radius * math.sin(theta)
            #############################################
            raw_points.append(np.array([x, y, z]))

        for i in range(len(raw_points) - 1):
            p_current = raw_points[i]
            p_next = raw_points[i+1]
            x, y, z = p_current[0], p_current[1], p_current[2] ################################## tool_position
            
            if y <= h_c:
                normal_vector = np.array([2*x, 0, 2*z])
            else:
                y_shifted = y - h_c
                normal_vector = np.array([2*x / a_sq, 2*y_shifted / b_sq, 2*z / a_sq])
            
            tangent_vector = p_next - p_current
            tool_orientation = get_orientation(normal_vector, tangent_vector)
            tool_position = p_current + OBJECT_CENTER_M
            
            p = Pose()
            p.position.x = tool_position[0]
            p.position.y = tool_position[1]
            p.position.z = tool_position[2]
            p.orientation = tool_orientation
            waypoints.append(p)
            
        self.get_logger().info(f"Generated {len(waypoints)} waypoints successfully.")

        # --- 3. PLAN AND EXECUTE ---
        if not waypoints:
             self.get_logger().error("No waypoints generated. Aborting motion.")
             return False
             
        self.get_logger().info("Calling /compute_cartesian_path service...")
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.start_state.joint_state = self.current_joint_state
        request.group_name = self.move_group_name
        request.waypoints = waypoints
        request.max_step = 0.01
        request.jump_threshold = 0.0
        
        future = self._cartesian_path_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        fraction = response.fraction
        self.get_logger().info(f"Cartesian path computed with fraction: {fraction * 100.0:.2f}%")
        if fraction < 0.99:
            self.get_logger().error("Could not compute a complete Cartesian path. Cancelling motion.")
            return False
        
        ### MODIFIED ###
        # The old, simple time-scaling loop has been replaced with a call
        # to the new, more robust function.
        self.get_logger().info("Rescaling trajectory time for slower motion...")
        # A factor of 4.0 means the trajectory will take 4x longer (25% speed).
        time_scaling_factor = 4.0
        rescaled_trajectory = rescale_trajectory_time(response.solution, time_scaling_factor)
        ### END OF MODIFICATION ###

        self.get_logger().info("Planning successful. Preparing to execute trajectory...")
        execute_goal = ExecuteTrajectory.Goal()
        # Use the rescaled trajectory
        execute_goal.trajectory = rescaled_trajectory
        execute_future = self._execute_trajectory_action_client.send_goal_async(execute_goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = execute_future.result()
        if goal_handle is None:
            return False
        if not goal_handle.accepted:
            return False
        self.get_logger().info("Trajectory execution in progress...")
        execute_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, execute_result_future)
        final_result = execute_result_future.result().result
        
        if final_result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Surface trajectory execution successful!")
            return True
        else:
            error_string = moveit_error_code_to_string(final_result.error_code.val)
            self.get_logger().error(f"Trajectory execution failed: {error_string}")
            return False

# --------------------------------------------------------------------------------
def main(args=None):
    # This function is unchanged.
    rclpy.init(args=args)
    spiral_controller = SpiralController()
    executor = MultiThreadedExecutor()
    executor.add_node(spiral_controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        if spiral_controller.wait_for_ready():
            start_joint_goal = [math.radians(0), math.radians(-111), math.radians(124), 
                                math.radians(145), math.radians(19), math.radians(0)]
            
            spiral_controller.get_logger().info("--- Step 1: Moving to Start Position ---")
            success_to_start = spiral_controller.plan_and_execute_joint_goal(start_joint_goal)
            
            if success_to_start:
                spiral_controller.get_logger().info("\n--- Step 2: Executing Surface Motion ---")
                # Attempt the spiral motion. We no longer check if it was successful
                # because we want to return home regardless.
                spiral_controller.plan_and_execute_spiral()
                
                ### MODIFIED ###
                # This block is now unindented. It will execute after the spiral motion
                # is attempted, regardless of whether it succeeded or failed.
                spiral_controller.get_logger().info("\n--- Step 3: Returning to Start Position ---")
                spiral_controller.plan_and_execute_joint_goal(start_joint_goal)
            else:
                spiral_controller.get_logger().error("Failed to move to start position. Cancelling motion.")

    except KeyboardInterrupt:
        spiral_controller.get_logger().info("Keyboard interrupt, shutting down.")
    
    spiral_controller.destroy_node()
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()


