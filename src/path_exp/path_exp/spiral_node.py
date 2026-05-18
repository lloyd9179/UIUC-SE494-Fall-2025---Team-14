#### SE 494, Fall 2025
### Written by Junyang Guan
###  lloyd9179@gmail.com
##### University of Illinois Urbana Champaign
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints, JointConstraint


# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
def moveit_error_code_to_string(val):
    """Translates a MoveItErrorCodes message to a string."""
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
# --------------------------------------------------------------------------------
class MoveItController(Node):

    def __init__(self):
        super().__init__('moveit_controller_node')

        # --- 参数定义 ---
        self.move_group_name = "ur_manipulator"
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        # Action Server 的最终正确名称
        action_server_name = '/move_action'

        # --- ROS 2 通信设置 ---
        self._action_client = ActionClient(self, MoveGroup, action_server_name)
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # --- 内部状态变量 ---
        self.current_joint_state = None
        self.get_logger().info("节点已初始化。")

    def joint_state_callback(self, msg: JointState):
        """回调函数：持续接收并存储机器人的当前关节状态。"""
        if self.current_joint_state is None:
            self.get_logger().info("成功接收到第一个 /joint_states 消息。")
        self.current_joint_state = msg

    def wait_for_ready(self):
        """
        一个非常重要的函数：在执行任何动作前，确保所有必要的连接都已建立。
        """
        self.get_logger().info("正在等待所有服务准备就绪...")

        # 1. 等待 MoveGroup Action Server
        self.get_logger().info(" - 正在等待 MoveGroup Action Server...")
        while not self._action_client.wait_for_server(timeout_sec=2.0) and rclpy.ok():
            self.get_logger().warn("MoveGroup Action Server 尚未可用，将重试...")
        if not rclpy.ok(): return False
        self.get_logger().info("   ...MoveGroup Action Server 已连接！")

        # 2. 等待接收到有效的关节状态
        self.get_logger().info(" - 正在等待接收 /joint_states 消息...")
        while self.current_joint_state is None and rclpy.ok():
            self.get_logger().warn("/joint_states 消息尚未收到，将等待...")
            # 使用 spin_once 来处理回调，而不是阻塞
            rclpy.spin_once(self, timeout_sec=1.0)
        if not rclpy.ok(): return False
        self.get_logger().info("   .../joint_states 消息已收到！")
        
        self.get_logger().info("所有服务准备就绪！可以开始规划。")
        return True

    def plan_and_execute_joint_goal(self, goal_joint_positions):
        """规划并执行一个关节空间的目标。"""
        self.get_logger().info(f"开始规划前往目标关节位置: {goal_joint_positions}")

        if len(goal_joint_positions) != len(self.joint_names):
            self.get_logger().error("目标位置数量与关节数量不匹配！")
            return False

        # --- 1. 创建 Action Goal ---
        goal_msg = MoveGroup.Goal()
        
        # 设置运动请求
        request = goal_msg.request
        request.group_name = self.move_group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        
        # 设置起始状态为机器人当前状态
        request.start_state.joint_state = self.current_joint_state
        request.start_state.is_diff = True
        
        # --- [最终修正] ---
        # 这是构建目标约束的正确方法
        
        # 首先创建一个 Constraints 对象
        goal_constraints = Constraints()
        for i, name in enumerate(self.joint_names):
            # 然后为每个关节创建一个 JointConstraint 对象
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = goal_joint_positions[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            # 将每个 JointConstraint 'append' 到 Constraints 对象的列表中
            goal_constraints.joint_constraints.append(joint_constraint)

        # 最后将这一个 Constraints 对象 'append' 到 request 的列表中
        request.goal_constraints.append(goal_constraints)
        # --- [修正结束] ---

        # --- 2. 发送 Goal 并等待结果 ---
        self.get_logger().info("正在发送 Goal 到 MoveGroup Action Server...")
        future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal 被服务器拒绝！")
            return False
        
        self.get_logger().info("Goal 已被接受，等待执行结果...")
        result_future = goal_handle.get_result_async()
        
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        error_string = moveit_error_code_to_string(result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"运动成功！结果: {error_string}")
            return True
        else:
            self.get_logger().error(f"运动失败！结果: {error_string} (代码: {result.error_code.val})")
            return False

# --------------------------------------------------------------------------------
# 主函数：程序的入口
# --------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    moveit_controller = MoveItController()
    executor = MultiThreadedExecutor()
    executor.add_node(moveit_controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # --- 执行主要逻辑 ---
        # 1. 等待所有服务准备就绪
        if moveit_controller.wait_for_ready():
            # 2. 定义一个目标关节位置 (单位：弧度)
            joint_goal = [0.0, -2.0, 1.5, -1.0, -1.57, 0.0]
            # 3. 执行运动
            moveit_controller.plan_and_execute_joint_goal(joint_goal)
    except KeyboardInterrupt:
        moveit_controller.get_logger().info("Keyboard interrupt, shutting down.")
    
    # 清理
    moveit_controller.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()