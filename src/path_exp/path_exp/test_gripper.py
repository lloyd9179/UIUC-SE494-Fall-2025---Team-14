#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO
import time

# --- 最终配置 (URCap 模式) ---
# fun = 1  -> "Set Digital Output"
# URCap 正在监听这个引脚
SET_IO_FUN = 1

# pin = 16 -> 映射到 "Tool Digital Output 0"
# (如果这个不行，请尝试 17)
GRIPPER_PIN = 17

# 0.0 = Low (URCap 解释为 Open)
# 1.0 = High (URCap 解释为 Close)
STATE_OPEN = 0.0
STATE_CLOSE = 1.0

WAIT_TIME_SECONDS = 3.0

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester_node')
        self.get_logger().info("Gripper Tester ready (URCap + SetIO Mode)")
        
        self.set_io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        
    def wait_for_service(self):
        self.get_logger().info("wait for /io_and_status_controller/set_io")
        while not self.set_io_client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("retrying...")
        self.get_logger().info("I/O ready。")

    def set_gripper_state(self, state, state_name):

        self.get_logger().info(f"sending command: {state_name} (Fun: {SET_IO_FUN}, Pin: {GRIPPER_PIN}, State: {state})...")
        
        request = SetIO.Request()
        request.fun = SET_IO_FUN      # 使用 fun = 1
        request.pin = GRIPPER_PIN   # 使用 pin = 16
        request.state = state

        future = self.set_io_client.call_async(request)
        
        try:
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

            response = future.result()
            
            if response and response.success:
                self.get_logger().info(f"command '{state_name}' sccussed")
                return True
            else:
                self.get_logger().error(f"command '{state_name}' failed")
                return False
        except Exception as e:
            self.get_logger().error(f"exception occurred: {e}")
            return False

    def run_test_sequence(self):
        try:
            # URCap 已经激活了电源，我们直接发送 I/O 命令
            self.get_logger().info("Gripper should be active (Blue Light ON). Sending I/O commands...")

            if not self.set_gripper_state(STATE_OPEN, "open"):
                return  
            self.get_logger().info(f"wait for {WAIT_TIME_SECONDS} seconds...")
            time.sleep(WAIT_TIME_SECONDS)

            if not self.set_gripper_state(STATE_CLOSE, "close"):
                return
            self.get_logger().info(f"wait for {WAIT_TIME_SECONDS} seconds...")
            time.sleep(WAIT_TIME_SECONDS)

            if not self.set_gripper_state(STATE_OPEN, "open"):
                return
            
        except KeyboardInterrupt:
            self.get_logger().info("keyboard interrupted")
        finally:
            self.get_logger().info("Test finished. Setting gripper to 'closed'.")
            self.set_gripper_state(STATE_CLOSE, "closed")

def main(args=None):
    rclpy.init(args=args)
    
    gripper_tester = GripperTester()
    
    # 确保驱动已重启并且示教器在 "Controlled by: Robotiq Grippers" 模式
    gripper_tester.wait_for_service()
    
    if rclpy.ok():
        gripper_tester.run_test_sequence()
        
    gripper_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()