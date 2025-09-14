import rclpy

from rclpy.node import Node

from orbit_command_msgs.msg import BlocklyCode

suffix ="""
from development_orbit_controller_api.orbit_api.orbit import Orbit

orbit = Orbit()
"""

preffix = """
orbit.stop_motor()
orbit.destroy_node()
"""

TEST_CODE = """
import time
print("starting execution")
time.sleep(2)
"""

class BlocklyServer(Node):
    def __init__(self):
        super().__init__("blockly_server")

        self.blockly_sub = self.create_subscription(BlocklyCode, "orbit_blockly", self.blockly_callback, 10)

    def blockly_callback(self, blockly_msg:BlocklyCode):
        student_id = blockly_msg.student_id
        student_name = blockly_msg.student_name
        python_code = blockly_msg.python_code
        executable_code = suffix + python_code + preffix
        exec(executable_code)
        self.get_logger().info("executing finished")

def main(args=None):
    rclpy.init(args=args)
    node = BlocklyServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has been stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()