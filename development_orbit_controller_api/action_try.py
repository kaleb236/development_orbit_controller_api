import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from action_tutorials_interfaces.action import Fibonacci
import asyncio


class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')

        # Action server
        self._server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        # Subscription to odometry
        self._last_odom = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        # just store latest odometry message
        self._last_odom = msg

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback = Fibonacci.Feedback()
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Goal canceled!")
                goal_handle.canceled()
                return Fibonacci.Result()

            # Use the latest odometry info
            if self._last_odom is not None:
                pos = self._last_odom.pose.pose.position
                self.get_logger().info(f"Odom position: x={pos.x:.2f}, y={pos.y:.2f}")

            # Fibonacci sequence just for demo
            sequence.append(sequence[i] + sequence[i - 1])
            feedback.partial_sequence = sequence
            goal_handle.publish_feedback(feedback)

            await asyncio.sleep(1)  # let executor process subscriptions

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result


def main():
    rclpy.init()
    node = MyActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
