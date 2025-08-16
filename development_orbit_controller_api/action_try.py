import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from nav_msgs.msg import Odometry

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

        
        self.odometry_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result
    
    def odometry_callback(self, msg: Odometry):
        self.get_logger().info(f"Received odometry: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}")


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()