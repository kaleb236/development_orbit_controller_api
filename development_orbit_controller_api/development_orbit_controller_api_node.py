import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from orbit_command_msgs.action import WheelGoal

class OrbitControllerApi(Node):
    def __init__(self):
        super().__init__('orbit_controller_api_node')
        self.get_logger().info('Orbit Controller API Node has been started.')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.tolerance_xy = 0.1  # Tolerance for position in meters

        self.current_odom_value = None

        self.odometry_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

        self._action_server = ActionServer(
            self,
            WheelGoal,
            'move_forward',
            self.move_forward)
    
    def move_forward(self, goal_handle):
        result = self.__move_distance(goal_handle.request.goal, 1)
        print(f'Move forward result: {result}')
        response = WheelGoal.Result()
        response.success = result
        return response
    
    def move_backward(self, distance: float):
        self.__move_distance(distance, -1)
        self.get_logger().info(f'Moving backward by {distance} meters.')
        # Implement the logic to move backward
    
    def turn_left(self, angle: int):
        self.get_logger().info(f'Turning left by {angle} degrees.')
        # Implement the logic to turn left
    
    def turn_right(self, angle: int):
        self.get_logger().info(f'Turning right by {angle} degrees.')
        # Implement the logic to turn right
    
    def set_rpm(self, rpm: list):
        self.get_logger().info(f'Setting RPM to {rpm}.')
        # Implement the logic to set RPM
    
    def set_speed(self, speed: float):
        self.get_logger().info(f'Setting speed to {speed} m/s.')
        # Implement the logic to set speed
    
    def set_postion(self, pose: list):
        self.get_logger().info(f'Setting position to {pose}.')
        # Implement the logic to set position
    
    def stop_motor(self):
        self.get_logger().info('Stopping motor.')
        # Implement the logic to stop the motor

    def odometry_callback(self, msg: Odometry):
        self.current_odom_value = msg
        # self.get_logger().info(f"Received odometry: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}")
    
    def __publish_twist(self, linear_x=0.0, angular_z=0.0):
        self.get_logger().info(f'Publishing Twist message: linear.x={linear_x}, angular.z={angular_z}')
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
    
    def __check_goal_reached(self, current_distance: list, target_distance: list) -> bool:
        self.get_logger().info(f'Checking if goal is reached: current={current_distance}, target={target_distance}')
        dx = target_distance[0] - current_distance[0]
        dy = target_distance[1] - current_distance[1]
        
        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance < self.tolerance_xy:
            self.get_logger().info('Goal reached.')
            return True
        else:
            return False
        
    
    def __move_distance(self, distance: float, direction: int) -> bool:
        self.get_logger().info(f'Moving distance by {distance} meters.')

        linear_x = 0.3 * direction
        distance = distance * direction

        self.__publish_twist(linear_x=linear_x, angular_z=0.0)

        if self.current_odom_value is None:
            self.get_logger().warn('Current odometry value is not available. Cannot determine starting position.')
            return False
        starting_position = self.current_odom_value.pose.pose.position

        while rclpy.ok() and not self.__check_goal_reached(
            [self.current_odom_value.pose.pose.position.x, self.current_odom_value.pose.pose.position.y],
            [starting_position.x + distance, starting_position.y]
        ):
            rclpy.spin_once(self)
        
        self.get_logger().info('Movement distance completed.')
        return True
    


def main(args=None):
    rclpy.init(args=args)
    node = OrbitControllerApi()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node has been stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()