import math
import rclpy
import tf_transformations
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import GoalResponse, CancelResponse

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

        self.ordom_timeout = 0.2  # Timeout for odometry updates in seconds

        self.current_odom_value = None

        self.odometry_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

        self._motor_action_server = ActionServer(
            self,
            WheelGoal,
            'control_motor',
            execute_callback=self.motor_action_callback)

    
    def motor_action_callback(self, goal_handle):
        self.get_logger().info('Received motor action request.')
        result = False
        function_id = goal_handle.request.function_id
        req_goal = goal_handle.request.goal
        response = WheelGoal.Result()

        if function_id == WheelGoal.Goal.FORWARD:
            result = self.__move_distance(req_goal, 1)
        elif function_id == WheelGoal.Goal.BACKWARD:
            result = self.__move_distance(req_goal, -1)
        elif function_id == WheelGoal.Goal.TURN_LEFT:
            result = self.__move_angle(req_goal, 1)
        elif function_id == WheelGoal.Goal.TURN_RIGHT:
            result = self.__move_angle(req_goal, -1)
        else:
            self.get_logger().warn(f'Unknown function ID: {function_id}')
            goal_handle.abort()
            response.success = False
            return response
        
        goal_handle.succeed()
        response.success = result
        return response
    
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
        self.__publish_twist(linear_x=0.0, angular_z=0.0)
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
       
    
    def __move_distance(self, distance: float, direction: int) -> bool:
        self.get_logger().info(f'Moving distance by {distance} meters.')

        linear_x = 0.3 * direction
        distance = distance * direction

        self.__publish_twist(linear_x=linear_x, angular_z=0.0)

        if self.current_odom_value is None:
            self.get_logger().warn('Current odometry value is not available. Cannot determine starting position.')
            return False
        
        target_x = self.current_odom_value.pose.pose.position.x + distance

        while abs(target_x - self.current_odom_value.pose.pose.position.x) > self.tolerance_xy:
            odom_time_stamp = rclpy.time.Time.from_msg(self.current_odom_value.header.stamp)
            if (self.get_clock().now() - odom_time_stamp).nanoseconds / 1e9 > self.ordom_timeout:
                print((self.get_clock().now() - odom_time_stamp).nanoseconds / 1e9)
                self.get_logger().warn('Odometry timeout reached. Stopping movement.')
                self.stop_motor()
                return False
        
        self.get_logger().info('Movement distance completed.')
        self.stop_motor()
        return True
    
    def __move_angle(self, angle:float, direction:int) -> bool:
        if self.current_odom_value is None:
            self.get_logger().error("Current odometry value is not available. Cannot determine starting position.")
            return False
        self.get_logger().info(f"Turning robot with angle: {angle}")
        target_yaw = math.radians(abs(angle))
        angular_z = 0.3 * direction
        self.__publish_twist(linear_x=0.0, angular_z=angular_z)
        start_yaw = self.__get_yaw(self.current_odom_value.pose.pose)

        while True:
            current_pose = self.current_odom_value.pose.pose
            current_yaw = self.__get_yaw(current_pose)

            yaw_diff = (current_yaw - start_yaw)
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            
            if abs(abs(yaw_diff) - abs(target_yaw)) < 0.1:
                self.get_logger().info("Goal reached, Stopping Movement")
                self.stop_motor()
                return True
    
    def __get_yaw(self, pose):
        q = pose.orientation
        return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    


def main(args=None):
    rclpy.init(args=args)
    node = OrbitControllerApi()
    
    try:

        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info('Node has been stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()