import rclpy
import requests
import vlc
from rclpy.node import Node
from rclpy.action import ActionClient

from arduino_msgs.srv import ArduinoWrite, SensorValue
from orbit_command_msgs.srv import Face, SetString, Records
from orbit_command_msgs.action import WheelGoal


class Orbit(Node):
    def __init__(self):
        super().__init__("OrbitLiveController")

        self.get_logger().info("OrbitLiveController Initialized")

        # Service clients
        self.arduino_write_client = self.create_client(ArduinoWrite, 'arduino_write_value')
        self.arduino_sensor_client = self.create_client(SensorValue, 'arduino_sensor_value')
        self.face_client = self.create_client(Face, 'change_face')
        self.dance_client = self.create_client(SetString, 'dance')
        self.text_task_client = self.create_client(Records, 'record_task')

        # Action client for motor control
        self.motor_action_client = ActionClient(self, WheelGoal, 'control_motor')

    # -------------------- Motor Control -------------------- #
    def move_forward(self, distance: int):
        self.get_logger().info(f"Moving forward {distance} m")
        self.__send_motor_goal(WheelGoal.Goal.FORWARD, distance)

    def move_backward(self, distance: int):
        self.get_logger().info(f"Moving backward {distance} m")
        self.__send_motor_goal(WheelGoal.Goal.BACKWARD, distance)

    def __send_motor_goal(self, function_id: int, distance: float):
        """Private helper to send motor action goals"""
        if not self.motor_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Motor action server not available!")
            return

        goal_msg = WheelGoal.Goal()
        goal_msg.function_id = function_id
        goal_msg.goal = float(distance)

        self.get_logger().info(
            f"Sending motor goal -> function_id={function_id}, distance={distance}"
        )

        send_goal_future = self.motor_action_client.send_goal_async(
            goal_msg,
            # feedback_callback=self.__motor_feedback_callback
        )
        # send_goal_future.add_done_callback(self.__motor_goal_response_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        self.get_logger().info(f"Goal handle: {goal_handle}")

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected")
            return None

        self.get_logger().info("Goal accepted, waiting for result...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f"Action finished, success={result.success}")

    def __motor_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Motor goal was rejected.")
            return

        self.get_logger().info("Motor goal accepted by server.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.__motor_result_callback)

    def __motor_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Motor feedback: {feedback}")

    def __motor_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Motor action result: success={result.success}")

    # -------------------- Turning -------------------- #
    def turn_left(self, angle: int):
        self.get_logger().info(f"Turning left {angle} degrees")
        self.__send_motor_goal(WheelGoal.Goal.TURN_LEFT, angle)

    def turn_right(self, angle: int):
        self.get_logger().info(f"Turning right {angle} degrees")
        self.__send_motor_goal(WheelGoal.Goal.TURN_RIGHT, angle)

    # -------------------- Speed & RPM -------------------- #
    def set_rpm(self, rpm: list):
        self.get_logger().info(f"Moving Robot with {rpm}")

    def set_speed(self, speed: list):
        self.get_logger().info(f"Moving Robot with {speed}")

    def stop_motor(self):
        self.get_logger().info("Stopping robot")

    # -------------------- Sensor Reads -------------------- #
    def distance_value(self) -> float:
        self.get_logger().info("Getting distance value")
        return self.__read_sensor_value(SensorValue.Request.DISTANCE)

    def ldr_value(self) -> float:
        self.get_logger().info("Getting ldr value")
        return self.__read_sensor_value(SensorValue.Request.LDR)

    def rfid_value(self) -> str:
        self.get_logger().info("Getting rfid_value")
        return "udba-9898dn"

    def line_follower(self) -> list:
        self.get_logger().info("Getting line follower values")
        return []

    def voltage(self) -> float:
        self.get_logger().info("Getting voltage value")
        return self.__read_sensor_value(SensorValue.Request.VOLTAGE)

    def current(self) -> float:
        self.get_logger().info("Getting current value")
        return self.__read_sensor_value(SensorValue.Request.CURRENT)

    def speed(self) -> list:
        self.get_logger().info("Getting speed values")
        return []

    # -------------------- LED & Face -------------------- #
    def set_rgb(self, rgb: list[int]):
        self.get_logger().info(f"setting rgb {rgb}")
        self.__call_arduino_srv(1, [6, rgb[0], rgb[1], rgb[2]])

    def set_led_animation(self, animation_type: int, rgb_color: list):
        self.get_logger().info(
            f"setting led animation with animation_type: {animation_type} rgb: {rgb_color}"
        )
        self.__call_arduino_srv(1, [animation_type, rgb_color[0], rgb_color[1], rgb_color[2]])

    def set_head_pose(self, x: int, y: int):
        self.get_logger().info(f"setting head pose to x:{x}, y:{y}")
        self.__call_arduino_srv(2, [x, y, 0, 0])

    def change_face(self, face_id: int):
        self.get_logger().info(f"changing face to {face_id}")
        if not self.face_client.wait_for_service():
            self.get_logger().error("Face Client not active")
        face_msg = Face.Request()
        face_msg.face = face_id
        future = self.face_client.call_async(face_msg)
        rclpy.spin_until_future_complete(self, future)

    # -------------------- Media & Dance -------------------- #
    def text_to_speech(self, text: str):
        self.get_logger().info(f"Speaking {text}")
        if not self.text_task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('text_task_client not available')
        
        request = Records.Request()
        request.records = text
        future = self.text_task_client.call_async(request)
        self.get_logger().info("TEXT_SERVICE_CALLED")
        rclpy.spin_until_future_complete(self, future)

    def play_song(self, song_name: str):
        self.get_logger().info(f"Playing {song_name} song")
        preview_url = self.__search_deezer_and_play_preview(song_name)
        if preview_url is None:
            return "Song not found"

        player = vlc.MediaPlayer(preview_url)
        player.play()
        while True:
            state = player.get_state()
            if state in [vlc.State.Ended, vlc.State.Error]:
                break

    def dance(self, song_name: str):
        self.get_logger().info(f"Dancing {song_name}")
        if not self.dance_client.wait_for_service():
            self.get_logger().error("Dance Service not ready")
        req = SetString.Request()
        req.req = song_name
        future = self.dance_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    # -------------------- Helpers -------------------- #
    def __call_arduino_srv(self, cmd_type: int, args: list):
        req = ArduinoWrite.Request()
        if not self.arduino_write_client.wait_for_service():
            self.get_logger().error("LED Service not ready")
        req.cmd_type = cmd_type
        req.arguments = args
        future = self.arduino_write_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("service called")

    def __read_sensor_value(self, sensor_id: int) -> float:
        if not self.arduino_sensor_client.wait_for_service():
            self.get_logger().error("Sensor Read service not ready")
            return
        req = SensorValue.Request()
        req.req = sensor_id
        future = self.arduino_sensor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().data

    def __search_deezer_and_play_preview(self, query):
        response = requests.get(f"https://api.deezer.com/search?q={query}")
        data = response.json()

        if 'data' not in data or len(data['data']) == 0:
            print("No results found.")
            return

        track = data['data'][0]
        title = track['title']
        artist = track['artist']['name']
        preview_url = track['preview']
        print(f"\nPlaying preview: {title} - {artist}")
        print(f"Preview URL: {preview_url}")
        return preview_url
