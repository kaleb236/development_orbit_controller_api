import rclpy
import requests
import vlc
from rclpy.node import Node

from arduino_msgs.srv import ArduinoWrite, SensorValue
from orbit_command_msgs.srv import Face, SetString

class Orbit(Node):
    def __init__(self):
        super().__init__("OrbitLiveController")

        self.get_logger().info("OrbitLiveController Initialized")

        self.arduino_write_client = self.create_client(ArduinoWrite, 'arduino_write_value')
        self.arduino_sensor_client = self.create_client(SensorValue, 'arduino_sensor_value')
        self.face_client = self.create_client(Face, 'change_face')
        self.dance_client = self.create_client(SetString, 'dance')
        

    def move_forward(self, distance:int):
        self.get_logger().info("Moving forward")
    
    def move_backward(self, distance:int):
        self.get_logger().info("Moving backward")
    
    def turn_left(self, angle:int):
        self.get_logger().info("Turning left")
    
    def turn_right(self, angle:int):
        self.get_logger().info("Turning right")
    
    def set_rpm(self, rpm: list):
        self.get_logger().info(f"Moving Robot with {rpm}")
    
    def set_speed(self, speed: list):
        self.get_logger().info(f"Moving Robot with {speed}")
    
    def stop_motor(self):
        self.get_logger().info("Stoping robot")
    
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
    
    def set_rgb(self, rgb: list[int]):
        self.get_logger().info(f"setting rgb {rgb}")
        self.__call_arduino_srv(1, [6, rgb[0], rgb[1], rgb[2]])
    
    def set_led_animation(self, animation_type:int, rgb_color: list):
        self.get_logger().info(f"setting led animation with animation_type: {animation_type} rgb: {rgb_color}")
        self.__call_arduino_srv(1, [animation_type, rgb_color[0], rgb_color[1], rgb_color[2]])
    
    def set_head_pose(self, x:int, y:int):
        self.get_logger().info(f"setting head pose to x:{x}, y: {y}")
        self.__call_arduino_srv(2, [x, y, 0, 0])
    
    def change_face(self, face_id: int):
        self.get_logger().info(f"changing face to {face_id}")
        if not self.face_client.wait_for_service():
            self.get_logger().error("Face Client not active")
        face_msg = Face.Request()
        face_msg.face = face_id
        future = self.face_client.call_async(face_msg)
        rclpy.spin_until_future_complete(self, future)
    
    def text_to_speech(self, text: str):
        self.get_logger().info(f"Speaking {text}")
    
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
    
    def __call_arduino_srv(self, cmd_type:int, args:list):
        req = ArduinoWrite.Request()
        if not self.arduino_write_client.wait_for_service():
            self.get_logger().error("LED Serive not ready")
        req.cmd_type = cmd_type
        req.arguments = args
        future = self.arduino_write_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("service called")
    
    def __read_sensor_value(self, sensor_id:int) -> float:
        if not self.arduino_sensor_client.wait_for_service():
            self.get_logger().error("Sensor Read service not ready")
            return
        req = SensorValue.Request()
        req.req = sensor_id
        future = self.arduino_sensor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().data
    
    def __search_deezer_and_play_preview(self, query):
        # 1. Search Deezer for the track
        response = requests.get(f"https://api.deezer.com/search?q={query}")
        data = response.json()

        if 'data' not in data or len(data['data']) == 0:
            print("No results found.")
            return

        # 2. Get the first result
        track = data['data'][0]
        title = track['title']
        artist = track['artist']['name']
        preview_url = track['preview']  # 30-second MP3
        print(f"\nPlaying preview: {title} - {artist}")
        print(f"Preview URL: {preview_url}")
        return preview_url