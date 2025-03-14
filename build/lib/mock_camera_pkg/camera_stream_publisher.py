import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

class CameraStreamPublisher(Node):
    def __init__(self):
        super().__init__('camera_stream_publisher')
        
        # Încărcăm configurațiile din fișierul YAML
        try:
            with open('camera_config.yaml', 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Error loading config file: {e}")
            return

        # Verificăm că toate valorile necesare sunt prezente în fișierul de configurare
        try:
            self.topic = self.config['camera']['topic']
            self.fps_rgb = self.config['camera']['fps_rgb']
            self.fps_depth = self.config['camera']['fps_depth']
            self.resolution_rgb = self.config['camera']['resolution_rgb']
            self.resolution_depth = self.config['camera']['resolution_depth']
            self.sensor_technology = self.config['camera']['sensor_technology']
        except KeyError as e:
            self.get_logger().error(f"Missing configuration value: {e}")
            return

        # Creează publisher-ul pentru topic-ul specificat
        self.publisher = self.create_publisher(String, self.topic, 10)

    def publish_message(self):
        # Publică informații despre camera RGB și depth
        msg = String()
        msg.data = f"Camera Stream - RGB: {self.resolution_rgb[0]}x{self.resolution_rgb[1]} at {self.fps_rgb} FPS, Depth: {self.resolution_depth[0]}x{self.resolution_depth[1]} at {self.fps_depth} FPS"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    publisher = CameraStreamPublisher()
    
    if publisher:
        # Publică un singur mesaj
        publisher.publish_message()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

