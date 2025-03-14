import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

class CameraStreamSubscriber(Node):
    def __init__(self):
        super().__init__('camera_stream_subscriber')
        
        # Încărcăm configurațiile din fișierul YAML
        with open('camera_config.yaml', 'r') as f:
            self.config = yaml.safe_load(f)

        # Parametrii din fișierul de configurare
        self.topic = self.config['camera']['topic']
        self.expected_fps_rgb = self.config['camera']['fps_rgb']
        self.expected_fps_depth = self.config['camera']['fps_depth']
        self.expected_resolution_rgb = self.config['camera']['resolution_rgb']
        self.expected_resolution_depth = self.config['camera']['resolution_depth']
        
        # Variabile pentru a salva mesajul primit
        self.received_msg = None
        
        # Creează subscription-ul pentru topic-ul specificat
        self.subscription = self.create_subscription(
            String, self.topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        self.received_msg = msg.data
        self.get_logger().info(f"Received: {self.received_msg}")

    def validate_stream(self):
        # Verifică dacă FPS-ul și rezoluția din mesajul primit sunt conforme cu configurația
        assert f"RGB: {self.expected_resolution_rgb[0]}x{self.expected_resolution_rgb[1]} at {self.expected_fps_rgb} FPS" in self.received_msg, f"Expected RGB stream settings not found in {self.received_msg}"
        assert f"Depth: {self.expected_resolution_depth[0]}x{self.expected_resolution_depth[1]} at {self.expected_fps_depth} FPS" in self.received_msg, f"Expected Depth stream settings not found in {self.received_msg}"

def main(args=None):
    rclpy.init(args=args)
    subscriber = CameraStreamSubscriber()
    rclpy.spin_once(subscriber, timeout_sec=1.0)
    subscriber.validate_stream()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

