import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import os

class CameraStreamSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_for_errors')
        
        # Obține calea directorului în care se află scriptul
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Construiește calea către fișierul YAML
        config_path = os.path.join(current_dir, 'camera_config.yaml')
        
        # Încărcăm configurațiile din fișierul YAML
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Error loading config file: {e}")
            raise  # Ridică excepția pentru a opri inițializarea nodului

        # Parametrii din fișierul de configurare
        self.topic = self.config['camera']['topic']
        self.expected_rgb_resolution = self.config['camera']['rgb']['resolution']
        self.expected_rgb_fps_min = self.config['camera']['rgb']['fps']['min']
        self.expected_rgb_fps_max = self.config['camera']['rgb']['fps']['max']
        self.expected_infrared_enabled = self.config['camera']['infrared']['enabled']
        self.expected_infrared_fps_min = self.config['camera']['infrared']['fps']['min']
        self.expected_infrared_fps_max = self.config['camera']['infrared']['fps']['max']
        self.expected_depth_min_distance = self.config['camera']['depth']['min_depth_distance']
        self.expected_depth_ideal_range_min = self.config['camera']['depth']['ideal_range']['min']
        self.expected_depth_ideal_range_max = self.config['camera']['depth']['ideal_range']['max']
        self.expected_depth_accuracy_min = self.config['camera']['depth']['accuracy']['min']
        self.expected_depth_accuracy_max = self.config['camera']['depth']['accuracy']['max']
        self.expected_depth_resolution = self.config['camera']['depth']['resolution']
        self.expected_depth_fps_min = self.config['camera']['depth']['fps']['min']
        self.expected_depth_fps_max = self.config['camera']['depth']['fps']['max']
        
        # Variabile pentru a salva mesajul primit
        self.received_msg = None
        
        # Creează subscription-ul pentru topic-ul specificat
        self.subscription = self.create_subscription(
            String, self.topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        self.received_msg = msg.data
        self.get_logger().info(f"Received: {self.received_msg}")

    def validate_stream(self):
        # Verifică dacă mesajul primit conține datele așteptate
        assert f"RGB: {self.expected_rgb_resolution} at {self.expected_rgb_fps_min}-{self.expected_rgb_fps_max} FPS" in self.received_msg, "RGB stream is incorrect"
        assert f"Infrared: {'Enabled' if self.expected_infrared_enabled else 'Disabled'} at {self.expected_infrared_fps_min}-{self.expected_infrared_fps_max} FPS" in self.received_msg, "Infrared stream is incorrect"
        assert f"Depth: {self.expected_depth_resolution}, {self.expected_depth_min_distance}m to {self.expected_depth_ideal_range_max}m, Accuracy: {self.expected_depth_accuracy_min}%-{self.expected_depth_accuracy_max}% at {self.expected_depth_fps_min}-{self.expected_depth_fps_max} FPS" in self.received_msg, "Depth stream is incorrect"

def main(args=None):
    rclpy.init(args=args)
    subscriber = CameraStreamSubscriber()
    rclpy.spin_once(subscriber, timeout_sec=1.0)
    subscriber.validate_stream()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
