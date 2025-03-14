import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import os

class CameraStreamPublisher_for_errors(Node):
    def __init__(self, config_path=None):
        super().__init__('publisher_for_errors')
        
        # Dacă nu este furnizată o cale, folosim calea implicită
        if config_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, 'camera_config.yaml')
        
        # Verifică dacă fișierul de configurare există
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found: {config_path}")
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        # Încărcăm configurațiile din fișierul YAML
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")
            raise yaml.YAMLError(f"Error parsing YAML file: {e}") from e
        except Exception as e:
            self.get_logger().error(f"Unexpected error loading config file: {e}")
            raise Exception(f"Unexpected error loading config file: {e}") from e

        # Verifică dacă toți parametrii necesari sunt prezenți în fișierul de configurare
        required_keys = ['camera', 'topic', 'rgb', 'infrared', 'depth']
        if not all(key in self.config for key in required_keys):
            self.get_logger().error("Missing required configuration keys")
            raise KeyError("Missing required configuration keys")

        # Extrage parametrii din configurație
        try:
            self.topic = self.config['camera']['topic']
            self.rgb_resolution = self.config['camera']['rgb']['resolution']
            self.rgb_fps_min = self.config['camera']['rgb']['fps']['min']
            self.rgb_fps_max = self.config['camera']['rgb']['fps']['max']
            self.infrared_enabled = self.config['camera']['infrared']['enabled']
            self.infrared_fps_min = self.config['camera']['infrared']['fps']['min']
            self.infrared_fps_max = self.config['camera']['infrared']['fps']['max']
            self.depth_min_distance = self.config['camera']['depth']['min_depth_distance']
            self.depth_ideal_range_min = self.config['camera']['depth']['ideal_range']['min']
            self.depth_ideal_range_max = self.config['camera']['depth']['ideal_range']['max']
            self.depth_accuracy_min = self.config['camera']['depth']['accuracy']['min']
            self.depth_accuracy_max = self.config['camera']['depth']['accuracy']['max']
            self.depth_resolution = self.config['camera']['depth']['resolution']
            self.depth_fps_min = self.config['camera']['depth']['fps']['min']
            self.depth_fps_max = self.config['camera']['depth']['fps']['max']
        except KeyError as e:
            self.get_logger().error(f"Missing configuration parameter: {e}")
            raise KeyError(f"Missing configuration parameter: {e}") from e
        except Exception as e:
            self.get_logger().error(f"Unexpected error extracting configuration: {e}")
            raise Exception(f"Unexpected error extracting configuration: {e}") from e

        # Verifică validitatea FPS
        if self.rgb_fps_min > self.rgb_fps_max:
            self.get_logger().error("Invalid RGB FPS range: min > max")
            raise ValueError("Invalid RGB FPS range: min > max")
        if self.infrared_fps_min > self.infrared_fps_max:
            self.get_logger().error("Invalid Infrared FPS range: min > max")
            raise ValueError("Invalid Infrared FPS range: min > max")

        # Creează publisher-ul pentru topic-ul specificat
        self.publisher = self.create_publisher(String, self.topic, 10)

    def publish_message(self):
        # Publică informații despre camera RGB, infrared și depth
        msg = String()
        msg.data = (
            f"Camera Stream - RGB: {self.rgb_resolution} at {self.rgb_fps_min}-{self.rgb_fps_max} FPS | "
            f"Infrared: {'Enabled' if self.infrared_enabled else 'Disabled'} at {self.infrared_fps_min}-{self.infrared_fps_max} FPS | "
            f"Depth: {self.depth_resolution}, {self.depth_min_distance}m to {self.depth_ideal_range_max}m, "
            f"Accuracy: {self.depth_accuracy_min}%-{self.depth_accuracy_max}% at {self.depth_fps_min}-{self.depth_fps_max} FPS"
        )
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
