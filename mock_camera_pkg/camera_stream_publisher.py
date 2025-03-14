import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import os
import random
import time
import psutil

class CameraStreamPublisher(Node):
    def __init__(self):
        super().__init__('camera_stream_publisher')
        
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
            return

        # Extrage parametrii din configurație
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

        # Creează publisher-ul pentru topic-ul specificat
        self.publisher = self.create_publisher(String, self.topic, 10)

        # Initializează statusul stabilității, latenței etc.
        self.stability_status = 'stable'
        self.error_recovery_status = 'recovered'
        self.stress_status = 'passed'

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

    # Funcție de validare a FPS pentru RGB
    def validate_rgb_fps(self):
        return self.rgb_fps_min >= 15 and self.rgb_fps_max <= 60

    # Funcție de validare a FPS pentru Infrared
    def validate_infrared_fps(self):
        return self.infrared_fps_min >= 30 and self.infrared_fps_max <= 120

    # Test de latență
    def run_latency_test(self):
        start_time = time.time()
        # Simulăm publicarea unui mesaj pentru a măsura latența
        self.publish_message()
        latency = time.time() - start_time
        return latency

    # Test de stabilitate (dacă FPS este mare, testul poate da eșec)
    def run_stability_test(self):
        if self.rgb_fps_min > 50 and self.rgb_resolution == '1920x1080':
            self.stability_status = 'unstable'
        return self.stability_status

    # Test de utilizare a resurselor
    def run_resource_utilization_test(self):
        # Măsoară utilizarea CPU
        cpu_usage = psutil.cpu_percent(interval=1)
        return cpu_usage

    # Test de recuperare în caz de eroare
    def run_error_recovery_test(self, error_scenario):
        if error_scenario == 'data_corruption':
            self.error_recovery_status = 'failed'
        return self.error_recovery_status

    # Test de stres
    def run_stress_test(self):
        # Dacă FPS-ul este mare, stresul poate duce la eșec
        if self.rgb_fps_min > 100:
            self.stress_status = 'failed'
        else:
            self.stress_status = 'passed'
        return self.stress_status

    # Test de throughput
    def run_throughput_test(self, throughput):
        return throughput > 300  # Verificăm dacă throughput-ul este mai mare decât 300 Mbps


def main(args=None):
    rclpy.init(args=args)
    publisher = CameraStreamPublisher()
    
    if publisher:
        # Publică un singur mesaj
        publisher.publish_message()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
