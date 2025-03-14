import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MockCameraPublisher(Node):

    def __init__(self, depth, latency, exposure):
        super().__init__('Input_Validation_Publisher')
        self.publisher_ = self.create_publisher(String, 'camera_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        
        # Setează valorile personalizate pentru depth, latency și exposure
        self.depth = depth
        self.latency = latency
        self.exposure = exposure

        # Liste predefinite pentru valori valide și invalide
        self.valid_depth_range = (0.6, 6.0)
        self.invalid_depth_range = [(-float('inf'), 0.6), (6.0, float('inf'))]

        self.valid_latency_max = 50  # ms
        self.invalid_latency_min = 100  # ms

        self.valid_exposure_range = (100, 200)  # Color exposure range
        self.valid_exposure_depth_range = (8000, 9000)  # Depth exposure range
        self.invalid_exposure_range = [(-float('inf'), 50), (10000, float('inf'))]

    def publish_data(self):
        msg = String()

        # Verifică dacă datele introduse sunt valide
        if self.is_valid():
            msg.data = f"Valid Data: Depth: {self.depth}, Latency: {self.latency}ms, Exposure: {self.exposure}"
        else:
            msg.data = f"Invalid Data: Depth: {self.depth}, Latency: {self.latency}ms, Exposure: {self.exposure}"

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def is_valid(self):
        # Verifică pentru depth, latency și exposure dacă sunt valide
        is_valid_depth = self.valid_depth_range[0] <= self.depth <= self.valid_depth_range[1]
        is_valid_latency = self.latency <= self.valid_latency_max
        is_valid_exposure = (self.valid_exposure_range[0] <= self.exposure <= self.valid_exposure_range[1] or
                             self.valid_exposure_depth_range[0] <= self.exposure <= self.valid_exposure_depth_range[1])

        return is_valid_depth and is_valid_latency and is_valid_exposure

def main(args=None):
    rclpy.init(args=args)
    
    # Introducere date din consolă
    custom_depth = float(input("Introduceți valoarea pentru depth (m): "))
    custom_latency = float(input("Introduceți valoarea pentru latency (ms): "))
    custom_exposure = int(input("Introduceți valoarea pentru exposure: "))

    mock_camera_publisher = MockCameraPublisher(custom_depth, custom_latency, custom_exposure)

    # Rulează doar pentru 5 mesaje
    for _ in range(5):
        rclpy.spin_once(mock_camera_publisher)  # Rulează o singură iterație
        time.sleep(1)  # Pauză între mesaje 

    mock_camera_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
