import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockCameraSubscriber(Node):

    def __init__(self):
        super().__init__('Input_Validation_Subscriber')
        self.subscription = self.create_subscription(
            String,
            'camera_data',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        data = msg.data
        self.get_logger().info(f'Received message: "{data}"')

        # Verifică dacă mesajul conține "Valid" sau "Invalid"
        if "Valid" in data:
            self.get_logger().info("Test Passed")
        else:
            self.get_logger().info("Test Failed")

def main(args=None):
    rclpy.init(args=args)
    mock_camera_subscriber = MockCameraSubscriber()
    rclpy.spin(mock_camera_subscriber)
    mock_camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

