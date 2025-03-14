import pytest
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from mock_camera_pkg.camera_stream_publisher import CameraStreamPublisher
from mock_camera_pkg.camera_stream_subscriber import CameraStreamSubscriber
from mock_camera_pkg.publisher_for_errors import CameraStreamPublisher_for_errors
from mock_camera_pkg.subscriber_for_errors import CameraStreamSubscriber

# Importă toate testele tale
from test.unit_tests.test_functional import (
    test_positive_publisher_initialization,
    test_positive_stream_activation,
    test_negative_infrared_disabled,
    test_positive_message_content,
)
from test.unit_tests.test_error_handling import (
    test_positive_publisher_initialization,
    test_rgb_fps_invalid_range,
    test_infrared_fps_invalid_range,
    test_negative_missing_config_parameter,
)
from test.unit_tests.test_performance import (
    test_rgb_fps_valid_range,
    test_infrared_fps_valid_range,
    test_latency,
    test_stability,
    test_resource_utilization,
    test_error_recovery,
    test_stress,
    test_throughput,
)

# Fixture pentru inițializarea și închiderea ROS 2
@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()

# Funcție pentru generarea descrierii testului
def generate_test_description():
    publisher_node = Node(
        package='mock_camera_pkg',
        executable='camera_stream_publisher',
        name='camera_publisher'
    )
    subscriber_node = Node(
        package='mock_camera_pkg',
        executable='camera_stream_subscriber',
        name='camera_subscriber'
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node,
        ReadyToTest()
    ])

# Test de integrare pentru toate testele funcționale
def test_functional_integration(rclpy_init):
    test_positive_publisher_initialization(rclpy_init)
    test_positive_stream_activation(rclpy_init)
    test_negative_infrared_disabled(rclpy_init)
    test_positive_message_content(rclpy_init)

# Test de integrare pentru toate testele de gestionare a erorilor
def test_error_handling_integration(rclpy_init):
    test_positive_publisher_initialization(rclpy_init)
    test_rgb_fps_invalid_range(rclpy_init)
    test_infrared_fps_invalid_range(rclpy_init)
    test_negative_missing_config_parameter(rclpy_init)

# Test de integrare pentru toate testele de performanță
def test_performance_integration(rclpy_init):
    test_rgb_fps_valid_range(rclpy_init)
    test_infrared_fps_valid_range(rclpy_init)
    test_latency(rclpy_init)
    test_stability(rclpy_init)
    test_resource_utilization(rclpy_init)
    test_error_recovery(rclpy_init)
    test_stress(rclpy_init)
    test_throughput(rclpy_init)

# Test de integrare pentru publisher și subscriber
@pytest.mark.launch_test
def test_camera_stream_integration(rclpy_init):
    publisher_node = CameraStreamPublisher()
    subscriber_node = CameraStreamSubscriber()

    # Publică un mesaj
    publisher_node.publish_message()

    # Așteaptă să primească mesajul
    start_time = publisher_node.get_clock().now()
    while (publisher_node.get_clock().now() - start_time).nanoseconds < 5e9:  # 5 secunde
        rclpy.spin_once(subscriber_node, timeout_sec=0.1)
        if subscriber_node.received_msg is not None:
            break

    # Verifică dacă mesajul a fost primit
    assert subscriber_node.received_msg is not None, "Subscriber did not receive the message"
    assert subscriber_node.received_msg == "Test message", "Received message does not match"

# Rulează toate testele de integrare
if __name__ == "__main__":
    pytest.main([__file__, "--html=report_integration.html", "--self-contained-html", "--tb=short"])
