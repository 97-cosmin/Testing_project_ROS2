import pytest
import rclpy
import time
import os
import yaml
from std_msgs.msg import String
from mock_camera_pkg.camera_stream_publisher import CameraStreamPublisher
from mock_camera_pkg.camera_stream_subscriber import CameraStreamSubscriber

# Fixture pentru inițializarea și închiderea ROS 2
@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()

# Funcție utilitară pentru a verifica existența fișierului de configurare
def check_config_file(config_path):
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Fișierul de configurare '{config_path}' nu a fost găsit.")

# Test 1: Inițializare corectă a publisher-ului
def test_positive_publisher_initialization(rclpy_init):
    try:
        publisher_node = CameraStreamPublisher()
        assert publisher_node is not None, "Publisher initialization failed"
        print("Test 1: Publisher initialization test: PASSED")
    except Exception as e:
        pytest.fail(f"Test 1: Publisher initialization test: FAILED - {str(e)}")



# Test 3: Verificare activare fluxuri
def test_positive_stream_activation(rclpy_init):
    try:
        publisher_node = CameraStreamPublisher()
        assert publisher_node.rgb_resolution is not None, "RGB stream not activated"
        assert publisher_node.infrared_enabled is not None, "Infrared stream not activated"
        assert publisher_node.depth_resolution is not None, "Depth stream not activated"
        print("Test 3: Stream activation test: PASSED")
    except Exception as e:
        pytest.fail(f"Test 3: Stream activation test: FAILED - {str(e)}")

# Test 4: Verificare dezactivare flux Infrared
def test_negative_infrared_disabled(rclpy_init):
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_config.yaml')
    try:
        check_config_file(config_path)  # Verificăm existența fișierului
    except FileNotFoundError as e:
        pytest.fail(f"Test 4: Infrared disabled test: FAILED - {str(e)}")

    with open(config_path, 'r') as f:
        original_config = yaml.safe_load(f)

    modified_config = original_config
    modified_config['camera']['infrared']['enabled'] = False

    with open(config_path, 'w') as f:
        yaml.safe_dump(modified_config, f)

    try:
        publisher_node = CameraStreamPublisher()
        assert publisher_node.infrared_enabled is False, "Infrared stream should be disabled"
        print("Test 4: Infrared disabled test: PASSED")
    except Exception as e:
        pytest.fail(f"Test 4: Infrared disabled test: FAILED - {str(e)}")
    finally:
        with open(config_path, 'w') as f:
            yaml.safe_dump(original_config, f)
            
# Test 9: Verificare conținut mesaj publicat
def test_positive_message_content(rclpy_init):
    try:
        publisher_node = CameraStreamPublisher()
        subscriber_node = CameraStreamSubscriber()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)

        publisher_node.publish_message()

        start_time = time.time()
        while time.time() - start_time < 5.0:
            executor.spin_once(timeout_sec=0.1)
            if subscriber_node.received_msg:
                break

        assert "RGB: 1280x800 at 15-60 FPS" in subscriber_node.received_msg, "RGB stream info missing"
        assert "Infrared: Enabled at 30-120 FPS" in subscriber_node.received_msg, "Infrared stream info missing"
        assert "Depth: 1280x720, 0.52m to 6.0m, Accuracy: 1.0%-2.0% at 30-90 FPS" in subscriber_node.received_msg, "Depth stream info missing"
        print("Test 9: Message content test: PASSED")
    except Exception as e:
        pytest.fail(f"Test 9: Message content test: FAILED - {str(e)}")
    finally:
        executor.remove_node(publisher_node)
        executor.remove_node(subscriber_node)
        executor.shutdown()


# Rulează toate testele
if __name__ == "__main__":
    pytest.main([__file__, f"--html={os.path.join(report_dir, 'report_functional.html')}", "--self-contained-html", "--tb=short"])


