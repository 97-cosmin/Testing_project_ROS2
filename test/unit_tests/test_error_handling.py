import pytest
import rclpy
import os
import yaml
from mock_camera_pkg.publisher_for_errors import CameraStreamPublisher_for_errors

# Calea catre directorul mock_camera_pkg
MOCK_CAMERA_PKG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),  # Mergi in sus pana la mock_camera_pkg
    'mock_camera_pkg'  # Intra in directorul mock_camera_pkg
)

# Fixture pentru initializarea si inchiderea ROS 2
@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()

# Fixture pentru a crea un fisier de configurare temporar
@pytest.fixture
def temp_config_file(tmpdir):
    config_path = tmpdir.join("camera_config.yaml")
    yield str(config_path)
    if os.path.exists(config_path):
        os.remove(config_path)

# Test 1: Initializare cu fisier de configurare valid
def test_positive_publisher_initialization(rclpy_init, temp_config_file):
    # Scriem un fisier de configurare valid
    with open(temp_config_file, 'w') as f:
        yaml.dump({
            'camera': {
                'topic': 'camera_stream',
                'rgb': {'resolution': '1080p', 'fps': {'min': 30, 'max': 60}},
                'infrared': {'enabled': True, 'fps': {'min': 30, 'max': 60}},
                'depth': {
                    'min_depth_distance': 0.5,
                    'ideal_range': {'min': 0.5, 'max': 4.0},
                    'accuracy': {'min': 90, 'max': 100},
                    'resolution': '720p',
                    'fps': {'min': 30, 'max': 60}
                }
            }
        }, f)
    try:
        publisher = CameraStreamPublisher_for_errors()
        assert publisher is not None
        print("Test 1: PASSED - Publisher initializat cu succes cu un fisier de configurare valid.")
    except Exception as e:
        pytest.fail(f"Test 1: FAILED - {str(e)}")

# Test 2: Initializare cu fisier de configurare lipsa
def test_negative_publisher_initialization_missing_config(rclpy_init):
    print("Se ruleaza testul pentru fisierul de configurare lipsa...")
    invalid_config_path = os.path.join(MOCK_CAMERA_PKG_DIR, 'invalid_config.yaml')
    if os.path.exists(invalid_config_path):
        os.remove(invalid_config_path)

    try:
        print("Se initializeaza publisherul...")
        publisher = CameraStreamPublisher_for_errors()
        print("Publisher initializat in mod neașteptat")
        pytest.fail("Test 2: FAILED - Nu s-a ridicat exceptia pentru lipsa fisierului de configurare.")
    except FileNotFoundError as e:
        print(f"Test 2: PASSED - S-a ridicat corect FileNotFoundError: {str(e)}")
    except Exception as e:
        print(f"Test 2: FAILED - Exceptie neasteptata: {str(e)}")

# Test 3: Fisier de configurare invalid (YAML malformat)
def test_negative_publisher_initialization_invalid_yaml(rclpy_init, temp_config_file):
    with open(temp_config_file, 'w') as f:
        f.write("invalid: yaml: content")
    try:
        publisher = CameraStreamPublisher_for_errors()
    except yaml.YAMLError:
        print("Test 3: PASSED - S-a ridicat corect YAMLError pentru format YAML invalid.")
    except Exception as e:
        print(f"Test 3: FAILED - Eroare neasteptata: {str(e)}")

# Test 4: Parametru lipsa in fisierul de configurare
def test_negative_missing_config_parameter(rclpy_init, temp_config_file):
    with open(temp_config_file, 'w') as f:
        yaml.dump({'camera': {'rgb': {'resolution': '1080p'}}}, f)
    try:
        publisher = CameraStreamPublisher_for_errors()
    except KeyError:
        print("Test 4: PASSED - S-a ridicat corect KeyError pentru parametru lipsa in configurare.")
    except Exception as e:
        print(f"Test 4: FAILED - Eroare neasteptata: {str(e)}")

# Test 5: Valori FPS invalide pentru RGB
def test_rgb_fps_invalid_range(rclpy_init, temp_config_file):
    with open(temp_config_file, 'w') as f:
        yaml.dump({
            'camera': {
                'topic': 'camera_stream',
                'rgb': {'resolution': '1080p', 'fps': {'min': 60, 'max': 30}},  # FPS invalid
                'infrared': {'enabled': True, 'fps': {'min': 30, 'max': 60}},
                'depth': {
                    'min_depth_distance': 0.5,
                    'ideal_range': {'min': 0.5, 'max': 4.0},
                    'accuracy': {'min': 90, 'max': 100},
                    'resolution': '720p',
                    'fps': {'min': 30, 'max': 60}
                }
            }
        }, f)
    try:
        publisher = CameraStreamPublisher_for_errors()
    except ValueError:
        print("Test 5: PASSED - S-a ridicat corect ValueError pentru FPS invalid la RGB.")
    except Exception as e:
        print(f"Test 5: FAILED - Eroare neasteptata: {str(e)}")

# Test 6: Valori FPS invalide pentru Infrared
def test_infrared_fps_invalid_range(rclpy_init, temp_config_file):
    with open(temp_config_file, 'w') as f:
        yaml.dump({
            'camera': {
                'topic': 'camera_stream',
                'rgb': {'resolution': '1080p', 'fps': {'min': 30, 'max': 60}},
                'infrared': {'enabled': True, 'fps': {'min': 60, 'max': 30}},  # FPS invalid
                'depth': {
                    'min_depth_distance': 0.5,
                    'ideal_range': {'min': 0.5, 'max': 4.0},
                    'accuracy': {'min': 90, 'max': 100},
                    'resolution': '720p',
                    'fps': {'min': 30, 'max': 60}
                }
            }
        }, f)
    try:
        publisher = CameraStreamPublisher_for_errors()
    except ValueError:
        print("Test 6: PASSED - S-a ridicat corect ValueError pentru FPS invalid la Infrared.")
    except Exception as e:
        print(f"Test 6: FAILED - Eroare neasteptata: {str(e)}")

# Test 7: Fisier de configurare gol
def test_empty_config_file(rclpy_init, temp_config_file):
    with open(temp_config_file, 'w') as f:
        f.write("")  # Fisier gol
    try:
        publisher = CameraStreamPublisher_for_errors()
    except yaml.YAMLError:
        print("Test 7: PASSED - S-a ridicat corect YAMLError pentru fisier de configurare gol.")
    except Exception as e:
        print(f"Test 7: FAILED - Eroare neasteptata: {str(e)}")

# Test 8: Configuratie invalida
def test_invalid_config_structure(rclpy_init, temp_config_file):
    with open(temp_config_file, 'w') as f:
        yaml.dump({
            'invalid_key': 'invalid_value'  # Structura invalida
        }, f)

    try:
        publisher = CameraStreamPublisher_for_errors()  # Ar trebui sa ridice o exceptie
        pytest.fail("Test 8: FAILED - Exceptia asteptata nu a fost ridicata.")  # Daca ajunge aici, e un esec
    except KeyError as e:
        print(f"Test 8: PASSED - S-a ridicat corect KeyError pentru structura invalida a configuratiei.")
    except Exception as e:
        print(f"Test 8: FAILED - Eroare neasteptata: {str(e)}")

# Ruleaza toate testele
if __name__ == "__main__":
    pytest.main([__file__, "--html=report.html", "--self-contained-html", "--tb=short"])

