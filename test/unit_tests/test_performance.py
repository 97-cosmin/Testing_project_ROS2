import pytest
import rclpy
import time
import os
import yaml
import random
from std_msgs.msg import String
from mock_camera_pkg.camera_stream_publisher import CameraStreamPublisher
import psutil  # Pentru monitorizarea utilizarii resurselor

# Fixture pentru initializarea si inchiderea ROS 2
@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()

# Functie utilitara pentru a verifica daca fisierul de configurare exista
def check_config_file(config_path):
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file '{config_path}' not found.")

# Test 5: Verifica FPS pentru RGB (caz valid)
def test_rgb_fps_valid_range(rclpy_init):
    try:
        publisher_node = CameraStreamPublisher()
        
        # Verifica daca FPS-ul RGB este in intervalul valid (15-60)
        expected_min_fps = 30
        expected_max_fps = 60
        actual_min_fps = publisher_node.rgb_fps_min
        actual_max_fps = publisher_node.rgb_fps_max

        # Verify minimum FPS
        assert actual_min_fps == expected_min_fps, (
            f"RGB FPS min ({actual_min_fps}) is not {expected_min_fps}. "
            f"Expected value: {expected_min_fps}, Actual value: {actual_min_fps}"
        )

        # Verify maximum FPS
        assert actual_max_fps == expected_max_fps, (
            f"RGB FPS max ({actual_max_fps}) is not {expected_max_fps}. "
            f"Expected value: {expected_max_fps}, Actual value: {actual_max_fps}"
        )

        print(f"Test 5: RGB FPS valid range test: PASSED. "
              f"Values: FPS min = {actual_min_fps}, FPS max = {actual_max_fps}")
    except Exception as e:
        pytest.fail(f"Test 5: RGB FPS valid range test: FAILED - {str(e)}")

# Test 7: Verifica FPS pentru Infrarosu (caz valid)
def test_infrared_fps_valid_range(rclpy_init):
    try:
        publisher_node = CameraStreamPublisher()
        
        # Verifica daca FPS-ul Infrarosu este in intervalul valid (30-120)
        assert publisher_node.infrared_fps_min == 30, f"Infrared FPS min ({publisher_node.infrared_fps_min}) is not 30"
        assert publisher_node.infrared_fps_max == 120, f"Infrared FPS max ({publisher_node.infrared_fps_max}) is not 120"
        print("Test 7: Infrared FPS valid range test: PASSED")
    except Exception as e:
        pytest.fail(f"Test 7: Infrared FPS valid range test: FAILED - {str(e)}")

# Test de Performanta: Test Latenta cu date random
def test_latency(rclpy_init):
    publisher_node = CameraStreamPublisher()
    
    # Simuleaza valori FPS random pentru testul de latenta
    random_fps = random.randint(15, 60)
    publisher_node.rgb_fps_min = random_fps
    publisher_node.rgb_fps_max = random_fps

    # Măsoară latenta
    start_time = time.time()
    publisher_node.run_latency_test()
    latency = time.time() - start_time
    
    # Verifica daca latenta este sub un prag (de ex. 0.05 secunde)
    assert latency < 0.05, f"Latency too high: {latency:.4f} seconds"
    print(f"Latency Test PASSED. Latency = {latency:.4f} seconds")

# Test de Performanta: Test Stabilitate cu valori FPS si rezolutie random
def test_stability(rclpy_init):
    publisher_node = CameraStreamPublisher()

    # Simuleaza valori random pentru FPS si rezolutie
    random_fps = random.randint(15, 60)
    random_resolution = random.choice(['1280x720', '1920x1080'])
    publisher_node.rgb_fps_min = random_fps
    publisher_node.rgb_fps_max = random_fps
    publisher_node.rgb_resolution = random_resolution

    # Ruleaza testul de stabilitate
    publisher_node.run_stability_test()

    # Verifica daca nodul este stabil (fara blocaje sau erori)
    # Sa facem testul sa esueze daca FPS-ul este mai mare de 50 si rezolutia este 1920x1080
    if random_fps > 50 and random_resolution == '1920x1080':
        publisher_node.stability_status = 'unstable'
    
    assert publisher_node.stability_status == 'stable', f"Stability check failed. Status: {publisher_node.stability_status}"
    print(f"Stability Test PASSED. Resolution = {random_resolution}, FPS = {random_fps}")

# Test de Performanta: Test Utilizare Resurse cu date random
def test_resource_utilization(rclpy_init):
    publisher_node = CameraStreamPublisher()

    # Simuleaza utilizarea CPU random
    random_cpu_limit = random.uniform(50, 90)
    publisher_node.run_resource_utilization_test()

    # Verifica daca utilizarea CPU este sub un prag rezonabil (de ex. 80%)
    cpu_usage = psutil.cpu_percent(interval=1)
    
    
    # Sa facem testul sa esueze daca utilizarea CPU este mai mare de 80%
    assert cpu_usage < 80, f"CPU usage too high: {cpu_usage}%"
    print(f"Resource Utilization Test PASSED. CPU usage = {cpu_usage}%")

# Test de Performanta: Test Recuperare Erori
def test_error_recovery(rclpy_init):
    publisher_node = CameraStreamPublisher()

    # Simuleaza un scenariu de recuperare dupa eroare random
    error_scenario = random.choice(['disconnect', 'timeout', 'data_corruption'])
    publisher_node.run_error_recovery_test(error_scenario)

    # Verifica daca eroarea este tratata corect
    # Sa facem testul sa esueze daca scenariul de eroare este 'coruptie_date'
    if error_scenario == 'data_corruption':
        publisher_node.error_recovery_status = 'failed'
    
    assert publisher_node.error_recovery_status == 'recovered', f"Error recovery failed for {error_scenario}. Status: {publisher_node.error_recovery_status}"
    print(f"Error Recovery Test PASSED. Error scenario: {error_scenario}")

# Test de Performanta: Test Stres cu date random
def test_stress(rclpy_init):
    publisher_node = CameraStreamPublisher()

    # Simuleaza un scenariu de incarcare mare cu valori FPS random
    random_fps = random.randint(15, 120)
    publisher_node.rgb_fps_min = random_fps
    publisher_node.rgb_fps_max = random_fps

    # Ruleaza testul de stres
    publisher_node.run_stress_test()

    # Verifica daca testul de stres este gestionat fara esec
    # Sa facem testul sa esueze daca FPS-ul este mai mare de 100
    if random_fps > 100:
        publisher_node.stress_status = 'failed'
    
    assert publisher_node.stress_status == 'passed', f"Stress test failed. Status: {publisher_node.stress_status}"
    print(f"Stress Test PASSED. FPS = {random_fps}")

# Test de Performanta: Test Throughput cu date random
def test_throughput(rclpy_init):
    publisher_node = CameraStreamPublisher()

    # Simuleaza un scenariu de throughput random
    random_throughput = random.randint(100, 1000)  # Mbps
    publisher_node.run_throughput_test(random_throughput)

    # Verifica daca throughput-ul este peste un prag minim (de ex. 300 Mbps)
    assert random_throughput > 300, f"Throughput too low: {random_throughput} Mbps"
    print(f"Throughput Test PASSED. Throughput = {random_throughput} Mbps")

