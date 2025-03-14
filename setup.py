from setuptools import setup, find_packages

package_name = 'mock_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Include toate pachetele, excluzând directorul 'test'
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Cosmin',
    maintainer_email='cosmin.ionoaia97@gmail.com',
    description='A simple ROS 2 subscriber and publisher for mock camera data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_camera_subscriber = mock_camera_pkg.mock_camera_subscriber:main',
            'mock_camera_publisher = mock_camera_pkg.mock_camera_publisher:main',
            'camera_stream_publisher = mock_camera_pkg.camera_stream_publisher:main',
            'camera_stream_subscriber = mock_camera_pkg.camera_stream_subscriber:main',
            'publisher_for_errors = mock_camera_pkg.publisher_for_errors:main',
            'subscriber_for_errors = mock_camera_pkg.camera_stream_subscriber:main',
            
            'Input_Validation_Publisher = mock_camera_pkg.Input_Validation_Publisher:main',
            'Input_Validation_Subscriber = mock_camera_pkg.Input_Validation_Subscriber:main',
        ],
    },
)
