from setuptools import setup

package_name = 'json_mqtt_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Instituto Tecnológico de Aragón',
    maintainer_email='ita@ita.es',
    description='Bridge between ROS2 messages and JSON through a MQTT broker',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = json_mqtt_bridge.json_mqtt_bridge.bridge_mqtt_node:main'
        ],
    },
)
