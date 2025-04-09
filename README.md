# Pilot02 Connectivity Json Bridge ROS2 MQTT (ROS2MQTTJsonBridge)

This repository contains a bidirectional bridge between ROS2 and MQTT, enabling message exchange between ROS2 systems and other distributed systems through MQTT brokers using JSON serialization.

Traditional ROS2 systems use binary message formats, which are not easily consumable by many cloud-based or non-ROS systems. Existing solutions like [`mqtt_bridge`](https://github.com/groove-x/mqtt_bridge) are no longer actively maintained. An alternative, [`mqtt_client`](https://github.com/ika-rwth-aachen/mqtt_client), supports bidirectional MQTT communication but transmits messages in a binary format, limiting interoperability.

**ROS2MQTTJsonBridge** provides a lightweight, extensible, and container-ready solution that enables:
- **Bidirectional communication** between ROS2 nodes and MQTT clients.
- **Automatic serialization and deserialization** of ROS2 messages into **JSON** format for improved interoperability.
- Seamless integration with edge devices running ROS2 and cloud-based MQTT ecosystems.

Features:
- Bidirectional bridging (ROS2 ↔ MQTT)
- Automatic JSON serialization/deserialization for ROS2 messages
- Configurable topic mapping (ROS ↔ MQTT)
- Docker support for containerized deployments

The repository includes the following components:
- Configuration files (how to address the bridge from ROS->MQTT and MQTT->ROS including the automatic serializer and deserializer)
- Docker setup for easy containerization
- A Docker Compose-based test environment to verify end-to-end functionality: ROS publisher → MQTT bridge → MQTT broker → Python MQTT client; Python MQTT client → MQTT broker → MQTT bridge → ROS subscriber.

## Guidelines for build and test the component 

### 1. **Build the Main Docker Image:**

In this step, we build the Docker image using the provided `Dockerfile`. The image is named `pilot02-connectivity-bridge-ros2-json-mqtt`.

```bash
cd src
docker build -t pilot02-connectivity-bridge-ros2-json-mqtt .
```
Make sure the path to your configuration and launch files is correctly mapped to the Docker container.

### 2. **Run the ROS 2 Container:**

After building the Docker image, you can run the container using the following command:

```bash
docker run -e CONFIG_FILE=your_custom_config_file pilot02-connectivity-bridge-ros2-json-mqtt
```

This will start the container and launch the MQTT bridge with the configuration given.

### 3. **Build and Run the test automation:**

Test automation is integrated by docker-compose file:

Run: 
```bash
docker-compose up --build
```

After execution, you will observe how messages published from ROS2 reach the Python MQTT client after being serialized into JSON. Conversely, commands sent from the MQTT client are received by the ROS2 subscriber, having been deserialized and converted back into the appropriate ROS2 message format.

```python
ros_json_mqtt_bridge  | ==============================
ros_json_mqtt_bridge  | ✅ Using configuration file: /ros_ws/config/config.yaml
ros_json_mqtt_bridge  | --- Starting node ---
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:✅ Configuration loaded successfully
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔹 Broker Host: mqtt_broker
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔹 Broker Port: 1883
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔹 Client ID: test_ita
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔹 Keep Alive: 60
ros_json_mqtt_bridge  | /ros_ws/src/json_mqtt_bridge/json_mqtt_bridge/bridge_mqtt_node.py:205: DeprecationWarning: Callback API version 1 is deprecated, update to latest version
ros_json_mqtt_bridge  |   client = mqtt.Client(client_id=client_config['id'], transport=transport)
mqtt_broker           | 1744185066: New connection from 192.168.112.3:38517 on port 1883.
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔄 Connecting to MQTT broker at mqtt_broker:1883...
mqtt_broker           | 1744185066: New client connected from 192.168.112.3:38517 as test_ita (p2, c1, k60).
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔄 Setting up mappings...
ros_json_mqtt_bridge  | Message type: <class 'demo_messages.msg._demo_state.DemoState'>
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔍 Mapping: {'action': 'serialize', 'type': 'ros2mqtt', 'input': '/demostatemessage', 'output': '/demostatemessage', 'msg_type': 'demo_messages/msg/DemoState'}
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:✅ Subscribed to ROS2 topic: /demostatemessage
ros_json_mqtt_bridge  | Message type: <class 'demo_messages.msg._demo_command.DemoCommand'>
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔍 Mapping: {'action': 'deserialize', 'input': '/democommandmessage', 'type': 'mqtt2ros', 'output': '/democommandmessage', 'msg_type': 'demo_messages/msg/DemoCommand'}
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:✅ Subscribed to MQTT topic: /democommandmessage
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:✅ Created ROS2 publisher for topic: /democommandmessage
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:✅ ROS2 and MQTT topic mappings set up
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:✅ Connected to MQTT broker
ros_subscriber        | [INFO] [1744185067.180438486] [demo_command_subscriber]: DemoCommandSubscriber started!
ros_publisher         | [INFO] [1744185067.271828414] [demo_state_publisher]: DemoStatePublisher started!
mqtt_test_client      | Collecting paho-mqtt
mqtt_test_client      |   Downloading paho_mqtt-2.1.0-py3-none-any.whl (67 kB)
mqtt_test_client      |      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 67.2/67.2 kB 2.8 MB/s eta 0:00:00
mqtt_test_client      | Installing collected packages: paho-mqtt
mqtt_test_client      | Successfully installed paho-mqtt-2.1.0
mqtt_test_client      | WARNING: Running pip as the 'root' user can result in broken permissions and conflicting behaviour with the system package manager. It is recommended to use a virtual environment instead: https://pip.pypa.io/warnings/venv
mqtt_test_client      |
mqtt_test_client      | [notice] A new release of pip is available: 23.0.1 -> 25.0.1
mqtt_test_client      | [notice] To update, run: pip install --upgrade pip
ros_publisher         | [INFO] [1744185068.281007790] [demo_state_publisher]: Publishing DemoState message: demo_messages.msg.DemoState(is_active=True, error_code=0, battery_level=87.5, status_message='Status update 1', position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=1.0), header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1744185068, nanosec=279995002), frame_id='base_link'), sensor_readings=[10, 20, 30])
ros_publisher         | [INFO] [1744185068.281873880] [demo_state_publisher]: Published DemoState message #1
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:📩 Received ROS message on topic: /demostatemessage
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔍 Action: serialize, Output: /demostatemessage, Type: ros2mqtt
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:📤 ROS to MQTT: /demostatemessage → /demostatemessage
mqtt_test_client      | /test/mqtt_client_launch.py:14: DeprecationWarning: Callback API version 1 is deprecated, update to latest version
mqtt_test_client      |   client = mqtt.Client()
mqtt_broker           | 1744185068: New connection from 192.168.112.4:41115 on port 1883.
mqtt_broker           | 1744185068: New client connected from 192.168.112.4:41115 as auto-2F3B38D5-B70F-171E-8342-7CEED9FFAFDA (p2, c1, k60).
mqtt_test_client      | ⏳ Listening to `/demostatemessage` (ROS → MQTT)...
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:📩 Received ROS message on topic: /demostatemessage
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔍 Action: serialize, Output: /demostatemessage, Type: ros2mqtt
ros_publisher         | [INFO] [1744185069.290884424] [demo_state_publisher]: Publishing DemoState message: demo_messages.msg.DemoState(is_active=True, error_code=0, battery_level=87.5, status_message='Status update 2', position=geometry_msgs.msg.Point(x=1.0, y=0.5, z=1.0), header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1744185069, nanosec=289970608), frame_id='base_link'), sensor_readings=[11, 21, 31])
mqtt_test_client      | 📥 Received on topic `/demostatemessage`: {"is_active": true, "error_code": 0, "battery_level": 87.5, "status_message": "Status update 2", "position": {"x": 1.0, "y": 0.5, "z": 1.0}, "header": {"stamp": {"sec": 1744185069, "nanosec": 289970608}, "frame_id": "base_link"}, "sensor_readings": [11, 21, 31]}
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:📤 ROS to MQTT: /demostatemessage → /demostatemessage
ros_publisher         | [INFO] [1744185069.291948406] [demo_state_publisher]: Published DemoState message #2
ros_publisher         | [INFO] [1744185070.280289924] [demo_state_publisher]: Publishing DemoState message: demo_messages.msg.DemoState(is_active=True, error_code=0, battery_level=87.5, status_message='Status update 3', position=geometry_msgs.msg.Point(x=2.0, y=1.0, z=1.0), header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1744185070, nanosec=279398977), frame_id='base_link'), sensor_readings=[12, 22, 32])
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:📩 Received ROS message on topic: /demostatemessage
ros_json_mqtt_bridge  | INFO:bridge_mqtt_node:🔍 Action: serialize, Output: /demostatemessage, Type: ros2mqtt
[rest of logs...]
```

## Bridge configuration

Configuration for the ROS2MQTTJsonBridge is provided via a YAML file, which includes broker settings, client behavior, buffering, and ROS2-MQTT topic mappings.

### Broker Configuration (broker)

Defines the MQTT broker connection settings.

+ protocol: `mqtt` (string, required) - Protocol to use (`mqtt`, `ws`, `wss`)
+ host: `mqtt_broker` (string, required) - MQTT broker hostname or IP
+ port: `1883` (number, required) - MQTT broker port
+ user: `""` (string, optional) - Username for authentication (if required)
+ pass: `""` (string, optional) - Password for authentication (if required)
+ tls (object, optional) - TLS/SSL settings for secure connections
  + enabled: `false` (boolean) - Whether to enable TLS
  + ca_certificate: `null` (string or null) - Path to CA certificate
  + validate_certificate: `false` (boolean) - Whether to validate TLS certificate

### MQTT Client Settings (client)

Configuration of the MQTT client behavior and reliability features.

+ id: `test_ita` (string, required) - Unique client ID
+ clean_session: `true` (boolean, optional) - Whether to start with a clean session
+ keep_alive_interval: `60` (number, optional) - Keep-alive interval in seconds
+ max_inflight: `65535` (number, optional) - Max number of in-flight messages
+ buffer (object, optional) - Offline message buffer settings
  + size: `100` (number) - Number of messages to buffer
  + directory: `"buffer"` (string) - Directory path for persistent message buffer

### Topic Mappings (mappings)

Defines bidirectional bridges between ROS2 topics and MQTT topics.

+ mappings (array[Mapping]) - List of topic mapping definitions

#### Mapping Object

Each mapping defines the direction and type of translation between a ROS2 topic and an MQTT topic.

+ action: `serialize` or `deserialize` (string, required)
  - `serialize`: Converts ROS2 messages to JSON and publishes to MQTT
  - `deserialize`: Converts JSON payload from MQTT into ROS2 messages
+ type: `ros2mqtt` or `mqtt2ros` (string, required)
  - Direction of the message flow
+ input: `/demostatemessage` (string, required) - Source topic (ROS2 or MQTT)
+ output: `/demostatemessage` (string, required) - Destination topic (ROS2 or MQTT)
+ msg_type: `demo_messages/msg/DemoState` (string, required) - ROS2 message type in `package/msg/Message` format

#### Example Mappings

```yaml
mappings:
  - action: 'serialize'
    type: 'ros2mqtt' 
    input: '/demostatemessage'
    output: '/demostatemessage'
    msg_type: 'demo_messages/msg/DemoState' 

  - action: 'deserialize'
    type: 'mqtt2ros' 
    input: '/democommandmessage'
    output: '/democommandmessage'
    msg_type: 'demo_messages/msg/DemoCommand' 
```

## Contributing

Feel free to open issues or submit pull requests. Contributions are welcome!

## License

This project is licensed under the Apache2 - see the [LICENSE](LICENSE) file for details.