import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import yaml
import json
import ssl
import logging
import os
import sys

import importlib 
import typing
from typing import Any, Dict
from importlib import import_module
import numpy as np
import array

# Configure logger
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("bridge_mqtt_node")

def ros_to_json(msg: Any) -> Dict:
    """
    Convert a ROS2 message object to a JSON-serializable dictionary.
    
    Args:
        msg: The ROS2 message instance to convert
        
    Returns:
        Dictionary containing the message data that can be serialized to JSON
    """
    if msg is None:
        logger.debug("Received None message, returning None")
        return None
        
    #logger.debug(f"Starting serialization of message type: {type(msg)}")
    
    # Get all fields from the message
    fields = msg.get_fields_and_field_types()
    #logger.debug(f"Message fields: {fields}")
    
    result = {}
    for field_name, field_type in fields.items():
        try:
            #logger.debug(f"Processing field: {field_name} (type: {field_type})")
            value = getattr(msg, field_name)
            #logger.debug(f"Raw value: {value} (python type: {type(value)})")
            
            # Handle nested ROS messages recursively
            if hasattr(value, 'get_fields_and_field_types'):
                #logger.debug(f"Field {field_name} is a nested message")
                result[field_name] = ros_to_json(value)
                #logger.debug(f"Processed nested message for {field_name}")
            
            # Handle all array-like types (list, tuple, numpy array, array.array)
            elif isinstance(value, (list, tuple, np.ndarray)) or (hasattr(array, 'array') and isinstance(value, array.array)):
                #logger.debug(f"Field {field_name} is array-like (type: {type(value)})")
                # Convert to list first if needed
                if isinstance(value, (np.ndarray, array.array)):
                    value = value.tolist()
                # Process each element
                processed_list = []
                for i, item in enumerate(value):
                    #logger.debug(f"Processing item {i} of {field_name}")
                    if hasattr(item, 'get_fields_and_field_types'):
                        processed_list.append(ros_to_json(item))
                    else:
                        #logger.debug(f"Item {i} is primitive: {item}")
                        processed_list.append(item)
                result[field_name] = processed_list
                #logger.debug(f"Finished processing array for {field_name}")
            
            # Handle bytes
            elif isinstance(value, bytes):
                #logger.debug(f"Field {field_name} is bytes (length: {len(value)})")
                result[field_name] = list(value)
                #logger.debug(f"Converted bytes to list for {field_name}")
            
            # Handle primitive types
            else:
                result[field_name] = value
                #logger.debug(f"Set primitive value for {field_name}")
                
            
        except Exception as e:
            logger.error(f"❌ Failed to process field {field_name} with type {field_type}")
            logger.error(f"❌ Error details: {str(e)}")
            logger.error(f"❌ Value causing error: {value} (type: {type(value)})")
            raise ValueError(f"Failed to serialize field {field_name}") from e
            
    #logger.debug(f"Finished serializing message. Result: {result}")
    return result

def json_to_ros(json_data: Dict, msg_type: str) -> Any:
    """
    Convert a JSON-compatible dictionary to a ROS2 message object.
    
    Args:
        json_data: Dictionary containing the message data
        msg_type: The ROS2 message type (e.g., 'std_msgs/msg/String')
        
    Returns:
        An instance of the specified ROS2 message type
    """
    if json_data is None:
        return None
        
    # Parse the message type string (format: "package/msg/Type")
    parts = msg_type.split('/')
    if len(parts) != 3 or parts[1] != 'msg':
        raise ValueError(f"Invalid message type format: {msg_type}. Expected 'package/msg/Type'")
    
    package_name, _, message_name = parts
    
    # Import the module and get the message class dynamically
    try:
        module = importlib.import_module(f'{package_name}.msg')
        message_class = getattr(module, message_name)
    except (ImportError, AttributeError) as e:
        raise ValueError(f"Could not import message type {msg_type}: {str(e)}")
    
    # Create an empty message instance
    msg = message_class()
    
    # Get all fields from the message
    fields = msg.get_fields_and_field_types()
    
    for field_name, field_type in fields.items():
        if field_name not in json_data:
            continue  # Use default value if field not in JSON
            
        value = json_data[field_name]
        
        # Handle nested ROS messages recursively
        if hasattr(message_class, field_name) and hasattr(getattr(message_class, field_name), 'get_fields_and_field_types'):
            # For nested messages, construct the full type string
            nested_parts = field_type.split('/')
            if len(nested_parts) == 1:
                # Handle nested types in the same package
                nested_msg_type = f"{package_name}/msg/{field_type}"
            else:
                # Handle fully qualified nested types
                nested_msg_type = field_type.replace('.', '/')
            setattr(msg, field_name, json_to_ros(value, nested_msg_type))
        # Handle arrays/lists
        elif isinstance(value, (list, tuple)):
            element_type = field_type.split('[')[0]  # Extract type from something like 'int32[3]'
            if element_type in ['byte', 'char']:
                setattr(msg, field_name, bytes(value))
            else:
                setattr(msg, field_name, value)
        # Handle primitive types
        elif isinstance(value, list) and isinstance(value[0], (int, float)):  # For lists of numbers (e.g., floats or ints)
            setattr(msg, field_name, value)
        # Handle numpy ndarray when deserializing
        elif isinstance(value, list):
            setattr(msg, field_name, np.array(value))  # Convert list back to ndarray
        else:
            setattr(msg, field_name, value)
            
    return msg
    
class MQTTROSBridge(Node):
    def __init__(self, config_path):
        super().__init__('mqtt_ros_bridge')
        self.config = self.load_config(config_path)

        # MQTT Configuration
        self.mqtt_client = self.setup_mqtt_client()
        self.mqtt_topics = {}

        # ROS2 Publishers and Subscribers
        self.ros_publishers = {}
        self.ros_subscribers = {}

        # Setup ROS2 and MQTT mappings
        self.setup_mappings()

        # Start MQTT loop
        self.mqtt_client.loop_start()

    def load_config(self, path):
        """Load configuration from a YAML file."""
        if not os.path.exists(path):
            logger.error(f"Configuration file not found: {path}")
            exit(1)

        with open(path, 'r') as file:
            config = yaml.safe_load(file)

        logger.info("✅ Configuration loaded successfully")
        return config

    def setup_mqtt_client(self):
        """Initialize and configure the MQTT client."""
        broker_config = self.config['broker']
        client_config = self.config['client']

        logger.info(f"🔹 Broker Host: {broker_config.get('host')}")
        logger.info(f"🔹 Broker Port: {broker_config.get('port')}")
        logger.info(f"🔹 Client ID: {client_config.get('id')}")
        logger.info(f"🔹 Keep Alive: {client_config.get('keep_alive_interval')}")

        transport = "websockets" if broker_config['protocol'] in ['ws', 'wss'] else "tcp"
        client = mqtt.Client(client_id=client_config['id'], transport=transport)

        # Authentication
        if broker_config.get('user') and broker_config.get('pass'):
            client.username_pw_set(broker_config['user'], broker_config['pass'])

        # TLS Configuration
        if broker_config['tls']['enabled']:
            validate_cert = broker_config['tls'].get('validate_certificate', True)
            ca_cert = broker_config['tls'].get('ca_certificate', None) or None
            
            if not validate_cert:
                logger.warning("⚠️ SSL certificate validation is disabled!")
                client.tls_set(cert_reqs=ssl.CERT_NONE)
                client.tls_insecure_set(True)
            else:
                client.tls_set(ca_certs=ca_cert, cert_reqs=ssl.CERT_REQUIRED)
                client.tls_insecure_set(False)

        client.on_connect = self.on_mqtt_connect
        client.on_message = self.on_mqtt_message

        try:
            client.connect(broker_config['host'], broker_config['port'], client_config['keep_alive_interval'])
            logger.info(f"🔄 Connecting to MQTT broker at {broker_config['host']}:{broker_config['port']}...")
        except Exception as e:
            logger.error(f"❌ Failed to connect to MQTT broker: {e}")
            exit(1)

        return client

    def get_ros_message_type(self, msg_type_str):
        """Function to dynamically obtain the message type based on its name."""
        try:
            # Ensure the message type includes '/msg', even if it's missing in the config
            if '/msg/' not in msg_type_str:
                msg_type_str = msg_type_str.replace('/', '/msg/')  # Add '/msg/' if missing
            
            # Split the message type into the package and message name
            package_name, msg_name = msg_type_str.split('/')[0], msg_type_str.split('/')[2]
            
            # Dynamically import the package
            module = importlib.import_module(f"{package_name}.msg")
            
            # Get the message type (e.g., 'State' or 'Command')
            ros_msg_type = getattr(module, msg_name)
            
            return ros_msg_type
        except Exception as e:
            # If there's an error, print it and return String as a default value
            print(f"❌ Error getting message type: {e}")
            return String  # Fallback to String type if an error occurs

    def setup_mappings(self):
        """Set up ROS2 and MQTT topic subscriptions and publishers."""
        logger.info("🔄 Setting up mappings...")

        for mapping in self.config['mappings']:
            action = mapping['action']
            
            # Dynamically get the message type
            msg_type = self.get_ros_message_type(mapping['msg_type'])
            
            print(f"Message type: {msg_type}")
            logger.info(f"🔍 Mapping: {mapping}")

            if action == "serialize" or action == "none":
                if mapping['type'] in ["ros2mqtt", "ros2ros"]:
                    # Subscribe to ROS2 topics
                    self.ros_subscribers[mapping['input']] = self.create_subscription(
                        msg_type, mapping['input'], lambda msg, m=mapping: self.handle_ros_message(msg, m), 10
                    )
                    logger.info(f"✅ Subscribed to ROS2 topic: {mapping['input']}")

            elif action == "deserialize":
                if mapping['type'] == "mqtt2ros":
                    # Subscribe to MQTT topics
                    self.mqtt_topics[mapping['input']] = mapping
                    self.mqtt_client.subscribe(mapping['input'])
                    logger.info(f"✅ Subscribed to MQTT topic: {mapping['input']}")

            # Create ROS2 publishers
            if mapping['type'] in ["mqtt2ros", "ros2ros"]:
                self.ros_publishers[mapping['output']] = self.create_publisher(msg_type, mapping['output'], 10)
                logger.info(f"✅ Created ROS2 publisher for topic: {mapping['output']}")

        logger.info("✅ ROS2 and MQTT topic mappings set up")

        

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT connection callback."""
        if reason_code == 0:
            logger.info("✅ Connected to MQTT broker")
        else:
            logger.error(f"❌ Connection failed with code: {reason_code}")
          
        # Optionally log properties if you're using MQTT v5
        if properties:
            logger.info(f"📜 Connection properties: {properties}")

    def on_mqtt_message(self, client, userdata, msg):
        """MQTT message received callback."""
        
        logger.info(f"📩 Received MQTT message on topic: {msg.topic}")

        if msg.topic in self.mqtt_topics:
            mapping = self.mqtt_topics[msg.topic]
            action = mapping['action']

            if action == "deserialize":
                try:
                    # Parse the message payload into a dictionary
                    data = json.loads(msg.payload.decode())
                    
                    # Get the correct ROS message type dynamically
                    ros_msg_type = mapping['msg_type']
                    ros_msg = json_to_ros(data, ros_msg_type)  # Use the correct dynamic type
                    
                    # Publish the deserialized ROS message
                    self.ros_publishers[mapping['output']].publish(ros_msg)
                    
                    logger.info(f"📩 MQTT to ROS: {msg.topic} → {mapping['output']}")
                except Exception as e:
                    logger.error(f"❌ Failed to deserialize MQTT message: {e}")
            else:
                logger.warning(f"⚠ Unknown action for MQTT topic {msg.topic}")

    def handle_ros_message(self, msg, mapping):
        """ROS2 message received callback."""
        
        try:
            # Log the received message and action details
            logger.info(f"📩 Received ROS message on topic: {mapping['input']}")
            action = mapping['action']
            logger.info(f"🔍 Action: {action}, Output: {mapping.get('output', 'N/A')}, Type: {mapping['type']}")

            # Determine the type (ros2mqtt, ros2ros)
            if mapping['type'] == "ros2mqtt":
                # If action is serialize, we serialize the ROS message to JSON and publish it to MQTT
                if action == "serialize":
                    try:
                        json_msg = ros_to_json(msg)
                        self.mqtt_client.publish(mapping['output'], json.dumps(json_msg))
                        logger.info(f"📤 ROS to MQTT: {mapping['input']} → {mapping['output']}")
                    except Exception as e:
                        logger.error(f"❌ Error during serialization of ROS message on topic {mapping['input']}: {e}")
                
                # If action is none, publish the raw ROS message to MQTT (assuming it's in an acceptable format for MQTT)
                elif action == "none":
                    try:
                        self.mqtt_client.publish(mapping['output'], msg.data)  # Direct forwarding
                        logger.info(f"📤 ROS Direct Forwarding to MQTT: {mapping['input']} → {mapping['output']}")
                    except Exception as e:
                        logger.error(f"❌ Error during direct forwarding of ROS message on topic {mapping['input']}: {e}")
                
                else:
                    logger.warning(f"⚠ Unknown action for ROS topic {mapping['input']}")

            elif mapping['type'] == "ros2ros":
                # If action is serialize, we serialize the ROS message and publish it on another ROS topic
                if action == "serialize":
                    try:
                        json_msg = ros_to_json(msg)
                        # Assuming this logic also applies to ROS2 → ROS2 communication
                        self.ros_publishers[mapping['output']].publish(json_msg)  # Publish serialized message in ROS2
                        logger.info(f"📤 ROS to ROS: {mapping['input']} → {mapping['output']}")
                    except Exception as e:
                        logger.error(f"❌ Error during serialization of ROS message on topic {mapping['input']}: {e}")
                
                # If action is none, publish the raw ROS message to another ROS topic
                elif action == "none":
                    try:
                        self.ros_publishers[mapping['output']].publish(msg)  # Direct forwarding to another ROS topic
                        logger.info(f"📤 ROS Direct Forwarding to ROS: {mapping['input']} → {mapping['output']}")
                    except Exception as e:
                        logger.error(f"❌ Error during direct forwarding of ROS message on topic {mapping['input']}: {e}")
                
                else:
                    logger.warning(f"⚠ Unknown action for ROS topic {mapping['input']}")

            else:
                logger.warning(f"⚠ Unknown type for ROS topic {mapping['input']}")

        except Exception as e:
            logger.error(f"❌ Error in handle_ros_message callback for topic {mapping['input']}: {e}")

def main():
    """Main function to start the ROS2-MQTT bridge."""
    if len(sys.argv) < 2:
        print("❌ No configuration file provided. Usage: python3 bridge_mqtt_node.py <config_file>")
        sys.exit(1)

    config_path = sys.argv[1]

    rclpy.init()
    bridge = MQTTROSBridge(config_path)
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("🛑 Shutting down bridge...")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
