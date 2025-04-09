import time
import json
import paho.mqtt.client as mqtt

# MQTT broker connection
MQTT_BROKER = "mqtt_broker"  # Change this to your broker address
MQTT_PORT = 1883

# Called whenever a message is received on a subscribed topic
def on_message(client, userdata, msg):
    print(f'📥 Received on topic `{msg.topic}`: {msg.payload.decode()}', flush=True)

# Setup client
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Subscribe to the ROS → MQTT topic
client.subscribe("/demostatemessage")  # This assumes ROS → MQTT is bridging this topic
client.on_message = on_message
client.loop_start()

print("⏳ Listening to `/demostatemessage` (ROS → MQTT)...", flush=True)
time.sleep(5)  # Give some time for messages to come in

print("🚀 Sending test `DemoCommand` messages (MQTT → ROS) every second for 10 seconds...", flush=True)

# Send one message per second for 10 seconds
for i in range(10):
    demo_command_msg = {
        "command_name": f"move_to_position_{i}",
        "target_value": i * 1.5,
        "execute_now": True if i % 2 == 0 else False
    }
    message_json = json.dumps(demo_command_msg)
    client.publish("/democommandmessage", message_json)
    print(f"📤 Published to `democommandmessage`: {message_json}", flush=True)
    time.sleep(1)

client.loop_stop()
print("✅ MQTT demo finished.", flush=True)
