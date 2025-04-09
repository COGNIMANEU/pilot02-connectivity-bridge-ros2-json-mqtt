# demo_interfaces/msg/DemoState.msg

bool is_active
int32 error_code
float64 battery_level
string status_message

geometry_msgs/Point position
std_msgs/Header header

int32[] sensor_readings
