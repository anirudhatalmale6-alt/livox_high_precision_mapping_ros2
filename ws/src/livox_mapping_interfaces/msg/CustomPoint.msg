# Livox custom point format (ROS2 port of the original livox_mapping/CustomPoint).
uint32 offset_time      # offset time relative to the base time (ns)
float32 x               # X axis, unit: m
float32 y               # Y axis, unit: m
float32 z               # Z axis, unit: m
uint8 reflectivity      # reflectivity, 0~255
uint8 line              # laser number in lidar
