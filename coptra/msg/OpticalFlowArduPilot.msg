# Optical Flow message compatible with ArduPilot format
# Based on OpenMV optical flow output format

Header header

# Flow rates in rad/s (not used by OpenMV, set to 0)
float32 flow_rate_x
float32 flow_rate_y

# Pixel shifts (dpix) - main data from OpenMV
int16 flow_x
int16 flow_y

# Quality of the optical flow measurement (0-255)
uint8 quality

# Sensor ID for multiple sensors
uint8 sensor_id
