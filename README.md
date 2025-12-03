# Jetson_GPS_Ros2
jetson_gps_ros2 is a ROS 2 workspace for NVIDIA Jetson that reads NMEA data from a GPS module over UART and publishes it as standard sensor_msgs/NavSatFix messages on the /fix topic. It includes a simple Python node (gps_node) using pynmea2 to parse GPS data and log satellites, fix quality, and basic position info.
