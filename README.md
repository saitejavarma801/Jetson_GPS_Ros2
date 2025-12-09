# gps_ros2_ws – Jetson Orin + u-blox M8N GPS + Compass (ROS 2)

## **THIS REPOSITORY IS DEVELOPED FOR THE OUTDOOR NAV PROJECT**

This workspace contains a ROS 2 package **`gps_reader`** for using a **u-blox NEO-M8N GPS + compass module** with a **Jetson Orin**.

The main node:

- Reads NMEA data from the GPS over UART (`/dev/ttyTHS1`).
- Publishes `sensor_msgs/NavSatFix` on **`/fix`**.
- Publishes heading (course over ground, in degrees) on **`/gps/heading`**.
- Logs satellites, HDOP and altitude for debugging.
- Can be connected to other ROS 2 applications (e.g. OutdoorNav) that consume `/fix` and `/gps/heading`.

---

## 1. Hardware Connections

**Board:** Jetson Orin J41 header  
**Module:** u-blox M8N GPS + compass (6-wire)

GPS Module                           Jetson Orin J41

      [Red]  ----------------------->  Pin 2  (5V)
      [Black] ----------------------->  Pin 6  (GND)

      [Yellow - TX]  --------------->  Pin 8 (UART2 RX -> /dev/ttyTHS1)
      [Green  - RX]  --------------->  Pin 10  (UART2 TX)

Compass:

      [White  - SDA] --------------->  Pin 3  (I2C SDA)
      [Orange - SCL] --------------->  Pin 5  (I2C SCL)

---
### 2. Software Requirements

On the Jetson Orin:

- Ubuntu + ROS 2 (e.g. Humble)

- Python 3 with:

- rclpy

- pynmea2

- pyserial

Install the Python dependencies:

      python3 -m pip install --user pynmea2 pyserial

- Download the project gps_ros2_ws and unzip it
---


### 3. Build Instructions

       cd ~/gps_ros2_ws
   
       colcon build --packages-select gps_reader
   
       source install/setup.bash

---

### 4. Running the GPS Node

Run the node (default port: /dev/ttyTHS1, baud: 9600):

    cd ~/gps_ros2_ws
    source install/setup.bash
    ros2 run gps_reader gps_node

Example output:

    [INFO] [..] [gps_node]: Opened /dev/ttyTHS1 @ 9600
    [INFO] [..] [gps_node]: GGA: Sats=8, FixQ=1, Alt=475.50 m, HDOP=1.48
    [INFO] [..] [gps_node]: Published NavSatFix: 10.936478, 76.741579 | Sats: 8, HDOP: 1.48, Heading: 123.4°

----
### 5. ROS Topics

- /fix – sensor_msgs/NavSatFix

- Latitude, longitude, altitude from GPS

- Status and covariance estimated from HDOP

- /gps/heading – std_msgs/Float64

- Heading in degrees (0–360), based on GPS course over ground or movement between fixes.
  
You can view the data with tools like:

      ros2 topic echo /fix
      ros2 topic echo /gps/heading

----
MIT License

Copyright (c) 2025 SAI TEJA VARMA

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
...

Contact / Support

For questions, issues, or suggestions, feel free to reach out:

Author: SAI TEJA VARMA
Email: saitejavarma801@gmail.com
