#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
import serial
import pynmea2


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        # --- Parameters ---
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("publish_rate", 2.0)  # Hz

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        # Last known GGA data
        self.num_sats = 0
        self.fix_quality = 0
        self.altitude_m = 0.0
        self.hdop = 0.0

        # Heading and previous position (for bearing estimate)
        self.heading_deg = 0.0
        self.prev_lat = None
        self.prev_lon = None

        # ROS 2 publishers
        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.heading_pub = self.create_publisher(Float64, "/gps/heading", 10)

        # Open serial port
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Opened {port} @ {baudrate}")
        except Exception as e:
            self.get_logger().error(f"Could not open serial: {e}")
            raise

        # Timer based on publish_rate parameter
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        if rate <= 0.0:
            rate = 2.0
        period = 1.0 / float(rate)
        self.create_timer(period, self.read_loop)

    def read_loop(self):
        try:
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not line or not line.startswith("$"):
                return

            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError as e:
                self.get_logger().warn(f"Failed to parse NMEA: {e}")
                return

            # --- GGA: sats, altitude, HDOP, fix quality ---
            if isinstance(msg, pynmea2.types.talker.GGA):
                try:
                    self.num_sats = int(msg.num_sats or 0)
                except ValueError:
                    self.num_sats = 0

                try:
                    self.fix_quality = int(msg.gps_qual or 0)
                except ValueError:
                    self.fix_quality = 0

                try:
                    self.altitude_m = float(msg.altitude or 0.0)
                except ValueError:
                    self.altitude_m = 0.0

                try:
                    self.hdop = float(msg.horizontal_dil or 0.0)
                except ValueError:
                    self.hdop = 0.0

                self.get_logger().info(
                    f"GGA: Sats={self.num_sats}, FixQ={self.fix_quality}, "
                    f"Alt={self.altitude_m:.2f} m, HDOP={self.hdop:.2f}"
                )

            # --- RMC: main position fix (lat/lon, status A, course over ground) ---
            if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":
                lat = msg.latitude
                lon = msg.longitude

                # 1) Try to use true_course if valid
                heading_from_rmc = None
                try:
                    if msg.true_course not in ("", None):
                        heading_from_rmc = float(msg.true_course)
                except Exception:
                    heading_from_rmc = None

                # 2) If no true_course, estimate bearing from previous GPS point
                if heading_from_rmc is None and self.prev_lat is not None:
                    heading_from_rmc = self._bearing_deg(self.prev_lat, self.prev_lon, lat, lon)

                # 3) If we got some heading (and movement is non-zero), update
                if heading_from_rmc is not None:
                    self.heading_deg = heading_from_rmc

                # store for next time
                self.prev_lat = lat
                self.prev_lon = lon

                # Build NavSatFix
                fix = NavSatFix()
                fix.header.stamp = self.get_clock().now().to_msg()
                fix.header.frame_id = (
                    self.get_parameter("frame_id")
                    .get_parameter_value()
                    .string_value
                )

                fix.latitude = lat
                fix.longitude = lon
                fix.altitude = self.altitude_m

                # Convert fix_quality to ROS NavSatStatus
                if self.fix_quality == 0:
                    fix.status.status = NavSatStatus.STATUS_NO_FIX
                else:
                    fix.status.status = NavSatStatus.STATUS_FIX

                fix.status.service = NavSatStatus.SERVICE_GPS

                # Very rough covariance estimate from HDOP
                if self.hdop > 0.0:
                    var = self.hdop ** 2
                    fix.position_covariance = [
                        var, 0.0, 0.0,
                        0.0, var, 0.0,
                        0.0, 0.0, var * 2.0,
                    ]
                    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                else:
                    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                # Publish NavSatFix
                self.fix_pub.publish(fix)

                # Publish heading as Float64 (degrees)
                heading_msg = Float64()
                heading_msg.data = self.heading_deg
                self.heading_pub.publish(heading_msg)

                self.get_logger().info(
                    f"Published NavSatFix: {lat:.6f}, {lon:.6f} | "
                    f"Sats: {self.num_sats}, HDOP: {self.hdop:.2f}, "
                    f"Heading: {self.heading_deg:.1f}°"
                )

        except Exception as e:
            self.get_logger().error(f"Error in read_loop: {e}")

    def _bearing_deg(self, lat1, lon1, lat2, lon2):
        """
        Compute bearing (degrees 0..360) from point 1 to point 2.
        Returns None if points are identical.
        """
        # convert to radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)

        x = math.sin(dlon) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)

        if abs(x) < 1e-9 and abs(y) < 1e-9:
            return None  # no movement

        bearing = math.degrees(math.atan2(x, y))
        bearing = (bearing + 360.0) % 360.0
        return bearing


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

