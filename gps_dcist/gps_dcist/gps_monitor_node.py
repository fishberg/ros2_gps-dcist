#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix
import numpy as np

from ros_system_monitor_msgs.msg import NodeInfoMsg

GPS_ERROR_TIMEOUT = 10.0  # seconds without a GPS fix to consider it an error

# ============================================================
# https://github.com/MIT-SPARK/ROS-System-Monitor/blob/main/ros_system_monitor_msgs/msg/NodeInfoMsg.msg
# ------------------------------------------------------------
# NodeInfoMsg.NOMINAL = 1   # The node is functioning normally
# NodeInfoMsg.WARNING = 2   # The node is partially failing but still functioning
# NodeInfoMsg.ERROR   = 3   # The node is no longer functioning
# NodeInfoMsg.NO_HB   = 4   # The node has not been seen for long enough that it is considered missing
# NodeInfoMsg.STARTUP = 5   # The node is still initializing or has not been seen yet
# ============================================================

# ============================================================
# https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatStatus.html
# ------------------------------------------------------------
# NavSatFix.status.status = 0    # STATUS_NO_FIX
# NavSatFix.status.status = 1    # STATUS_FIX
# NavSatFix.status.status = 2    # STATUS_SBAS_FIX
# ------------------------------------------------------------
# Ublox package does not seem to use this field correctly.
# It seems like it does the following:
# -1 -> No fix, or extremely bad (i.e., no antenna attached)
#  0 -> Fix w/o RTK
#  2 -> RTK fix
# ============================================================

class GPSMonitorNode(Node):

    def __init__(self):
        super().__init__('gps_monitor_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ("robot_id", 0),              # robot id for this node
                ("nickname", "gps_monitor"),  # default nickname
            ]
        )

        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value
        self.nickname = self.get_parameter("nickname").get_parameter_value().string_value

        self.status_pub = self.create_publisher(NodeInfoMsg, "gps/status", qos_profile=QoSProfile(depth=10))
        self.create_subscription(NavSatFix, "ublox_gps_node/fix", self.gps_cb, 10) # TODO fix namespacing
        self.timer = self.create_timer(1.0, self.timer_cb)
        
        self.status = NodeInfoMsg.STARTUP
        self.last_pos = np.array([np.nan, np.nan, np.nan])
        self.last_cov = np.full((3,3), np.nan)
        self.last_fix_timestamp = self.get_clock().now()

    def log_and_send_status(self, note, status=NodeInfoMsg.NOMINAL):
        """
        Log a message and send it to the status topic.
        """
        self.get_logger().info(note)
        status_msg = NodeInfoMsg()
        status_msg.nickname = self.nickname
        status_msg.node_name = self.get_fully_qualified_name()
        status_msg.status = status
        status_msg.notes = note
        self.status_pub.publish(status_msg)

    def gps_status(self, msg: NavSatFix) -> str:
        """
        Determine the quality of the GPS fix from the NavSatFix message.
        """
        """
        if np.any(np.isnan([msg.latitude, msg.longitude, msg.altitude])):
            return "No Fix"
        elif np.all(~np.isnan([msg.latitude, msg.longitude, msg.altitude])):
            return "Fix"
        else:
            return "Unknown"
        """
        gps_status = msg.status.status
        if gps_status == -1:
            return "No Fix"
        elif gps_status == 0:
            return "Fix"
        elif gps_status == 2:
            return "RTK Fix"
        else:
            return "Unknown"

    def gps_cb(self, msg: NavSatFix):
        #self.get_logger().debug(f"Received GPS fix: {msg}")
        self.get_logger().debug(f"Received GPS message")

        self.last_fix_timestamp = self.get_clock().now()
        self.last_pos = np.array([msg.latitude, msg.longitude, msg.altitude])
        self.last_cov = msg.position_covariance

        gps_status = self.gps_status(msg)

        if (self.status != NodeInfoMsg.STARTUP) and (gps_status == "No Fix"):
            self.status = NodeInfoMsg.ERROR
        elif gps_status == "Fix":
            self.status = NodeInfoMsg.WARNING
        elif gps_status == "RTK Fix":
            self.status = NodeInfoMsg.NOMINAL

    def timer_cb(self):
        status = self.status
        time_since_last_fix = self.get_clock().now() - self.last_fix_timestamp
        secs = time_since_last_fix.nanoseconds / 1e9
        #if time_since_last_fix > rclpy.duration.Duration(seconds=GPS_ERROR_TIMEOUT):
        if secs > GPS_ERROR_TIMEOUT:
            self.status = NodeInfoMsg.ERROR
            status = NodeInfoMsg.ERROR

        # TODO note if just GPS or RTK is fixed?
        if status == NodeInfoMsg.NOMINAL:
            sigmax = np.round(self.last_cov[0],2)**0.5
            sigmay = np.round(self.last_cov[4],2)**0.5
            sigmaz = np.round(self.last_cov[8],2)**0.5
            self.log_and_send_status(f"RTK fix [σx = {sigmax:.02f}, σy = {sigmay:.02f}, σz = {sigmaz:.02f}]", status) #  |Σ|_F = {self.covariance_Fnorm}m]"
        elif status == NodeInfoMsg.WARNING:
            sigmax = np.round(self.last_cov[0],2)**0.5
            sigmay = np.round(self.last_cov[4],2)**0.5
            sigmaz = np.round(self.last_cov[8],2)**0.5
            self.log_and_send_status(f"GPS fix (No RTK) [σx = {sigmax:.02f}, σy = {sigmay:.02f}, σz = {sigmaz:.02f}]", status) #  |Σ|_F = {self.covariance_Fnorm}m]"
        elif status == NodeInfoMsg.ERROR:
            self.log_and_send_status(f"No GPS fix [Δt = {secs:.02f}s since last msg]", status)
        elif status == NodeInfoMsg.STARTUP:
            self.log_and_send_status("Waiting for first GPS fix", status)
        else:
            self.log_and_send_status("Unknown status", NodeInfoMsg.ERROR)

# class GPSMonitorNode(Node):
#     def __init__(self):
#         super().__init__('gps_monitor_node')

#         self.mygps_pub = self.create_publisher(NavSatFix, "mygps", qos_profile=QoSProfile(depth=10))
#         self.status_pub = self.create_publisher(NodeInfoMsg, "gps/status", qos_profile=QoSProfile(depth=10))
#         self.timer = self.create_timer(1.0, self.publish_empty_fix)

#     def publish_empty_fix(self):
#         msg = NavSatFix()
#         self.mygps_pub.publish(msg)

#         msg = NodeInfoMsg()
#         self.status_pub.publish(msg)

#         print("Published empty NavSatFix on /mygps")
#         self.get_logger().debug('Published empty NavSatFix on /mygps')

def main(args=None):
    rclpy.init(args=args)
    node = GPSMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()