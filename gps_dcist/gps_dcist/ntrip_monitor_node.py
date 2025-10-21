#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rtcm_msgs.msg import Message as RTCMMessage
import numpy as np

from ros_system_monitor_msgs.msg import NodeInfoMsg

NTRIP_ERROR_TIMEOUT = 10.0  # seconds without a NTRIP message to consider it an error

# ============================================================
# https://github.com/MIT-SPARK/ROS-System-Monitor/blob/main/ros_system_monitor_msgs/msg/NodeInfoMsg.msg
# ------------------------------------------------------------
# NodeInfoMsg.NOMINAL = 1   # The node is functioning normally
# NodeInfoMsg.WARNING = 2   # The node is partially failing but still functioning
# NodeInfoMsg.ERROR   = 3   # The node is no longer functioning
# NodeInfoMsg.NO_HB   = 4   # The node has not been seen for long enough that it is considered missing
# NodeInfoMsg.STARTUP = 5   # The node is still initializing or has not been seen yet
# ============================================================

class NTRIPMonitorNode(Node):

    def __init__(self):
        super().__init__('ntrip_monitor_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ("robot_id", 0),              # robot id for this node
                ("nickname", "ntrip_monitor"),  # default nickname
            ]
        )

        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value
        self.nickname = self.get_parameter("nickname").get_parameter_value().string_value

        self.status_pub = self.create_publisher(NodeInfoMsg, "ntrip/status", qos_profile=QoSProfile(depth=10))
        self.create_subscription(RTCMMessage, "rtcm", self.ntrip_cb, 10) # TODO fix namespacing
        self.timer = self.create_timer(1.0, self.timer_cb)
        
        self.status = NodeInfoMsg.STARTUP
        self.last_message = None
        self.last_recv_timestamp = self.get_clock().now()

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

    def ntrip_cb(self, msg: RTCMMessage):
        #self.get_logger().debug(f"Received GPS fix: {msg}")
        self.get_logger().debug(f"Received GPS message")

        self.last_recv_timestamp = self.get_clock().now()
        self.last_message = msg.message

        if self.last_message is not None:
            self.status = NodeInfoMsg.NOMINAL
        else:
            self.status = NodeInfoMsg.ERROR

    def timer_cb(self):
        status = self.status
        time_since_last_recv = self.get_clock().now() - self.last_recv_timestamp
        secs = time_since_last_recv.nanoseconds / 1e9
        #if time_since_last_fix > rclpy.duration.Duration(seconds=GPS_ERROR_TIMEOUT):
        if secs > NTRIP_ERROR_TIMEOUT:
            self.status = NodeInfoMsg.ERROR
            status = NodeInfoMsg.ERROR

        if status == NodeInfoMsg.NOMINAL:
            self.log_and_send_status(f"NTRIP message received [Δt = {secs:.02f}s since last msg]", status)
        elif status == NodeInfoMsg.ERROR:
            self.log_and_send_status(f"No NTRIP message [Δt = {secs:.02f}s since last msg]", status)
        elif status == NodeInfoMsg.STARTUP:
            self.log_and_send_status("Waiting for first NTRIP message", status)
        else:
            self.log_and_send_status("Unknown status", NodeInfoMsg.ERROR)

def main(args=None):
    rclpy.init(args=args)
    node = NTRIPMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()