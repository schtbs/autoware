#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

TAI_OFFSET_SEC = 37.0

class TaiClockPublisher(Node):
    def __init__(self):
        super().__init__('tai_clock_publisher')
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.005, self.publish_clock)  # 200 Hz

    def publish_clock(self):
        tai_ns = int((time.time() + TAI_OFFSET_SEC) * 1e9)
        msg = Clock()
        msg.clock.sec = tai_ns // 1_000_000_000
        msg.clock.nanosec = tai_ns % 1_000_000_000
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TaiClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()