#!/usr/bin/env python3
"""
Clock Sync Client
------------------
Runs on the base station. Calls the ground robot's /sync_clock service with
this machine's own current time, so the robot's clock gets corrected over
the ROS2 network instead of SSH. One-shot: reports the result and exits, so
it's suited to being a step in a launch file.
"""
import sys

import rclpy
from rclpy.node import Node

from custom_msgs.srv import SyncClock


class ClockSyncClient(Node):

    def __init__(self):
        super().__init__('clock_sync_client')
        self.client = self.create_client(SyncClock, 'sync_clock')

    def run(self) -> bool:
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('sync_clock service not available on the ground robot.')
            return False

        req = SyncClock.Request()
        req.reference_time = self.get_clock().now().to_msg()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        result = future.result()
        if result is None:
            self.get_logger().error('sync_clock call failed (no response).')
            return False
        if result.success:
            self.get_logger().info(f'Clock sync: {result.message}')
        else:
            self.get_logger().error(f'Clock sync failed: {result.message}')
        return result.success


def main(args=None):
    rclpy.init(args=args)
    node = ClockSyncClient()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
