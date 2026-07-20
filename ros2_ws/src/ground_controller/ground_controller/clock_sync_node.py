#!/usr/bin/env python3
"""
Clock Sync Node
----------------
Runs on the ground robot itself. Exposes a ROS2 service that lets the base
station push its own current time over, so the robot can correct its system
clock without SSH - only a shared ROS2 network is required, so this keeps
working even if the two machines aren't on the same physical LAN (e.g. a VPN
between ROS2 domains) as long as normal topic/service traffic reaches both
sides.

The robot's board has no battery-backed RTC, so its clock resets to a bogus
value on every power cycle. A stale clock makes tf2 report large
"extrapolation into the future/past" errors once AMCL/costmaps start
publishing timestamped data, since transform lookups are compared against
each node's own idea of "now".
"""
import subprocess
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node

from custom_msgs.srv import SyncClock

SKEW_THRESHOLD_SECONDS = 5.0


class ClockSyncNode(Node):

    def __init__(self):
        super().__init__('clock_sync_node')
        self.srv = self.create_service(SyncClock, 'sync_clock', self._on_sync_clock)
        self.get_logger().info('Clock sync service ready at /sync_clock.')

    def _on_sync_clock(self, request, response):
        ref = request.reference_time
        ref_sec = ref.sec + ref.nanosec * 1e-9
        local_sec = self.get_clock().now().nanoseconds * 1e-9
        skew = ref_sec - local_sec

        if abs(skew) <= SKEW_THRESHOLD_SECONDS:
            response.success = True
            response.message = f'Clock already in sync (skew={skew:.1f}s). No change made.'
            self.get_logger().info(response.message)
            return response

        formatted = datetime.fromtimestamp(ref_sec, tz=timezone.utc).strftime('%Y-%m-%d %H:%M:%S')
        result = subprocess.run(
            ['sudo', '-n', 'date', '-u', '-s', formatted],
            capture_output=True, text=True)

        if result.returncode != 0:
            response.success = False
            response.message = f'Failed to set clock: {result.stderr.strip()}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f'Clock corrected (skew was {skew:.1f}s).'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ClockSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
