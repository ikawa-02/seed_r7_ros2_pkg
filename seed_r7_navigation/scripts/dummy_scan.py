#!/usr/bin/env python3
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros


class DummyScan(Node):
    def __init__(self):
        super().__init__('dummy_scan')

        self.pub_ = self.create_publisher(LaserScan, 'scan', 1)

        self.tf_buffer_   = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.pi2 = math.pi * 2
        self.laser_height = 0.2344
        self.mapgrid_list = []

        # emulate URG
        self.scan_ = LaserScan()
        self.scan_.header.frame_id = 'wheels_base_laser_link'
        self.scan_.angle_min       = -1.57079637051
        self.scan_.angle_max       =  1.57079637051
        self.scan_.angle_increment =  0.00436332309619
        self.scan_.time_increment  =  1.73611151695e-05
        self.scan_.scan_time       =  0.0250000003725
        self.scan_.range_min       =  0.019999999553
        self.scan_.range_max       = 30.0

        self.ranges_size_ = int(
            1.0 + (self.scan_.angle_max - self.scan_.angle_min)
            / self.scan_.angle_increment)

        # Subscribe to map once
        self.sub_map_ = self.create_subscription(
            OccupancyGrid, 'map', self._map_callback, 1)

        # Publish at 40 Hz
        self.timer_ = self.create_timer(1.0 / 40.0, self._publish)

    def _map_callback(self, msg):
        info = msg.info
        cr = math.cos(math.acos(info.origin.orientation.w) * 2.0)
        sr = math.sqrt(max(0.0, 1.0 - cr * cr))
        self.mapgrid_list = []
        for y in range(info.height):
            yidx = y * info.width
            my = y * info.resolution + info.origin.position.y
            for x in range(info.width):
                if msg.data[x + yidx] >= 90:
                    mx = x * info.resolution + info.origin.position.x
                    self.mapgrid_list.append([
                        mx * cr - my * sr,
                        mx * sr + my * cr,
                        self.laser_height,
                    ])
        # Unsubscribe after first map
        self.destroy_subscription(self.sub_map_)

    def _make_map_range(self):
        laser_pos   = (0.0, 0.0, 0.0)
        laser_theta = 0.0
        try:
            t = self.tf_buffer_.lookup_transform(
                'map', 'wheels_base_laser_link',
                rclpy.time.Time())
            laser_pos = (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
            t_base = self.tf_buffer_.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time())
            laser_theta = math.atan2(
                laser_pos[1] - t_base.transform.translation.y,
                laser_pos[0] - t_base.transform.translation.x,
            )
        except Exception:
            pass

        self.scan_.ranges = [0.0] * self.ranges_size_

        for mp in self.mapgrid_list:
            lx = mp[0] - laser_pos[0]
            ly = mp[1] - laser_pos[1]
            lrange = math.sqrt(lx * lx + ly * ly)
            ltheta = math.atan2(ly, lx) - laser_theta
            if ltheta > math.pi:
                ltheta -= self.pi2
            elif ltheta < -math.pi:
                ltheta += self.pi2

            if self.scan_.angle_min <= ltheta <= self.scan_.angle_max:
                li = int((ltheta - self.scan_.angle_min)
                         / self.scan_.angle_increment)
                if li < self.ranges_size_:
                    if self.scan_.ranges[li] < 1e-5 or self.scan_.ranges[li] > lrange:
                        self.scan_.ranges[li] = lrange

    def _publish(self):
        self._make_map_range()
        self.scan_.header.stamp = self.get_clock().now().to_msg()
        self.pub_.publish(self.scan_)


def main():
    rclpy.init()
    node = DummyScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
