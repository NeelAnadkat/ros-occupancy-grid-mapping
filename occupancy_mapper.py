#!/usr/bin/env python3
"""
Simple occupancy-grid mapping using:
- /scan (LaserScan)
- /odom (Odometry)

Publishes:
- /map (nav_msgs/OccupancyGrid)
- /map_metadata (nav_msgs/MapMetaData)
"""

import rospy
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class Map(object):
    """
    Occupancy grid stored as a 2D numpy array:
    0.0 = free/unknown-ish
    1.0 = occupied
    """

    def __init__(self, origin_x=-5.0, origin_y=-5.0, resolution=0.05, width=200, height=200):
        # 200 * 0.05 = 10m map size, centered-ish around (0,0) by origin -5,-5
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=np.float32)

    def world_to_grid(self, x, y):
        """Convert world coords (meters) to grid indices (row, col)."""
        gx = (x - self.origin_x) / self.resolution
        gy = (y - self.origin_y) / self.resolution
        col = int(round(gx))
        row = int(round(gy))
        return row, col

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def set_cell(self, x, y, val):
        """Set cell that contains world point (x,y) to val if in map bounds."""
        row, col = self.world_to_grid(x, y)
        if self.in_bounds(row, col):
            # Keep the max (so once occupied, it stays occupied)
            self.grid[row, col] = max(self.grid[row, col], float(val))

    def to_message(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        grid_msg.info = MapMetaData()
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0.0),
                                    Quaternion(0, 0, 0, 1))

        # Convert [0..1] to [0..100] int8
        flat = (self.grid.reshape(-1) * 100.0)
        flat = np.clip(flat, 0, 100).astype(np.int8)
        grid_msg.data = list(flat)
        return grid_msg


class Mapper(object):
    def __init__(self):
        rospy.init_node("mapper")

        # Map params (you can tweak these for better quality)
        self._map = Map(origin_x=-5.0, origin_y=-5.0, resolution=0.05, width=200, height=200)

        # Robot pose updated from /odom
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False

        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        self._map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self._map_meta_pub = rospy.Publisher("/map_metadata", MapMetaData, queue_size=1, latch=True)

        rospy.loginfo("Mapper node started. Subscribed to /odom and /scan.")
        rospy.spin()

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_yaw = yaw

        self.have_odom = True

    def scan_callback(self, scan):
        if not self.have_odom:
            # Wait until odom arrives so we can place points globally
            return

        # Downsample scan for speed (use every Nth ray)
        step = 5  # smaller = denser map but slower

        angle = scan.angle_min
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]

            # Only use valid ranges
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment * step
                continue
            if r <= scan.range_min or r >= scan.range_max:
                angle += scan.angle_increment * step
                continue

            # Beam angle in robot frame
            beam_angle = scan.angle_min + i * scan.angle_increment

            # Local (robot frame) obstacle point
            x_s = r * math.cos(beam_angle)
            y_s = r * math.sin(beam_angle)

            # Rotate into global frame using robot yaw, then translate by robot position
            c = math.cos(self.robot_yaw)
            s = math.sin(self.robot_yaw)

            x_g = self.robot_x + (c * x_s - s * y_s)
            y_g = self.robot_y + (s * x_s + c * y_s)

            # Mark occupied
            self._map.set_cell(x_g, y_g, 1.0)

        self.publish_map()

    def publish_map(self):
        msg = self._map.to_message()
        self._map_meta_pub.publish(msg.info)
        self._map_pub.publish(msg)


if __name__ == "__main__":
    try:
        Mapper()
    except rospy.ROSInterruptException:
        pass
