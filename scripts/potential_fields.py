#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PotentialFieldAvoidance:
    def __init__(self):
        rospy.init_node('potential_field_node')
        rospy.set_param('/use_sim_time', True)

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.d0 = 0.4        # Obstacle detection range
        self.k_rep = 1.0     # Repulsive gain
        self.max_speed = 0.25
        self.angular_gain = 2.5
        self.forward_bias = np.array([0.5, 0])  # Small push forward

    def scan_callback(self, scan):
        force = self.compute_repulsive_force(scan) + self.forward_bias
        cmd = self.force_to_cmd(force)
        self.cmd_pub.publish(cmd)

    def compute_repulsive_force(self, scan):
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges)
        mask = np.isfinite(ranges) & (ranges > 0.05)

        xs = ranges[mask] * np.cos(angles[mask])
        ys = ranges[mask] * np.sin(angles[mask])
        points = np.stack((xs, ys), axis=-1)

        force = np.zeros(2)
        for p in points:
            d = np.linalg.norm(p)
            if d > self.d0:
                continue
            direction = -p / d
            magnitude = self.k_rep * (1.0 / d**2 - 1.0 / self.d0**2)
            force += max(magnitude, 0) * direction

        return force

    def force_to_cmd(self, force_vector):
        angle = np.arctan2(force_vector[1], force_vector[0])
        speed = min(np.linalg.norm(force_vector), self.max_speed)

        cmd = Twist()
        cmd.linear.x = speed * np.cos(angle)
        cmd.angular.z = np.clip(self.angular_gain * angle, -1.5, 1.5)  # capped
        return cmd

if __name__ == '__main__':
    PotentialFieldAvoidance()
    rospy.spin()
