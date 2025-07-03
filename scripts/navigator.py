#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class Navigator:
    def __init__(self):
        rospy.init_node('navigator')
        rospy.set_param('/use_sim_time', True)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/planned_path', Path, self.path_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.pose = None
        self.scan = None
        self.path = []
        self.index = 0

        self.k_rho = 0.6
        self.k_alpha = 1.5
        self.k_beta = -0.5
        self.epsilon = 0.2

        self.k_rep = 1.2
        self.d0 = 0.4
        self.angular_gain = 3.0

        self.rate = rospy.Rate(10)
        self.control_loop()

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (p.x, p.y, theta)

    def scan_callback(self, msg):
        self.scan = msg

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.index = 0
        rospy.loginfo("Loaded path with %d waypoints", len(self.path))

    def compute_tracking_cmd(self):
        if self.pose is None or self.index >= len(self.path):
            return Twist()

        x, y, theta = self.pose
        xg, yg = self.path[self.index]
        dx = xg - x
        dy = yg - y
        rho = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx) - theta
        beta = -theta - alpha

        cmd = Twist()
        cmd.linear.x = min(0.3, self.k_rho * rho)
        cmd.angular.z = self.k_alpha * alpha + self.k_beta * beta

        if rho < self.epsilon:
            self.index += 1
            rospy.loginfo("Switched to waypoint %d/%d", self.index, len(self.path))
        return cmd

    def compute_repulsive_force(self):
        if not self.scan:
            return np.zeros(2)

        angles = np.linspace(self.scan.angle_min, self.scan.angle_max, len(self.scan.ranges))
        ranges = np.array(self.scan.ranges)
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
        speed = min(np.linalg.norm(force_vector), 0.25)
        cmd = Twist()
        cmd.linear.x = speed * np.cos(angle)
        cmd.angular.z = np.clip(self.angular_gain * angle, -2.0, 2.0)
        return cmd

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.pose is None or self.scan is None:
                self.rate.sleep()
                continue

            if self.index >= len(self.path):
                self.cmd_pub.publish(Twist())
                rospy.loginfo("Reached final waypoint.")
                self.rate.sleep()
                continue

            if min(self.scan.ranges) < 0.2:
                rospy.loginfo("Avoiding obstacle")
                force = self.compute_repulsive_force()
                cmd = self.force_to_cmd(force)
            else:
                cmd = self.compute_tracking_cmd()

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        Navigator()
    except rospy.ROSInterruptException:
        pass
