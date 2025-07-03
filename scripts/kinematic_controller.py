#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class KinematicController:
    def __init__(self):
        rospy.init_node('kinematic_controller')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Example hardcoded path (replace with your planner path)
        self.waypoints = [(0.5, 0.0), (1.0, 0.0), (1.5, 0.5), (2.0, 2.0)]
        self.waypoint_index = 0

        # Controller gains
        self.k_rho = 0.5
        self.k_alpha = 1.5
        self.k_beta = -0.5

        self.x = 0
        self.y = 0
        self.theta = 0

        self.control_loop()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        orientation = pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation.x, orientation.y,
                                                  orientation.z, orientation.w])

    def control_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.waypoint_index < len(self.waypoints):
            xg, yg = self.waypoints[self.waypoint_index]

            dx = xg - self.x
            dy = yg - self.y
            rho = math.hypot(dx, dy)
            alpha = math.atan2(dy, dx) - self.theta
            beta = -self.theta - alpha

            v = self.k_rho * rho
            w = self.k_alpha * alpha + self.k_beta * beta

            cmd = Twist()
            cmd.linear.x = min(v, 0.3)  # clip max speed
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            if rho < 0.1:  # Switch waypoint if close
                self.waypoint_index += 1

            rate.sleep()

        rospy.loginfo("Reached all waypoints")
        self.cmd_pub.publish(Twist())  # Stop the robot

if __name__ == '__main__':
    try:
        KinematicController()
    except rospy.ROSInterruptException:
        pass

