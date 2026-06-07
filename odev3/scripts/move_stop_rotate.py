#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class MoveStopRotate:
    def __init__(self):
        rospy.init_node("move_stop_rotate_node", anonymous=True)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.front_distance = 9999.0

        self.safe_distance = 0.7
        self.forward_speed = 0.25
        self.rotate_speed = 0.6

        self.rate = rospy.Rate(10)

    def scan_callback(self, scan_data):
        front_ranges = []

        for i, r in enumerate(scan_data.ranges):
            if math.isnan(r) or math.isinf(r):
                continue

            angle = scan_data.angle_min + i * scan_data.angle_increment

            # Açıyı -pi ile +pi arasına sıkıştır
            if angle > math.pi:
                angle = angle - 2 * math.pi

            # Sadece robotun önündeki yaklaşık +-20 dereceyi al
            if abs(angle) < math.radians(20):
                front_ranges.append(r)

        if len(front_ranges) > 0:
            self.front_distance = min(front_ranges)
        else:
            self.front_distance = 9999.0

    def run(self):
        while not rospy.is_shutdown():
            cmd = Twist()

            if self.front_distance > self.safe_distance:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                rospy.loginfo("MOVE | Front distance: %.2f m", self.front_distance)

            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.rotate_speed
                rospy.logwarn("STOP-ROTATE | Front obstacle: %.2f m", self.front_distance)

            self.pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    node = MoveStopRotate()
    node.run()
