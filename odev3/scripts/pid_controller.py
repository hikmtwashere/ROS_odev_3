#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class PIDObstacleAvoidance:
    def __init__(self):
        rospy.init_node("pid_obstacle_avoidance_node", anonymous=True)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.front_distance = 9999.0
        self.left_distance = 9999.0
        self.right_distance = 9999.0

        # PID hedefi: robot engelden yaklaşık 1 metre uzakta kalsın
        self.target_distance = 1.0

        # PID katsayıları
        self.Kp = 0.6
        self.Ki = 0.0
        self.Kd = 0.15

        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()

        # Güvenlik eşikleri
        self.danger_distance = 0.55
        self.safe_distance = 0.80

        # Hız sınırları
        self.max_linear_speed = 0.22
        self.min_linear_speed = -0.08
        self.rotate_speed = 0.65

        self.rate = rospy.Rate(10)

    def scan_callback(self, scan_data):
        front_ranges = []
        left_ranges = []
        right_ranges = []

        for i, r in enumerate(scan_data.ranges):
            if math.isnan(r) or math.isinf(r):
                continue

            angle = scan_data.angle_min + i * scan_data.angle_increment

            if angle > math.pi:
                angle = angle - 2 * math.pi

            angle_deg = math.degrees(angle)

            # Ön bölge: -20 ile +20 derece
            if -20 <= angle_deg <= 20:
                front_ranges.append(r)

            # Sol bölge: +30 ile +90 derece
            elif 30 <= angle_deg <= 90:
                left_ranges.append(r)

            # Sağ bölge: -90 ile -30 derece
            elif -90 <= angle_deg <= -30:
                right_ranges.append(r)

        self.front_distance = min(front_ranges) if len(front_ranges) > 0 else 9999.0
        self.left_distance = min(left_ranges) if len(left_ranges) > 0 else 9999.0
        self.right_distance = min(right_ranges) if len(right_ranges) > 0 else 9999.0

    def limit(self, value, min_value, max_value):
        if value > max_value:
            return max_value
        elif value < min_value:
            return min_value
        return value

    def compute_pid(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()

        if dt <= 0:
            dt = 0.1

        error = self.front_distance - self.target_distance

        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = self.limit(output, self.min_linear_speed, self.max_linear_speed)

        self.previous_error = error
        self.last_time = now

        return output, error

    def run(self):
        while not rospy.is_shutdown():
            cmd = Twist()

            # 1) Ön taraf tehlikeli derecede yakınsa: dur ve boş tarafa dön
            if self.front_distance < self.danger_distance:
                cmd.linear.x = 0.0

                if self.left_distance > self.right_distance:
                    cmd.angular.z = self.rotate_speed
                    direction = "LEFT"
                else:
                    cmd.angular.z = -self.rotate_speed
                    direction = "RIGHT"

                rospy.logwarn(
                    "DANGER | Front: %.2f | Left: %.2f | Right: %.2f | Rotate: %s",
                    self.front_distance,
                    self.left_distance,
                    self.right_distance,
                    direction
                )

            # 2) Ön taraf biraz yakınsa: yavaşla ve yine boş tarafa dön
            elif self.front_distance < self.safe_distance:
                cmd.linear.x = 0.03

                if self.left_distance > self.right_distance:
                    cmd.angular.z = 0.45
                    direction = "LEFT"
                else:
                    cmd.angular.z = -0.45
                    direction = "RIGHT"

                rospy.logwarn(
                    "AVOID | Front: %.2f | Left: %.2f | Right: %.2f | Turn: %s",
                    self.front_distance,
                    self.left_distance,
                    self.right_distance,
                    direction
                )

            # 3) Ön taraf açıksa: PID ile ileri hız kontrolü
            else:
                pid_output, error = self.compute_pid()

                cmd.linear.x = pid_output
                cmd.angular.z = 0.0

                rospy.loginfo(
                    "PID | Front: %.2f | Error: %.2f | Linear: %.2f",
                    self.front_distance,
                    error,
                    pid_output
                )

            self.pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    node = PIDObstacleAvoidance()
    node.run()
