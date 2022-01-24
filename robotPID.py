#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import time

# steering positive left, negative right
# multiply values by 4 when outside the simulator
# front  0     or  0    in simulator
# left   360   or  90   in simulator
# right  1080  or  270  in simulator
# back   720   or  180  in simulator


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('forward_demo')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        time.sleep(15)
        self.laser_ranges = []

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
		#e_i = []

    def PID(self):
        Kp = 0.35  		# proportional #0.5 0.01 0.1
        Ki = 0.1  		# integral
        Kd = 0.45  		# derivative
        desired = 0.5   # desired distance
        e_i = 0.0  		# initial error
        e_prev = 0.0    # previous error
        e_d = 0.0       # desired error
        minx = min(self.laser_ranges[1000:1160])  # grabs min value within this range

        print(self.laser_ranges[1080])  # print the range of the right sensor from wall

        self.cmd.linear.x = 0.3
        e = desired - float(minx)  # error
        e_i += e  # initial error
        e_d = e - e_prev  # desired error
        e_prev = e  # previous error
        
        #for i in range(10):
        output = (Kp * e) + (Ki * e_i) + (Kd * e_d)  # output of PID control
        #e_i[i].append(e)
        self.cmd.angular.z = output  # we make that the angular control
        if -9 > e_i > 9:
        	e_i = 0.0
        if -9 > e > 9:
            e = 0.0


    def timer_callback(self):
        self.PID()
        self.publisher_.publish(self.cmd)
        string = 'Publishing:' + str(self.cmd.linear.x)
        self.get_logger().info(string)

    def laser_callback(self, msg):
        self.get_logger().info(str(msg.ranges[0]))
        self.laser_ranges = msg.ranges


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)


if __name__ == '__main__':
    main()
