#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import time

# steering positive left, negative right
# ros uses 4x degrees than gazebo
# multiply values by 4 when outside the simulator
# front  0     or  0    in simulator
# left   360   or  90   in simulator
# right  1080  or  270  in simulator
# back   720   or  180  in simulator


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('forward_demo')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        time.sleep(10)
        self.laser_ranges = []
        for i in range(360):
            self.laser_ranges.append(0.0)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        BuildFuzzy()

    def BuildFuzzy(self):
        ## make our fuzzy sets and rule base
        title = ['close', 'med', 'far']
        a = [0.00, 0.25, 0.50]
        b = [0.00, 0.50, 0.75]
        c = [0.25, 0.50, 0.75]
        d = [0.50, 0.75, 1.00]
        self.LFS = FuzzySet(title, a, b, c, d)
        self.MFS = FuzzySet(title, a, b, c, d)
        self.RFS = FuzzySet(title, a, b, c, d)
        # create rulebase
        self.ruleMFS = ["close", "close", "close", "med", "med", "med", "far", "far", "far", "close", "close",
                            "close", "med", "med", "med", "far", "far", "far", "close", "close", "close", "med", "med",
                            "med", "far", "far", "far"]
        self.ruleLFS = ["close", "close", "close", "close", "close", "close", "close", "close", "close", "med",
                            "med", "med", "med", "med", "med", "med", "med", "med", "far", "far", "far", "far", "far",
                            "far", "far", "far", "far"]
        self.ruleRFS = ["close", "med", "far", "close", "med", "far", "close", "med", "far", "close", "med", "far",
                            "close", "med", "far", "close", "med", "far", "close", "med", "far", "close", "med", "far",
                            "close", "med", "far", ]

         # create rulebase
        self.ruleRF = ["close", "close", "close", "med", "med", "med", "far", "far", "far"]
        self.ruleRB = ["close", "med", "far", "close", "med", "far", "close", "med", "far"]
        self.Build_Fuzzy()

    def Defuzz(self, FireStrength):
        speed = [0.135, 0.3, 0.465]  # slow, medium, fast
        steer = [-0.5, 0.0, 0.5]     # right to left
      
		for i in range(len(LFSFSreturn)):
			for j in range(len(MFFSreturn)):
                for k in range(len(RFSFSreturn)):
                    FS_1 = min(LFSFSreturn[i])
                    FS_2 = min(MFSFSreturn[j])
                    FS_3 = min(RFSFSreturn[k])

                    if self.ruleLFS == "close" and self.ruleMFS == "close" and self.ruleRFS == "close":
                        # row 1 slow right
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "close" and self.ruleRFS == "med":
                        # row 2 slow right
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "close" and self.ruleRFS == "far":
                        # row 3 med right
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "med" and self.ruleRFS == "close":
                        # row 4 slow left
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "med" and self.ruleRFS == "med":
                        # row 5 slow right
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "med" and self.ruleRFS == "far":
                        # row 6 slow right
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "far" and self.ruleRFS == "close":
                        # row 7 med steady
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[1])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "far" and self.ruleRFS == "med":
                        # row 8 med steady
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[1])/sum(FireStrength[i])
                    elif self.ruleLFS == "close" and self.ruleMFS == "far" and self.ruleRFS == "far":
                        # row 9 med left
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "close" and self.ruleRFS == "close":
                        # row 10 slow left
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "close" and self.ruleRFS == "med":
                        # row 11 slow left
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "close" and self.ruleRFS == "far":
                        # row 12 slow right
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "med" and self.ruleRFS == "close":
                        # row 13 slow left
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "med" and self.ruleRFS == "med":
                        # row 14 slow right
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "med" and self.ruleRFS == "far":
                        # row 15 med right
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "far" and self.ruleRFS == "close":
                        # row 16 med right
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "far" and self.ruleRFS == "med":
                        # row 17 fast steady
                        outputspeed = (op1*speed[2])/sum(FireStrength[i])
                        outputsteer = (op2*steer[1])/sum(FireStrength[i])
                    elif self.ruleLFS == "med" and self.ruleMFS == "far" and self.ruleRFS == "far":
                        # row 18 med right
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "close" and self.ruleRFS == "close":
                        # row 19 slow left
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "close" and self.ruleRFS == "med":
                        # row 20 slow left
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "close" and self.ruleRFS == "far":
                        # row 21 slow right
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "med" and self.ruleRFS == "close":
                        # row 22 slow left
                        outputspeed = (op1*speed[0])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "med" and self.ruleRFS == "med":
                        # row 23 med left
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "med" and self.ruleRFS == "far":
                        # row 24 med right
                        outputspeed = (op1*speed[1])/sum(FireStrength[i])
                        outputsteer = (op2*steer[0])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "far" and self.ruleRFS == "close":
                        # row 25 fast left
                        outputspeed = (op1*speed[2])/sum(FireStrength[i])
                        outputsteer = (op2*steer[2])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "far" and self.ruleRFS == "med":
                        # row 26 fast steady
                        outputspeed = (op1*speed[2])/sum(FireStrength[i])
                        outputsteer = (op2*steer[1])/sum(FireStrength[i])
                    elif self.ruleLFS == "far" and self.ruleMFS == "far" and self.ruleRFS == "far":
                        # row 27 fast steady
                        outputspeed = (op1*speed[2])/sum(FireStrength[i])
                        outputsteer = (op2*steer[1])/sum(FireStrength[i])

		print("speed: ", outputspeed)	
		print("steer: ", outputsteer)
		self.cmd.linear.x = outputspeed
		self.cmd.angular.z = outputsteer



    def FireStrength(self, inputval, fuzzysets):
        FS = []  # list to store firing strengths
        membershipfcn = []
        for membershipfcn in fuzzysets:

            if (membershipfcn[1] < inputval) and (membershipfcn[2] > inputval):
                # rising edge
                FS.append((inputval - membershipfcn[1]) / (membershipfcn[2] - membershipfcn[1]))

            elif (membershipfcn[3] < inputval) and (membershipfcn[4] > inputval):
                # falling edge
                FS.append((membershipfcn[4] - inputval) / (membershipfcn[4] - membershipfcn[3]))

            elif (membershipfcn[2] <= inputval) and (membershipfcn[3] >= inputval):
                # flat
                FS.append(1.0)
            else:
                FS.append(0.0)
            print("Fire Strength: ", FS)
        return FS



    def FuzzySet(self, title, a, b, c, d):
        FZ_ans = []            # creating a list to store fuzzy set as tuples
        for i in range(title):
            FZ_ans.append(tuple(title[i], [a[i], b[i], c[i], d[i]])
        return FZ_ans

    def FuzzyRun(self):

        #print(self.laser_ranges[980:1080])
        #print(self.laser_ranges[1081:1180])
        LFSmin = float(min(self.laser_ranges[100:180]))  # grabs min value for left front sensor
        MFSmin_0 = [min(self.laser_ranges[0:40], min(self.laser_ranges[1400:1439]))] # grabs min value for middle front sensor
        MFSmin = float(min(MFSmin_0))
        RFSmin = float(min(self.laser_ranges[1260:1340]))  # grabs min value for right front sensor

        LFSFSreturn = FireStrength(LFSmin, self.LFS)
        MFSFSreturn = FireStrength(MFSmin, self.MFS)
        RFSFSreturn = FireStrength(RFSmin, self.RFS)

        # print(self.laser_ranges[980:1080])
        # print(self.laser_ranges[1081:1180])
        RBmin0 = float(min(self.laser_ranges[245:270]))  # grabs min value for right back
        RBmin = min(RBmin0, 1.0)
        RFmin0 = float(min(self.laser_ranges[271:295]))  # grabs min value for right front
        RFmin = min(RFmin0, 1.0)
        RBFSreturn = self.Fire_Strength(RBmin, self.RB)
        RFFSreturn = self.Fire_Strength(RFmin, self.RF)
        print("RBmin: ", RBmin)
        print("RFmin: ", RFmin)
        print("RBFSreturn: ", RBFSreturn)
        print("RBFSreturn: ", RBFSreturn)

    def timer_callback(self):
        self.FuzzyRun()
        self.publisher_.publish(self.cmd)
        string = 'Publishing:' + str(self.cmd.linear.x)
        self.get_logger().info(string)

    def laser_callback(self, msg):
        self.get_logger().info(str(msg.ranges[1080]))
        self.laser_ranges = msg.ranges


def main(args=None):
    # time.sleep(4500)
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)


if __name__ == '__main__':
    main()
