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
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		timer_period = 1.0  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		# create rulebase
		self.ruleRF = ["close", "close", "close", "med",   "med", "med", "far",   "far", "far"]
		self.ruleRB = ["close", "med",   "far",   "close", "med", "far", "close", "med", "far"]
		self.Build_Fuzzy()



		

	def Defuzz(self, Fire_Strength):
		speed = [0.135, 0.3, 0.465]  # slow, medium, fast
		steer = [-0.5, 0.0, 0.5]     # right to left

		#for i in range(len(FS)):
		for i in range(len(RBFSreturn)):
			for j in range(len(RFFSreturn)):
				op1 = min(RBFSreturn[i])
				op2 = min(RFFSreturn[j])

				
				if self.ruleRF == "close" and self.ruleRB == "close":
					# slow steady
					outputspeed = (op1*speed[0])/sum(op1+op2)
					outputsteer = (op2*steer[1])/sum(op1+op2)
				
				elif self.ruleRF == "close" and self.ruleRB == "med":
					# slow right
					outputspeed = (op1*speed[0])/(op1+op2)
					outputsteer = (op2*steer[0])/(op1+op2)
				
				elif self.ruleRF == "close" and self.ruleRB == "far":
					# med left
					outputspeed = (op1*speed[1])/sum(op1+op2)
					outputsteer = (op2*steer[2])/sum(op1+op2)
					
				elif self.ruleRF == "med" and self.ruleRB == "close":
					# slow left
					outputspeed = (op1*speed[0])/sum(op1+op2)
					outputsteer = (op2*steer[2])/sum(op1+op2)
				
				elif self.ruleRF == "med" and self.ruleRB == "med":
					# med right
					outputspeed = (op1*speed[1])/sum(op1+op2)
					outputsteer = (op2*steer[0])/sum(op1+op2)
				
				elif self.ruleRF == "med" and self.ruleRB == "far":
					# fast steady
					outputspeed = (op1*speed[2])/sum(op1+op2)
					outputsteer = (op2*steer[1])/sum(op1+op2)
				
				elif self.ruleRF == "far" and self.ruleRB == "close":
					# med right
					outputspeed = (op1*speed[1])/sum(op1+op2)
					outputsteer = (op2*steer[0])/sum(op1+op2)				
					
				elif self.ruleRF == "far" and self.ruleRB == "med":
					# fast left
					outputspeed = (op1*speed[2])/sum(op1+op2)
					outputsteer = (op2*steer[2])/sum(op1+op2)				
					
				elif self.ruleRF == "far" and self.ruleRB == "far":
					# fast left
					outputspeed = (op1*speed[2])/sum(op1+op2)
					outputsteer = (op2*steer[2])/sum(op1+op2)
		
		print("speed: ", outputspeed)	
		print("steer: ", outputsteer)
				
		self.cmd.linear.x = outputspeed
		self.cmd.angular.z = outputsteer



	def Fire_Strength(self, inputval, fuzzysets):
		FS = []			# list to store firing strengths
		membershipfcn = []
		for membershipfcn in fuzzysets:

			if (membershipfcn[1] < inputval) and (membershipfcn[2] > inputval):
				# rising edge
				FS.append((inputval - membershipfcn[1])/(membershipfcn[2]-membershipfcn[1]))
			
			elif (membershipfcn[3] < inputval) and (membershipfcn[4] > inputval):
				# falling edge
				FS.append((membershipfcn[4] - inputval)/(membershipfcn[4] - membershipfcn[3]))
			
			elif (membershipfcn[2] <= inputval) and (membershipfcn[3] >= inputval):
				# flat
				FS.append(1.0)
			else:
				FS.append(0.0)
			print("Fire Strength: ", FS)
		return FS



	def Fuzzy_Set(self, title, a, b, c, d):
		FZ_ans = []            # creating a list to store fuzzy set as tuples
		for i in range(len(title)):
			FZ_ans.append( (title[i], a[i], b[i], c[i], d[i]))
		return FZ_ans

	def Build_Fuzzy(self):
		## make our fuzzy sets and rule base
		title = ['close', 'medium', 'far']
		a = [0.00,  0.25,  1.0]
		b = [0.00,  0.50,  1.25]
		c = [0.25,  1.0,  1.5]
		d = [0.5,  1.25,   1.5]
		self.RF = self.Fuzzy_Set(title, a, b, c, d)
		self.RB = self.Fuzzy_Set(title, a, b, c, d)
		
	def Fuzzy_Run(self):
		#print(self.laser_ranges[980:1080])
		#print(self.laser_ranges[1081:1180])
		RBmin0 = float(min(self.laser_ranges[245:270]))    # grabs min value for right back
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
		self.Fuzzy_Run()
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
