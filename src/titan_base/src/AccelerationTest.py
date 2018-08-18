#!/usr/bin/env python

import rospy
import rosbag
from titan_base.msg import *
from std_msgs.msg import Float32
import numpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import sys

class AccelerationTest():
	def __init__(self):
		self.bag = rosbag.Bag('output.bag','w')
		self.pidf_pub = rospy.Publisher('/set_pidf_param', PIDF, queue_size=10)
		self.vel_pub = rospy.Publisher('/motor_velocity', MotorVelocity, queue_size=10)

		self.left_sp = 0.0
		self.right_sp = 0.0


	def initPlot(self):
		style.use('fivethirtyeight')
		plt.close('all')
		self.fig, self.axarr = plt.subplots(5,sharex=True)


	def cbMotorStatus(self,status):
		#timestamp = status.header.stamp.secs + (status.header.stamp.nsecs / 1.0e9)
		bag.write('motor_status',status)


	def publishPIDF(self):
		pidf = PIDF()
		pidf.P_Gain = 0.0
		pidf.I_Gain = 0.0
		pidf.D_Gain = 0.0
		pidf.F_Gain = 1.0
		pidf.FeedbackCoeff = 0.0
		pidf.RampRate = 100
		r = rospy.Rate(5)
		r.sleep()
		self.bag.write('set_pidf_param', pidf)
		self.pidf_pub.publish(pidf)

	def initROS(self):
		rospy.init_node('Accel', anonymous=True)
		rospy.Subscriber("/motor_status", Status, self.cbMotorStatus)
		self.publishPIDF()

	def sendMotorCommand(self,left_sp,right_sp):
		cur_time = rospy.get_rostime()
		motor_cmd = MotorVelocity()
		motor_cmd.header.stamp.secs = cur_time.secs
		motor_cmd.header.stamp.nsecs = cur_time.nsecs
		motor_cmd.left_angular_vel = left_sp;
		motor_cmd.right_angular_vel = right_sp;
		self.bag.write('motor_velocity',motor_cmd)
	
		self.vel_pub.publish(motor_cmd);
		
		
		

	

def Main():
	a = AccelerationTest()
	a.initROS()

	


	start_time = rospy.get_rostime()
	prev_time = start_time
	cur_time = start_time
	

	d = rospy.Duration.from_sec(1.5)
	end_duration = rospy.Duration.from_sec(9.0)
	r = rospy.Rate(20)
	i = 0
	while not rospy.is_shutdown():
		cur_time = rospy.get_rostime()
		if (cur_time - prev_time <= d):
			
			if (i % 2 == 0):
				left_sp = 1.0 * 6.248
				right_sp = 1.0 * 6.248
			else:
				left_sp = 0.0
				right_sp = 0.0
			
			
		else:
			prev_time = rospy.get_rostime()
			i = i+1
		a.sendMotorCommand(left_sp,right_sp)
		if (cur_time - start_time >= end_duration):
			break
		r.sleep()

	self.bag.close()
	
	self.axarr[0].clear()
	self.axarr[1].clear()
	self.axarr[2].clear()
	self.axarr[3].clear()

	self.axarr[0].plot(self.time_pts,self.pos_pts)
	self.axarr[1].plot(self.time_pts,self.error_pts)
	self.axarr[2].plot(self.time_pts,self.throttle_pts)		
	self.axarr[3].plot(self.time_pts,self.vel_pts)
	self.axarr[4].plot(self.time_pts,self.sp_pts)
	#plt.show()
	#print self.pos_pts
	return

	#print "test"

if __name__ == '__main__':
	Main()

