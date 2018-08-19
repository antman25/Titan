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
	def __init__(self,bag_path):
		self.bag = rosbag.Bag(bag_path + '/raw.bag','w')
		self.pidf_pub = rospy.Publisher('/set_pidf_param', PIDF, queue_size=10)
		self.vel_pub = rospy.Publisher('/motor_velocity', MotorVelocity, queue_size=10)
		self.left_motor_state = []
		self.right_motor_state = []
		self.left_sp = 0.0
		self.right_sp = 0.0
		self.initPlot()
		self.initROS()


	def initPlot(self):
		style.use('fivethirtyeight')
		plt.close('all')
		self.fig, self.axarr = plt.subplots(5,sharex=True)


	def cbMotorStatus(self,status):
		#timestamp = status.header.stamp.secs + (status.header.stamp.nsecs / 1.0e9)
		#print status
		#print self.bag
		self.bag.write('/motor_status',status,rospy.get_rostime())
		s = MotorState(status.header.stamp, status.SensorPosition, status.SensorVelocity, status.AppliedThrottle,status.CloseLoopErr)
		if (status.DeviceId == 1):
			self.left_motor_state.append(s)
			#print "LEFT: " + str(s)
		if (status.DeviceId == 4):	
			self.right_motor_state.append(s)
			#print "RIGHT: " + str(s)


	def publishPIDF(self):
		pidf = PIDF()
		pidf.P_Gain = 4.5
		pidf.I_Gain = 0.0
		pidf.D_Gain = 0.05
		pidf.F_Gain = 2.5
		pidf.FeedbackCoeff = 1000.0 / (2048.0) 
		pidf.RampRate = 50
		r = rospy.Rate(5)
		r.sleep()
		self.bag.write('set_pidf_param', pidf,rospy.get_rostime())
		self.pidf_pub.publish(pidf)

	def initROS(self):
		rospy.init_node('Accel', anonymous=True)
		self.status_sub = rospy.Subscriber("/motor_status", Status, self.cbMotorStatus)
		self.publishPIDF()

	def close(self):
		self.status_sub.unregister()
		self.bag.close()
		self.bag.reindex()
		self.axarr[0].clear()
		self.axarr[1].clear()
		self.axarr[2].clear()
		self.axarr[3].clear()
		self.axarr[4].clear()

		#self.axarr[0].plot(self.time_pts,self.pos_pts)
		#self.axarr[1].plot(self.time_pts,self.error_pts)
		#self.axarr[2].plot(self.time_pts,self.throttle_pts)		
		#self.axarr[3].plot(self.time_pts,self.vel_pts)
		#self.axarr[4].plot(self.time_pts,self.sp_pts)
		#plt.show()

	def sendMotorCommand(self,left_sp,right_sp):
		cur_time = rospy.get_rostime()
		motor_cmd = MotorVelocity()
		motor_cmd.header.stamp.secs = cur_time.secs
		motor_cmd.header.stamp.nsecs = cur_time.nsecs
		motor_cmd.left_angular_vel = left_sp;
		motor_cmd.right_angular_vel = right_sp;
		motor_cmd.header.frame_id = ''
		#self.bag.write('/motor_velocity',motor_cmd,rospy.get_rostime())
	
		self.vel_pub.publish(motor_cmd);

	def analyze(self):
		cur_state = None
		prev_state = None
		print "Calculating Values"
		for i in range(len(self.left_motor_state)):
			cur_state = self.left_motor_state[i]
			print str(cur_state)
			if (prev_state == None):
				print "Init"
				prev_state = self.left_motor_state[i]
				continue	
			d = MotorStateDelta(cur_state,prev_state)
			prev_state = cur_state
			print d
		

class MotorStateDelta():
	#def __init__(self,time_delta,enc_delta,vel_delta):
	#	self.time_delta = time_delta
	#	self.enc_delta = enc_delta 
	#	self.angular_delta = enc_delta * (1.0 / 2048.0) * (2.0 * 3.1415926)
	#
	#	self.enc_vel_delta = self.angular_delta / time_delta.to_sec()
	#	self.vel_delta = vel_delta
	#	self.accel = vel_delta / time_delta.to_sec()

	def __init__(self, s1, s2):
		self.s1 = s1
		self.s2 = s2

		self.time_delta = s1.timestamp.to_sec() - s2.timestamp.to_sec()
		self.enc_delta = s1.encoder_val - s2.encoder_val
		self.vel_delta = (s1.vel - s2.vel)
		print "dV [rot]: " + str(self.vel_delta)
		self.angular_delta = self.enc_delta * (1.0/1000.0)
		print "dV [rad/s]: " + str(self.vel_delta)
		self.accel = self.vel_delta / self.time_delta
		#self.sensor_vel = (s1.vel + s2.vel) / 2.0
		#self.ang_vel = self.angular_delta / self.time_delta
		

		print "State1: " + str(s1)
		print "State2: " + str(s2)

	#def tickToRad(self,tick)
	#	return 
		

	def __str__(self):
		return "dt: " + str(self.time_delta) + "[s] dEnc: " + str(self.enc_delta) + "[rot] / " + str(self.angular_delta) + " [rad] Accel: " + str(self.accel) + "rad/s^2"
		#print "s1 Vel: " + str(s1.vel) + " [tick/100ms] s2 Vel: " + str(s1.vel) 

class MotorState():
	def __init__(self,timestamp,encoder_val,vel,throttle,err):
		self.timestamp = timestamp		
		self.encoder_val = encoder_val * (1.0 / 1000.0)
		self.vel = vel * (10.0 / 1000.0)
		self.throttle = throttle
		self.err = err * (10.0 / 1000.0)

	#def __neg__(self,other):
	#	return MotorStateDelta(self,other)

	def __str__(self):
		#angular = self.encoder_val * (1.0 / 2048.0) * (2.0 * 3.1415926)
		return "Timestamp: " + str(self.timestamp.to_sec()) + " Encoder: " + str(self.encoder_val) + " [rev] Throttle: " + str(self.throttle) + " Err: " + str(self.err) + " Vel: " + str(self.vel) + " rev/s" 



def Main():
	bag_path = '/home/antman/projects/bags/'
	a = AccelerationTest(bag_path)

	start_time = rospy.get_rostime()
	prev_time = start_time
	cur_time = start_time
	

	d = rospy.Duration.from_sec(2.0)
	end_duration = rospy.Duration.from_sec(4.5)
	r = rospy.Rate(20)
	i = 0
	while not rospy.is_shutdown():
		cur_time = rospy.get_rostime()
		if (cur_time - prev_time <= d):
			
			if (i % 2 == 1):
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

	#
	a.close()
	a.analyze()

	return

if __name__ == '__main__':
	Main()
	#bag_path = '/home/antman/projects/bags/'
	#analyzeBag(bag_path)
	

