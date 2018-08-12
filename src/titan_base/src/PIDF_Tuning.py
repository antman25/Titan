#!/usr/bin/env python

import rospy
from titan_base.msg import *
import numpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import sys

class PIDFTuning():

	def __init__(self):
		style.use('fivethirtyeight')
		plt.close('all')
		self.fig, self.axarr = plt.subplots(5,sharex=True)

		rospy.init_node('PIDFTuning', anonymous=True)
		
		self.recording = True
		self.time_pts = []
		self.error_pts = []
		self.throttle_pts = []
		self.pos_pts = []
		self.vel_pts = []
		self.sp_pts = []
		self.cur_sp = 0.0;
		
	
		rospy.Subscriber("/motor_status", Status, self.cbMotorStatus)
	

		self.vel_pub = rospy.Publisher('/motor_velocity', MotorVelocity, queue_size=10)
		self.pidf_pub = rospy.Publisher('/set_pidf_param', PIDF, queue_size=10)

		pidf = PIDF()
		pidf.P_Gain = 0.0
		pidf.I_Gain = 0.0
		pidf.D_Gain = 0.0
		pidf.F_Gain = 1.0
		pidf.FeedbackCoeff = 0.0
		pidf.RampRate = 100
		r = rospy.Rate(5)
		r.sleep()
		self.pidf_pub.publish(pidf)
		self.start_time = rospy.get_rostime()
		self.prev_time = rospy.get_rostime()
		
	def cbMotorStatus(self,status):
		dev_str = ''
	
		if (status.DeviceId == 1):
			dev_str = 'LEFT'
		if (status.DeviceId == 4):
			dev_str = 'RIGHT'
		timestamp = status.header.stamp.secs + (status.header.stamp.nsecs / 1.0e9)

		total_current = 0.0
		
		if (status.DeviceId == 1):
			#
			if (self.recording == True):
				sp_count = self.cur_sp * (102.40/(2*3.1415))
				rospy.loginfo(str(timestamp) + " - " + dev_str + " - V: %f / I: %f - CloseLoopErr: %i - Pos %i - Vel %i - Throttle: %i - SP: %f", status.BatteryV,status.Current, status.CloseLoopErr,status.SensorPosition,status.SensorVelocity,status.AppliedThrottle,sp_count)
				
				#recorded_data.append( (timestamp, status.CloseLoopErr, status.AppliedThrottle, status.SensorPosition, status.SensorVelocity))
				self.time_pts.append(timestamp)
				self.error_pts.append(status.CloseLoopErr)
				self.throttle_pts.append(status.AppliedThrottle)
				self.pos_pts.append(status.SensorPosition)
				self.vel_pts.append(status.SensorVelocity)
				self.sp_pts.append(sp_count)
	
	def compareVel(self):
		for i in range(len(self.time_pts)):
			if (i > 0):
				deltaTime = self.time_pts[i] - self.time_pts[i-1]
				deltaEncoder = self.pos_pts[i] - self.pos_pts[i-1]
				deltaRadian = deltaEncoder ;


				enc_vel = deltaEncoder / deltaTime;
				rad_vel = deltaRadian / deltaTime
				print "dt: " + str(deltaTime) + " dEncoder: " + str(deltaEncoder) + " dRadian: " + str(deltaRadian)
				print "Calc: " + str(enc_vel) + " tick/100 ms -- Sensor: " + str(self.vel_pts[i])
				print "Rad/s: " + str(rad_vel)

	def Main(self):
		d = rospy.Duration.from_sec(1.5)
		end_duration = rospy.Duration.from_sec(9.0)
		r = rospy.Rate(20)
		i = 0
		while not rospy.is_shutdown():
			cur_time = rospy.get_rostime()
			if (cur_time - self.prev_time <= d):
				motor_cmd = MotorVelocity()
				if (i % 2 == 0):
					self.cur_sp = 1.0 * 6.248
				else:
					self.cur_sp = 0
				motor_cmd.left_angular_vel = self.cur_sp;
				motor_cmd.right_angular_vel = self.cur_sp;
				self.vel_pub.publish(motor_cmd);
				
			else:
				self.prev_time = rospy.get_rostime()
				i = i+1
			if (cur_time - self.start_time >= end_duration):
				break
			r.sleep()

		self.recording = False
		#self.compareVel()
		
		self.axarr[0].clear()
		self.axarr[1].clear()
		self.axarr[2].clear()
		self.axarr[3].clear()

		self.axarr[0].plot(self.time_pts,self.pos_pts)
		self.axarr[1].plot(self.time_pts,self.error_pts)
		self.axarr[2].plot(self.time_pts,self.throttle_pts)		
		self.axarr[3].plot(self.time_pts,self.vel_pts)
		self.axarr[4].plot(self.time_pts,self.sp_pts)
		plt.show()
		#print self.pos_pts
		return

	#print "test"

if __name__ == '__main__':
	p = PIDFTuning()
	p.Main()
	#if sys.flags.interactive != 1 or not hasattr(pg.QtCore, 'PYQT_VERSION'):
	#	pg.QtGui.QApplication.exec_()
