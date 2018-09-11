#!/usr/bin/env python

import rospy
from titan_msgs.msg import *

class FakeHardware():
	def __init__(self):
		self.motor_sub = rospy.Subscriber("/motor_velocity", MotorVelocity, self.cbMotorCmd)
	

		self.motor_status_pub = rospy.Publisher('/motor_status', Status, queue_size=10)
		
		self.prev_time = rospy.get_rostime()
		self.encoder_left = 0
		self.encoder_right = 0
		self.ticks_per_rev = 2048 / (2.0 * 3.1415926) #tick/rad

	def cbMotorCmd(self, msg):
		#print "Motor Velcoity Command:\n\tLeft: " + str(msg.left_angular_vel) + "\n\tRight: " + str(msg.right_angular_vel)
		dT = (rospy.get_rostime() - self.prev_time).to_sec()
		#print "dT: " + str(dT)
		left_ang_dist = msg.left_angular_vel * dT
		right_ang_dist = msg.right_angular_vel * dT

		

		left_tick = left_ang_dist * self.ticks_per_rev
		right_tick = right_ang_dist * self.ticks_per_rev

		self.encoder_left += left_tick
		self.encoder_right += right_tick
		self.prev_time = rospy.get_rostime()



	def update(self):
		#print "Update"
		#print "Current Tick:\n\tLeft: " + str(self.encoder_left) + "\n\tRight: " + str(self.encoder_right)
		status_left = Status()
		status_right = Status()

		status_left.DeviceId = 1
		status_left.header.stamp = rospy.get_rostime()
		status_left.SensorPosition = self.encoder_left
		self.motor_status_pub.publish(status_left);

		status_right.DeviceId = 4
		status_right.header.stamp = rospy.get_rostime()
		status_right.SensorPosition = self.encoder_right
		self.motor_status_pub.publish(status_right);
		



if __name__ == '__main__':
	rospy.init_node('fake_hardware', anonymous=True)

	r = rospy.Rate(20)
	f = FakeHardware()

	while not rospy.is_shutdown():
	
		f.update()
		r.sleep()
