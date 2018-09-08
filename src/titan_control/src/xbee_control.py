
#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
import serial

def talker():
    ser = serial.Serial('/dev/ttyUSB0', 9600)

    pub = rospy.Publisher('xbee_data', String)
    rospy.init_node('xbee_broadcaster')
    while not rospy.is_shutdown():
       data= ser.read(2) # I have "hi" coming from the arduino as a test run over the serial port
       rospy.loginfo(data)
       pub.publish(String(data))
       rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
