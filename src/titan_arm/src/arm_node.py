#!/usr/bin/env python

import rospy
import sys



###############################################################################
# Main ROS interface
class TitanArmNode(ArbotiX):
    
    def __init__(self):
        pause = False

        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = int(rospy.get_param("~baud", "115200"))

        self.rate = rospy.get_param("~rate", 100.0)
        self.fake = rospy.get_param("~sim", False)

       
        # setup publishers
        self.diagnostics = DiagnosticsPublisher()
        self.joint_state_publisher = JointStatePublisher()

        # start an arbotix driver
        if not self.fake:
            ArbotiX.__init__(self, port, baud)        
            rospy.sleep(1.0)
            rospy.loginfo("Started ArbotiX connection on port " + port + ".")
        else:
            rospy.loginfo("ArbotiX being simulated.")

        # setup joints
        self.joints = dict()
        for name in rospy.get_param("~joints", dict()).keys():
            joint_type = rospy.get_param("~joints/"+name+"/type", "dynamixel")
            if joint_type == "dynamixel":
                self.joints[name] = DynamixelServo(self, name)
		rospy.loginfo("AWWW YEA")
            elif joint_type == "hobby_servo":
                self.joints[name] = HobbyServo(self, name)
            elif joint_type == "calibrated_linear":
                self.joints[name] = LinearJoint(self, name)

        # setup controller
        self.controllers = [ServoController(self, "servos"), ]
        controllers = rospy.get_param("~controllers", dict())
        for name, params in controllers.items():
            try:
                controller = controller_types[params["type"]](self, name)
                self.controllers.append( controller )
                pause = pause or controller.pause
            except Exception as e:
                if type(e) == KeyError:
                    rospy.logerr("Unrecognized controller: " + params["type"])
                else:  
                    rospy.logerr(str(type(e)) + str(e))

        # wait for arbotix to start up (especially after reset)
        if not self.fake:
            if rospy.has_param("~digital_servos") or rospy.has_param("~digital_sensors") or rospy.has_param("~analog_sensors"):
                pause = True
            if pause:
                while self.getDigital(1) == -1 and not rospy.is_shutdown():
                    rospy.loginfo("Waiting for response...")
                    rospy.sleep(0.25)
            rospy.loginfo("ArbotiX connected.")

        for controller in self.controllers:
            controller.startup()

        # services for io
        rospy.Service('~SetupAnalogIn',SetupChannel, self.analogInCb)
        rospy.Service('~SetupDigitalIn',SetupChannel, self.digitalInCb)
        rospy.Service('~SetupDigitalOut',SetupChannel, self.digitalOutCb)
        # initialize digital/analog IO streams
        self.io = dict()
        if not self.fake:
            for v,t in {"digital_servos":DigitalServo,"digital_sensors":DigitalSensor,"analog_sensors":AnalogSensor}.items():
                temp = rospy.get_param("~"+v,dict())
                for name in temp.keys():
                    pin = rospy.get_param('~'+v+'/'+name+'/pin',1)
                    value = rospy.get_param('~'+v+'/'+name+'/value',0)
                    rate = rospy.get_param('~'+v+'/'+name+'/rate',10)
                    leng = rospy.get_param('~'+v+'/'+name+'/length',1)  # just for analog sensors
                    self.io[name] = t(name, pin, value, rate, leng, self)
		    rospy.loginfo(name + " " + str(value))
        
        r = rospy.Rate(self.rate)

        # main loop -- do all the read/write here
        while not rospy.is_shutdown():
    
            # update controllers
            for controller in self.controllers:
                controller.update()

            # update io
            for io in self.io.values():
                io.update()

            # publish feedback
            self.joint_state_publisher.update(self.joints.values(), self.controllers)
            self.diagnostics.update(self.joints.values(), self.controllers)

            r.sleep()

        # do shutdown
        for controller in self.controllers:
            controller.shutdown()

    def analogInCb(self, req):
        # TODO: Add check, only 1 service per pin
        if not self.fake:
            self.io[req.topic_name] = AnalogSensor(req.topic_name, req.pin, req.value, req.rate, req.leng, self)
        return SetupChannelResponse()

    def digitalInCb(self, req):
        if not self.fake:
            self.io[req.topic_name] = DigitalSensor(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()

    def digitalOutCb(self, req):
        if not self.fake:
            self.io[req.topic_name] = DigitalServo(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()


if __name__ == "__main__":
    rospy.init_node('titan_arm_node')
    t = TitanArmNode()

