#!/usr/bin/env python

import xml.dom.minidom
from operator import add
import sys
import threading
from moveit_ros_planning_interface._moveit_robot_interface import RobotInterface

import rospy
import roslib
import numpy
import time
import tf
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerInit


class JoyStatus():
    def __init__(self):
        self.select = False
        self.start = False

        self.button_x = False
        self.button_y = False
        self.button_a = False
        self.button_b = False

        self.up = False
        self.down = False
        self.left = False
        self.right = False

        self.L1 = False
        self.R1 = False
	
	self.left_analog_x = 10.0
        self.left_analog_y = 10.0
        self.right_analog_x = 10.0
        self.right_analog_y = 10.0


class GamepadStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        #if msg.buttons[16] == 1:
        #    self.center = True
        #else:
        #    self.center = False

        if msg.buttons[8] == 1:
            self.select = True
        else:
            self.select = False

        if msg.buttons[9] == 1:
            self.start = True
        else:
            self.start = False


        if msg.buttons[3] == 1:
            self.button_x = True
        else:
            self.button_x = False

	if msg.buttons[2] == 1:
            self.button_y = True
        else:
            self.button_y = False
	
	if msg.buttons[0] == 1:
            self.button_a = True
        else:
            self.button_a = False

	if msg.buttons[1] == 1:
            self.button_b = True
        else:
            self.button_b = False
		
        if msg.axes[4] >= 0.9:
            self.up = True
        else:
            self.up = False

        if msg.axes[4] <= -0.9:
            self.down = True
        else:
            self.down = False

        if msg.axes[3] >= 0.9:
            self.left = True
        else:
            self.left = False

        if msg.axes[3] <= -0.9:
            self.right = True
        else:
            self.right = False
        
        if msg.buttons[10] == 1:
            self.L1 = True
        else:
            self.L1 = False
        if msg.buttons[11] == 1:
            self.R1 = True
        else:
            self.R1 = False
       
	self.left_analog_x = 10.0
        self.left_analog_y = 10.0
        self.right_analog_x = 10.0
        self.right_analog_y = 10.0

        self.orig_msg = msg


class StatusHistory():
	def __init__(self, max_length=10):
		self.max_length = max_length
		self.buffer = []
	def add(self, status):
		self.buffer.append(status)
		if len(self.buffer) > self.max_length:
			self.buffer = self.buffer[1:self.max_length+1]
	def all(self, proc):
		for status in self.buffer:
			if not proc(status):
				return False
		return True
	def latest(self):
		if len(self.buffer) > 0:
			return self.buffer[-1]
		else:
			return None
	def length(self):
		return len(self.buffer)
	def new(self, status, attr):
		if len(self.buffer) == 0:
			return getattr(status, attr)
		else:
			return getattr(status, attr) and not getattr(self.latest(), attr)

class MoveitJoy:
    def parseSRDF(self):
	print "test"
        ri = RobotInterface("/robot_description")
	#print "test2"
        planning_groups = {}
        #for g in ri.get_group_names():
	for g in ['ArmGroup']:
            self.planning_groups_tips[g] = ri.get_group_joint_tips(g)
            planning_groups[g] = ["/rviz/moveit/move_marker/goal_" + l
                                  for l in self.planning_groups_tips[g]]
        for name in planning_groups.keys():
            if len(planning_groups[name]) == 0:
                del planning_groups[name]
            else:
                print name, planning_groups[name]
        self.planning_groups = planning_groups
        self.planning_groups_keys = planning_groups.keys()   #we'd like to store the 'order'
        self.frame_id = ri.get_planning_frame()


    def __init__(self):
        self.initial_poses = {}
        self.planning_groups_tips = {}
        self.tf_listener = tf.TransformListener()
        self.marker_lock = threading.Lock()
        self.prev_time = rospy.Time.now()
        self.counter = 0
        self.history = StatusHistory(max_length=10)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.current_planning_group_index = 0
        self.current_eef_index = 0
        self.initialize_poses = False
        self.initialized = False
        self.parseSRDF()
        self.plan_group_pub = rospy.Publisher('/rviz/moveit/select_planning_group', String, queue_size=5)
        self.updatePlanningGroup(0)
        self.updatePoseTopic(0, False)
        self.joy_pose_pub = rospy.Publisher("/joy_pose", PoseStamped, queue_size=1)
        self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty, queue_size=5)
        self.execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty, queue_size=5)
        self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty, queue_size=5)
        self.update_goal_state_pub = rospy.Publisher("/rviz/moveit/update_goal_state", Empty, queue_size=5)
        self.interactive_marker_sub = rospy.Subscriber("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full",
                                                       InteractiveMarkerInit,
                                                       self.markerCB, queue_size=1)
        self.sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

    def updatePlanningGroup(self, next_index):
        if next_index >= len(self.planning_groups_keys):
            self.current_planning_group_index = 0
        elif next_index < 0:
            self.current_planning_group_index = len(self.planning_groups_keys) - 1
        else:
            self.current_planning_group_index = next_index
        next_planning_group = None
        try:
            next_planning_group = self.planning_groups_keys[self.current_planning_group_index]
        except IndexError:
            msg = 'Check if you started movegroups. Exiting.'
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)
        rospy.loginfo("Changed planning group to " + next_planning_group)
        self.plan_group_pub.publish(next_planning_group)
    def updatePoseTopic(self, next_index, wait=True):
        planning_group = self.planning_groups_keys[self.current_planning_group_index]
        topics = self.planning_groups[planning_group]
        if next_index >= len(topics):
            self.current_eef_index = 0
        elif next_index < 0:
            self.current_eef_index = len(topics) - 1
        else:
            self.current_eef_index = next_index
        next_topic = topics[self.current_eef_index]

        rospy.loginfo("Changed controlled end effector to " + self.planning_groups_tips[planning_group][self.current_eef_index])
        self.pose_pub = rospy.Publisher(next_topic, PoseStamped, queue_size=5)
        if wait:
            self.waitForInitialPose(next_topic)
        self.current_pose_topic = next_topic
    def markerCB(self, msg):
        try:
            self.marker_lock.acquire()
            if not self.initialize_poses:
                return
            self.initial_poses = {}
            for marker in msg.markers:
                if marker.name.startswith("EE:goal_"):
                    # resolve tf
                    if marker.header.frame_id != self.frame_id:
                        ps = PoseStamped(header=marker.header, pose=marker.pose)
                        try:
                            transformed_pose = self.tf_listener.transformPose(self.frame_id, ps)
                            self.initial_poses[marker.name[3:]] = transformed_pose.pose
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, e):
                            rospy.logerr("tf error when resolving tf: %s" % e)
                    else:
                        self.initial_poses[marker.name[3:]] = marker.pose   #tf should be resolved
        finally:
            self.marker_lock.release()
    def waitForInitialPose(self, next_topic, timeout=None):
        counter = 0
        while not rospy.is_shutdown():
            counter = counter + 1
            if timeout and counter >= timeout:
                return False
            try:
                self.marker_lock.acquire()
                self.initialize_poses = True
                topic_suffix = next_topic.split("/")[-1]
                if self.initial_poses.has_key(topic_suffix):
                    self.pre_pose = PoseStamped(pose=self.initial_poses[topic_suffix])
                    self.initialize_poses = False
                    return True
                else:
                    rospy.logdebug(self.initial_poses.keys())
                    rospy.loginfo("Waiting for pose topic of '%s' to be initialized",
                                  topic_suffix)
                    rospy.sleep(1)
            finally:
                self.marker_lock.release()
    def joyCB(self, msg):

	status = GamepadStatus(msg)
        self.run(status)
        self.history.add(status)

    def computePoseFromJoy(self, pre_pose, status):
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time(0.0)
        # move in local
        dist = status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x
        scale = 200.0
        x_diff = signedSquare(status.left_analog_y) / scale
        y_diff = signedSquare(status.left_analog_x) / scale
        # z
        if status.L2:
            z_diff = 0.005
        elif status.R2:
            z_diff = -0.005
        else:
            z_diff = 0.0
        if self.history.all(lambda s: s.L2) or self.history.all(lambda s: s.R2):
            z_scale = 4.0
        else:
            z_scale = 2.0
        local_move = numpy.array((x_diff, y_diff,
                                  z_diff * z_scale,
                                  1.0))
        q = numpy.array((pre_pose.pose.orientation.x,
                         pre_pose.pose.orientation.y,
                         pre_pose.pose.orientation.z,
                         pre_pose.pose.orientation.w))
        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q),
                         local_move)
        new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
        new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
        new_pose.pose.position.z = pre_pose.pose.position.z + xyz_move[2]
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.005
        if status.L1:
            if self.history.all(lambda s: s.L1):
                yaw = yaw + DTHETA * 2
            else:
                yaw = yaw + DTHETA
        elif status.R1:
            if self.history.all(lambda s: s.R1):
                yaw = yaw - DTHETA * 2
            else:
                yaw = yaw - DTHETA
        if status.up:
            if self.history.all(lambda s: s.up):
                pitch = pitch + DTHETA * 2
            else:
                pitch = pitch + DTHETA
        elif status.down:
            if self.history.all(lambda s: s.down):
                pitch = pitch - DTHETA * 2
            else:
                pitch = pitch - DTHETA
        if status.right:
            if self.history.all(lambda s: s.right):
                roll = roll + DTHETA * 2
            else:
                roll = roll + DTHETA
        elif status.left:
            if self.history.all(lambda s: s.left):
                roll = roll - DTHETA * 2
            else:
                roll = roll - DTHETA
        diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        new_q = tf.transformations.quaternion_multiply(q, diff_q)
        new_pose.pose.orientation.x = new_q[0]
        new_pose.pose.orientation.y = new_q[1]
        new_pose.pose.orientation.z = new_q[2]
        new_pose.pose.orientation.w = new_q[3]
        return new_pose
    def run(self, status):
        if not self.initialized:
            # when not initialized, we will force to change planning_group
            while True:
                self.updatePlanningGroup(self.current_planning_group_index)
                planning_group = self.planning_groups_keys[self.current_planning_group_index]
                topics = self.planning_groups[planning_group]
                next_topic = topics[self.current_eef_index]
                if not self.waitForInitialPose(next_topic, timeout=3):
                    rospy.logwarn("Unable to initialize planning group " + planning_group + ". Trying different group.")
                    rospy.logwarn("Is 'Allow External Comm.' enabled in Rviz? Is the 'Query Goal State' robot enabled?")
                else:
                    rospy.loginfo("Initialized planning group")
                    self.initialized = True
                    self.updatePoseTopic(self.current_eef_index)
                    return
                # Try to initialize with different planning group
                self.current_planning_group_index += 1
                if self.current_planning_group_index >= len(self.planning_groups_keys):
                    self.current_planning_group_index = 0 # reset loop
        if self.history.new(status, "select"):   #increment planning group
            self.updatePlanningGroup(self.current_planning_group_index + 1)
            self.current_eef_index = 0    # force to reset
            self.updatePoseTopic(self.current_eef_index)
            return
        elif self.history.new(status, "start"):   #decrement planning group
            self.updatePlanningGroup(self.current_planning_group_index - 1)
            self.current_eef_index = 0    # force to reset
            self.updatePoseTopic(self.current_eef_index)
            return
        elif self.history.new(status, "triangle"):
            self.updatePoseTopic(self.current_eef_index + 1)
            return
        elif self.history.new(status, "cross"):
            self.updatePoseTopic(self.current_eef_index - 1)
            return
        elif self.history.new(status, "square"):   #plan
            rospy.loginfo("Plan")
            self.plan_pub.publish(Empty())
            return
        elif self.history.new(status, "circle"):   #execute
            rospy.loginfo("Execute")
            self.execute_pub.publish(Empty())
            return
        self.marker_lock.acquire()
        pre_pose = self.pre_pose
        new_pose = self.computePoseFromJoy(pre_pose, status)
        now = rospy.Time.from_sec(time.time())
        # placement.time_from_start = now - self.prev_time
        if (now - self.prev_time).to_sec() > 1 / 30.0:
            # rospy.loginfo(new_pose)
            self.pose_pub.publish(new_pose)
            self.joy_pose_pub.publish(new_pose)
            self.prev_time = now
        # sync start state to the real robot state
        self.counter = self.counter + 1
        if self.counter % 10:
            self.update_start_state_pub.publish(Empty())
        self.pre_pose = new_pose
        self.marker_lock.release()
        # update self.initial_poses
        self.marker_lock.acquire()
        self.initial_poses[self.current_pose_topic.split("/")[-1]] = new_pose.pose
        self.marker_lock.release()


if __name__ == "__main__":
    rospy.init_node("moveit_joy")
    app = MoveitJoy()
    rospy.spin()
