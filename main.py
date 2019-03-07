#!/usr/bin/python

import rospy
import threading
from numpy import sin
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue, JointVelocities


class SpeedController:
	
	def __init__(self):
		rospy.init_node("speed_c_tests")

		self.js_sub = rospy.Subscriber("joint_states", JointState, self.js_callback)
		self.jp_pub = rospy.Publisher("arm_1/arm_controller/position_command", JointPositions, queue_size=10, latch=True)
		self.jv_pub = rospy.Publisher("arm_1/arm_controller/velocity_command", JointVelocities, queue_size=10, latch=True)

		self.cur_angles = []
		self.start_time = 0
		self.lock = threading.Lock()

		self.flag = True
		self.q = 5.6

		self.file = open("file.txt", "w")

	def __del__(self):
		print('AAAAAAAAAAAAa')
		self.file.close()

	def js_callback(self, js_msg):
		time = rospy.Time.now().to_sec() - self.start_time

		self.lock.acquire()
		ca = js_msg.position
		cv = js_msg.velocity
		ct = js_msg.effort
		if len(ct) == 0:
			ct = [0]
		k = 4
		if time < 10000:
			self.file.write(str(ca[k]) + ' ' + str(cv[k]) + ' ' + str(0) + ' ' + str(time) + '\n')

		self.lock.release()

		if ca[k] > 5.55:
			self.flag = True
			self.q = 0.12

	def goWithVelocity(self, v):
		jv_msg = JointVelocities()
		for i in range(1):
			jv_msg.velocities.append(JointValue())
			jv_msg.velocities[i].joint_uri = 'arm_joint_' + str(i+1)
			jv_msg.velocities[i].unit = 's^-1 rad'

		jv_msg.velocities[0].value = v
		self.jv_pub.publish(jv_msg)

	def goToPose(self, q):
		jp_msg = JointPositions()
		# for i in range(1):
		# 	jp_msg.positions.append(JointValue())
		# 	jp_msg.positions[i].joint_uri = 'arm_joint_' + str(i+1)
		# 	jp_msg.positions[i].unit = 'rad'

		jp_msg.positions.append(JointValue())
		jp_msg.positions[0].joint_uri = 'arm_joint_5'
		jp_msg.positions[0].unit = 'rad'

		jp_msg.positions[0].value = q
		self.jp_pub.publish(jp_msg)

	def loop(self):
		self.start_time = rospy.Time.now().to_sec()
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			if self.flag:
				self.flag = False
				self.goToPose(self.q)
			rate.sleep()

"""
def aaa(self):
flag = True
t = 0
v = 0
while not rospy.is_shutdown():
    t = t + 1
    if flag:
	    flag = False
	    #self.goToPose([])
	    v = 2*sin(0.5*t)
	    t = t + 0.1
	    self.goWithVelocity(-1)
    if t > 50:
	    self.goWithVelocity(0)
    rate.sleep()		
"""

if __name__=="__main__":
	sc = SpeedController()
	sc.loop()
