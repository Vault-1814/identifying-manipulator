#!/usr/bin/python

import rospy
import threading
from numpy import arange, zeros, sin, pi, cos

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse
from brics_actuator.msg import JointPositions, JointValue, JointVelocities

from youbot_arm_kinematics.kinematics import Kinematics
from youbot_arm_kinematics.dh_parameters import YouBotDHParameters


class TestTrajectories:

    def __init__(self):
        self.log_file = open('trajectories_mes.txt', 'w')
        self.log_file_goal = open('trajectories_goal.txt', 'w')
        self.log_file_xyz = open('trajectories_mes_xyz.txt', 'w')

        rospy.init_node("test_trajectories")
        self.lock = threading.Lock()

        self.js_sub = rospy.Subscriber("joint_states", JointState, self.js_callback)
        self.jp_pub = rospy.Publisher("arm_1/arm_controller/position_command", JointPositions, queue_size=10,
                                      latch=True)
        self.jv_pub = rospy.Publisher("arm_1/arm_controller/velocity_command", JointVelocities, queue_size=10,
                                      latch=True)

        self.ks = Kinematics(YouBotDHParameters.DH_A, YouBotDHParameters.DH_ALPHA,
                             YouBotDHParameters.DH_D, YouBotDHParameters.DH_THETA)

        self.loop()
        self.cur_t = 0

    def __del__(self):
        self.log_file.close()
        self.log_file_xyz.close()

    def js_callback(self, js_msg):
        self.lock.acquire()
        cur_angs = js_msg.position
        cur_vels = js_msg.velocity
        self.lock.release()

        self.log_file.write("{} {} {} {} {} {} {} {} {} {} {}\n".format(self.cur_t, cur_angs[0], cur_angs[1], cur_angs[2], cur_angs[3], cur_angs[4],
                            cur_vels[0], cur_vels[1], cur_vels[2], cur_vels[3], cur_vels[4]))

    def getQ(self, t):
        N = 5

        joints_min = [0.025, 0.025, -5.0, 0.025, 0.12]
        joints_max = [5.80, 2.6, -0.025, 3.4, 5.64]

        jointsA = [abs(joints_max[i] - joints_min[i]) for i in range(N)]
        jointsAd2 = [jointsA[i] / 2 for i in range(N)]

        nf = [8, 8, 8, 8, 8]
        T = [9, 6, 7, 4.5, 6]

        # a = [  # 1    2      6    4     5      6     7     8
        #     [-1, -3.00, 0.50, 0.10, 1.00, 0.10, 0.50, 0.10, 3.00],
        #     [-1, 2.00, 0.10, 1.00, 0.10, 0.00, 0.00, 0.00, 1.50],
        #     [-1, -3.00, 1.00, 0.50, 1.00, 0.50, 1.00, 1.00, 6.00],
        #     [-1, -2.00, 0.10, 0.10, 1.00, 0.10, 0.02, 0.10, 1.00],
        #     [-1, 3.00, 0.10, 1.00, 0.02, 0.10, 0.02, 0.10, 2.00]
        # ]

        a = [  # 1    2      6    4     5      6     7     8
            [-1, -3.00, 0.50, 0.10, 1.00, 0.10, 0.50, 0.10, 3.00],
            [-1, 2.00, 0.10, 1.00, 0.10, 0.00, 0.00, 0.00, 1.50],
            [-1, -3.00, 1.00, 0.50, 1.00, 0.50, 1.00, 1.00, 6.00],
            [-1, -2.00, 0.10, 0.10, 1.00, 0.10, 0.02, 0.10, 1.00],
            [-1, 3.00, 0.10, 1.00, 0.02, 0.10, 0.02, 0.10, 2.00]
        ]

        scale = [0.39, 0.29, 0.2, 0.42, 0.45]
        q0 = [jointsAd2[0] - 0.4, jointsAd2[1] - 0.13, -jointsAd2[2] - 0.3, jointsAd2[3] - 0.3, jointsAd2[4] - 0.3]

        q, dq = zeros(N), zeros(N)
        for i in range(N):
            w0 = 2 * pi / T[i]
            for k in range(1, nf[i] + 1):
                q[i] = q[i] + a[i][k] * cos(k * w0 * t)
                dq[i] = dq[i] - a[i][k] * sin(k * w0 * t)
            q[i] = q0[i] + scale[i] * q[i]
            dq[i] = scale[i] * dq[i]
        return q, dq

    def loop(self):

        rate = rospy.Rate(30)
        start_time = rospy.Time.now().to_sec()
        prev_t = 0
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - start_time
            dt = t - prev_t
            prev_t = t
            self.cur_t = t
            # print(t)

            q, dq = self.getQ(t)
            (x, y, z), qtn, rpy, h = self.ks.forward(q)
            self.log_file_goal.write("{} {} {} {} {} {} {} {} {} {} {}\n".format(t, q[0], q[1], q[2], q[3], q[4], dq[0], dq[1], dq[2], dq[3], dq[4],))
            self.log_file_xyz.write("{} {} {}\n".format(x, y, z))

            # safe zone: cylinder, plane of floor
            R, z0 = 0.1, 0.4
            x0, y0 = 0, 0
            if (((x - x0)**2 + (y - y0)**2 > R**2) or (z > z0)) and (z > 0.1):
                jp_msg = JointPositions()
                for i in range(5):
                    jp_msg.positions.append(JointValue())
                    jp_msg.positions[i].joint_uri = 'arm_joint_' + str(i+1)
                    jp_msg.positions[i].unit = 'rad'
                    jp_msg.positions[i].value = q[i]
                self.jp_pub.publish(jp_msg)
                # jv_msg = JointVelocities()
                # for i in range(5):
                #     jv_msg.velocities.append(JointValue())
                #     jv_msg.velocities[i].joint_uri = 'arm_joint_' + str(i + 1)
                #     jv_msg.velocities[i].unit = 's^-1 rad'
                #     jv_msg.velocities[i].value = dq[i]
                # self.jv_pub.publish(jv_msg)
            else:
                print("COLLISION: " + str(t))
            rate.sleep()


if __name__ == "__main__":
    tt = TestTrajectories()
