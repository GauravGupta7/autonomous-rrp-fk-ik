#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

class RRPInverseKinematics:
    def __init__(self):
        rospy.init_node('rrp_ik_solver_node', anonymous=True)

        # Robot Link Lengths
        self.d1 = 0.5          # Base height to joint_2
        self.a2 = 0.4          # Upper arm fixed length
        self.ee_offset = 0.4   # Prismatic rod base offset to tool tip

        # Joint Limits
        self.q1_limits = (-3.1415, 3.1415)
        self.q2_limits = (-1.5708, 1.5708)
        self.q3_limits = (0.0, 0.4)

        # Active joint pose state (defaults to [0.0, 0.0, 0.0])
        self.current_q = [0.0, 0.0, 0.0]

        # Subscriber & Publisher
        self.target_sub = rospy.Subscriber('/rrp_ik_target', PointStamped, self.target_callback)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        rospy.loginfo("RRP IK Solver initialized as active /joint_states streamer.")

    def compute_ik(self, x, y, z):
        q1 = np.arctan2(y, x)

        r = np.sqrt(x**2 + y**2)
        delta_z = z - self.d1
        
        L = np.sqrt(r**2 + delta_z**2)
        q2 = np.arctan2(delta_z, r)

        q3 = L - self.a2 - self.ee_offset

        if not (self.q1_limits[0] <= q1 <= self.q1_limits[1]):
            rospy.logwarn("Target out of bounds: q1 = %.2f rad", q1)
            return None
        if not (self.q2_limits[0] <= q2 <= self.q2_limits[1]):
            rospy.logwarn("Target out of bounds: q2 = %.2f rad", q2)
            return None
        if not (self.q3_limits[0] <= q3 <= self.q3_limits[1]):
            rospy.logwarn("Target out of reach: required q3 = %.3f m", q3)
            return None

        return q1, q2, q3

    def target_callback(self, msg):
        ik_result = self.compute_ik(msg.point.x, msg.point.y, msg.point.z)
        if ik_result is not None:
            self.current_q = list(ik_result)
            rospy.loginfo("New IK Solution set: [q1: %.3f rad, q2: %.3f rad, q3: %.3f m]", *self.current_q)

    def publish_loop(self):
        rate = rospy.Rate(30) # 30 Hz streaming loop
        while not rospy.is_shutdown():
            joint_msg = JointState()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = ['joint_1', 'joint_2', 'joint_3']
            joint_msg.position = self.current_q
            self.joint_pub.publish(joint_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        ik_solver = RRPInverseKinematics()
        ik_solver.publish_loop()
    except rospy.ROSInterruptException:
        pass