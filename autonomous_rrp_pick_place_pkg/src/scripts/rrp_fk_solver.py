#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

class RRPForwardKinematics:
    def __init__(self):
        rospy.init_node('rrp_fk_solver_node', anonymous=True)

        # Base offsets from URDF
        self.d1 = 0.5  # Base height (0.2 + 0.3)
        self.a2 = 0.4  # Upper arm fixed length
        self.ee_offset = 0.4  # Base rod offset to end-effector tip

        # Subscriber for joint states
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Publisher for computed end-effector position
        self.ee_pub = rospy.Publisher('/rrp_fk_ee_position', PointStamped, queue_size=10)

        rospy.loginfo("RRP Forward Kinematics Solver Node Initialized")

    def dh_transform(self, theta, d, a, alpha):
        """ Computes 4x4 Homogeneous Transformation Matrix using DH parameters """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [ 0,       sa,       ca,      d],
            [ 0,        0,        0,      1]
        ])

    def compute_fk(self, q1, q2, q3):
        """
        Computes Forward Kinematics for RRP Manipulator
        q1: Revolute Yaw (rad)
        q2: Revolute Pitch (rad)
        q3: Prismatic Extension (m)
        """
        # DH Transformation Matrices
        T0_1 = self.dh_transform(q1, self.d1, 0.0, np.pi/2)
        T1_2 = self.dh_transform(q2, 0.0, self.a2, 0.0)
        T2_3 = self.dh_transform(0.0, q3 + self.ee_offset, 0.0, 0.0)

        # Overall Transformation T_0_ee
        T0_ee = T0_1.dot(T1_2).dot(T2_3)

        # Extract End-Effector Cartesian Position (X, Y, Z)
        x = T0_ee[0, 3]
        y = T0_ee[1, 3]
        z = T0_ee[2, 3]

        return x, y, z

    def joint_state_callback(self, msg):
        try:
            # Map joint names to current positions
            joint_positions = dict(zip(msg.name, msg.position))
            
            q1 = joint_positions.get('joint_1', 0.0)
            q2 = joint_positions.get('joint_2', 0.0)
            q3 = joint_positions.get('joint_3', 0.0)

            # Calculate FK position
            x, y, z = self.compute_fk(q1, q2, q3)

            # Publish calculated point
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "world"
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z

            self.ee_pub.publish(point_msg)

            rospy.loginfo_throttle(1.0, "Joints: [q1: %.2f rad, q2: %.2f rad, q3: %.2f m] -> FK EE Position: (X: %.3f, Y: %.3f, Z: %.3f)",
                                   q1, q2, q3, x, y, z)

        except Exception as e:
            rospy.logerr("FK Computation Error: %s", str(e))

if __name__ == '__main__':
    try:
        fk_solver = RRPForwardKinematics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass