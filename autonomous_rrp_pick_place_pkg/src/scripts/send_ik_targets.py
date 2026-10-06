#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped

def target_publisher():
    rospy.init_node('ik_target_publisher_node', anonymous=True)
    pub = rospy.Publisher('/rrp_ik_target', PointStamped, queue_size=10)
    
    # Wait for subscribers to connect before publishing
    rospy.sleep(1)

    # Sequence of target Cartesian coordinates inside reachable workspace [0.8m <= L <= 1.2m]
    targets = [
        (0.8, 0.0, 0.5),    # Extended straight forward
        (0.6, 0.4, 0.7),    # Front-left diagonal high
        (0.0, 0.8, 0.5),    # Left side horizontal
        (0.5, -0.5, 0.6),   # Front-right lower
        (0.7, 0.0, 0.9)     # High overhead reach
    ]

    rate = rospy.Rate(0.33)  # Publish a new target every 3 seconds

    idx = 0
    rospy.loginfo("Starting sequence target publisher...")

    while not rospy.is_shutdown():
        x, y, z = targets[idx]

        target_msg = PointStamped()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = 'world'
        target_msg.point.x = x
        target_msg.point.y = y
        target_msg.point.z = z

        pub.publish(target_msg)
        rospy.loginfo("Published Target %d/%d: Point (X=%.2f, Y=%.2f, Z=%.2f)", 
                      idx + 1, len(targets), x, y, z)

        idx = (idx + 1) % len(targets)  # Loop back through target list
        rate.sleep()

if __name__ == '__main__':
    try:
        target_publisher()
    except rospy.ROSInterruptException:
        pass