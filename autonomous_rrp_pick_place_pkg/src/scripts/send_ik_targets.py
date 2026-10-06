#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped

def target_publisher():
    rospy.init_node('ik_target_publisher_node', anonymous=True)
    pub = rospy.Publisher('/rrp_ik_target', PointStamped, queue_size=10)
    
    rospy.sleep(1)

    # Target points requiring explicit prismatic joint extension (q3 > 0):
    # L = sqrt(X^2 + Y^2 + (Z - 0.5)^2)
    # q3 = L - 0.8m
    targets = [
        # 1. Medium extension straight forward: L = 1.0m  -> q3 = 0.20m
        (1.0, 0.0, 0.5),

        # 2. Maximum extension forward-left: L = 1.2m -> q3 = 0.40m (Max limit)
        (0.8, 0.8, 0.9),

        # 3. High overhead extension: L = 1.1m -> q3 = 0.30m
        (0.0, 0.7, 1.3),

        # 4. Partial extension forward-right: L = 0.95m -> q3 = 0.15m
        (0.6, -0.6, 0.35),

        # 5. Full extension sideways: L = 1.2m -> q3 = 0.40m
        (0.0, 1.1, 0.98)
    ]

    rate = rospy.Rate(0.33)  # Switch target every 3 seconds

    idx = 0
    rospy.loginfo("Starting sequence target publisher with active prismatic extension...")

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

        idx = (idx + 1) % len(targets)
        rate.sleep()

if __name__ == '__main__':
    try:
        target_publisher()
    except rospy.ROSInterruptException:
        pass