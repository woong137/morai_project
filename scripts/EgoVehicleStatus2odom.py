#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from morai_msgs.msg import EgoVehicleStatus
from tf.transformations import quaternion_from_euler


class EgoVehicleStatus2odom:
    def __init__(self):
        rospy.init_node("EgoVehicleStatus2odom", anonymous=False)
        rospy.Subscriber(
            "/Ego_topic", EgoVehicleStatus, self.ego_vehicle_status_callback
        )
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = "base_link"

        self.is_status = False
        self.is_odom = False

    def ego_vehicle_status_callback(self, data):
        self.ego_vehicle_status = data
        self.is_status = True

    def convert(self):
        rate = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():
            if self.is_status == True:
                self.odom_msg.header.stamp = rospy.Time.now()
                self.odom_msg.pose.pose.position.x = self.ego_vehicle_status.position.x
                self.odom_msg.pose.pose.position.y = self.ego_vehicle_status.position.y
                self.odom_msg.pose.pose.position.z = self.ego_vehicle_status.position.z

                q = quaternion_from_euler(0, 0, np.deg2rad(self.ego_vehicle_status.heading))
                self.odom_msg.pose.pose.orientation = Quaternion(*q)

                self.odom_msg.twist.twist.linear.x = self.ego_vehicle_status.velocity.x
                self.odom_msg.twist.twist.linear.y = self.ego_vehicle_status.velocity.y
                self.odom_msg.twist.twist.linear.z = self.ego_vehicle_status.velocity.z

                self.odom_pub.publish(self.odom_msg)
                self.is_odom = True
            rate.sleep()

if __name__ == "__main__":
    try:
        ego_vehicle_status_2_odom = EgoVehicleStatus2odom()
        ego_vehicle_status_2_odom.convert()
    except rospy.ROSInterruptException:
        pass
