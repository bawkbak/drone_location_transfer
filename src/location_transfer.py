#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
import message_filters

from geodesy.utm import UTMPoint, fromLatLong
import tf
import math
from math import sqrt
from math import pow
from math import atan2
import numpy as np

class Tansfer(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.ekf_offset_x = 0
        self.ekf_offset_y = 0
        self.mavros_offset_x = 0
        self.mavros_offset_y = 0

        # Service

        # Publisher
        self.pub_goal = rospy.Publisher("drone/goal", PoseStamped, queue_size=1)
        
        # Subscriber
        self.sub_pose = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
        sub_ekf = message_filters.Subscriber("ekf/pose", PoseStamped)
        sub_mavros = message_filters.Subscriber("mavros/pose", PoseStamped)
        ats = ApproximateTimeSynchronizer((sub_ekf, sub_mavros), queue_size = 1, slop = 0.1)
        ats.registerCallback(self.syncCallback)

    def goalCallback(self, msg):
        msg_goal = PoseStamped()
        msg_goal.header.stamp = rospy.Time.now()
        msg_goal.header.frame_id = "map"
        msg_goal.pose.position.x = msg.pose.position.x + self.mavros_offset_x
        msg_goal.pose.position.y = msg.pose.position.y + self.mavros_offset_y
        msg_goal.pose.position.z = msg.pose.position.z 
        msg_goal.pose.orientation = msg.pose.orientation
        self.pub_goal.publish(msg_goal)


    def syncCallback(self, msg_ekf, msg_mavros):
        self.ekf_offset_x = msg_ekf.pose.position.x - msg_mavros.pose.position.x
        self.ekf_offset_y = msg_ekf.pose.position.y - msg_mavros.pose.position.y
        self.mavros_offset_x = msg_mavros.pose.position.x - msg_ekf.pose.position.x
        self.mavros_offset_y = msg_mavros.pose.position.y - msg_ekf.pose.position.y

        print("mavros_x", self.mavros_offset_x)
        print("mavros_y", self.mavros_offset_y)
        print("ekf_x", self.ekf_offset_x)
        print("ekf_y", self.ekf_offset_y)

        broadcast = tf.TransformBroadcaster()
        broadcast.sendTransform((self.ekf_offset_x, self.ekf_offset_y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "drone_origin",
                        "map")


if __name__ == '__main__':
    rospy.init_node('transfer',anonymous=False)
    uav = Tansfer()
    rospy.spin()

