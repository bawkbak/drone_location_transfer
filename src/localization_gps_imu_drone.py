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
from duckieboat_msgs.srv import SetValue, SetValueRequest, SetValueResponse
import message_filters

from geodesy.utm import UTMPoint, fromLatLong
import tf
import math
from math import sqrt
from math import pow
from math import atan2
from scipy.stats import norm
import numpy as np

class LocailizationGPSImu(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.pose = Pose()
        self.prior_pose = Pose()
        self.prior_roll = 0
        self.prior_pitch = 0
        self.prior_yaw = 0
        self.start = False   
        self.covariance = np.zeros((36,), dtype=float)
        self.odometry = Odometry()
        self.slam_pose = PoseStamped()
        self.br = tf.TransformBroadcaster()
        
        # param
        self.imu_offset = 0
        self.lat_orig = rospy.get_param('~latitude', 0.0)
        self.long_orig = rospy.get_param('~longitude', 0.0)
        self.utm_orig = fromLatLong(self.lat_orig, self.long_orig)
        self.imu_rotate = rospy.get_param('~imu_rotate',np.pi/2)
        self.location_x = 0
        self.location_y = 0
        self.prior_location_x = 0
        self.prior_location_y = 0
        self.prior_time = 0
        self.time = 0

        # Service
        self.srv_imu_offset = rospy.Service('~imu_offset', SetValue, self.cb_srv_imu_offest)

        # Publisher
        self.pub_odm = rospy.Publisher("~odometry", Odometry, queue_size=1)
        self.pub_pose = rospy.Publisher("~pose", PoseStamped, queue_size=1)
        

        # Subscriber
        sub_imu = message_filters.Subscriber("imu", Imu)
        sub_gps = message_filters.Subscriber("gps", NavSatFix)
        ats = ApproximateTimeSynchronizer((sub_imu, sub_gps), queue_size = 1, slop = 0.1)
        ats.registerCallback(self.cb_gps_imu)

    def cb_gps(self, msg_gps):
        utm_point = fromLatLong(msg_gps.latitude, msg_gps.longitude)
        self.pose.position.x = utm_point.easting - self.utm_orig.easting
        self.pose.position.y = utm_point.northing - self.utm_orig.northing
        self.pose.position.z = 0


    def cb_imu(self, msg_imu):
        self.pose.orientation = msg_imu.orientation

    def cb_gps_imu(self, msg_imu, msg_gps):
        self.cb_gps(msg_gps)
        self.cb_imu(msg_imu)
        self.kalman_filter()

    def kalman_filter(self):

        q = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
        roll = tf.transformations.euler_from_quaternion(q)[0]
        pitch = tf.transformations.euler_from_quaternion(q)[1]
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        # self.pub_heading.publish(yaw)
        # self.pub_pitch.publish(pitch)
        yaw = yaw + self.imu_rotate

        if self.start == False:
            self.start = True
            self.prior_pose.position.x = norm(loc = self.pose.position.x, scale = 100)
            self.prior_pose.position.y = norm(loc = self.pose.position.y, scale = 100)
            self.prior_pose.position.z = norm(loc = self.pose.position.z, scale = 100)
            self.prior_roll = norm(loc = roll, scale = 10)
            self.prior_pitch = norm(loc = pitch, scale = 10)
            self.prior_yaw = norm(loc = yaw, scale = 10)
            return

        covariance = self.covariance

        #p rediction step
        kernel = norm(loc = 0, scale = 2)
        kernel_euler = norm(loc = 0, scale = 0.5)
        x = self.pose.position.x
        y = self.pose.position.y
        z = self.pose.position.z
        
        predicted_x = norm(loc = self.prior_pose.position.x.mean()+kernel.mean(), scale = np.sqrt(self.prior_pose.position.x.var()+kernel.var()))
        predicted_y = norm(loc = self.prior_pose.position.y.mean()+kernel.mean(), scale = np.sqrt(self.prior_pose.position.y.var()+kernel.var()))
        predicted_z = norm(loc = self.prior_pose.position.z.mean()+kernel.mean(), scale = np.sqrt(self.prior_pose.position.z.var()+kernel.var()))
        predicted_roll = norm(loc = self.prior_roll.mean()+kernel_euler.mean(), scale = np.sqrt(self.prior_roll.var()+kernel_euler.var()))
        predicted_pitch = norm(loc = self.prior_pitch.mean()+kernel_euler.mean(), scale = np.sqrt(self.prior_pitch.var()+kernel_euler.var()))
        predicted_yaw = norm(loc = self.prior_yaw.mean()+kernel_euler.mean(), scale = np.sqrt(self.prior_yaw.var()+kernel_euler.var()))

        # update step
        posterior_x = self.update_con(predicted_x, x, 0.05)
        posterior_y = self.update_con(predicted_y, y, 0.05)
        posterior_z = self.update_con(predicted_z, z, 0.05)
        posterior_roll = self.update_con(predicted_roll, roll, 0.05)
        posterior_pitch = self.update_con(predicted_pitch, pitch, 0.05)
        posterior_yaw = self.update_con(predicted_yaw, yaw, 0.05)

        self.prior_roll = posterior_roll
        self.prior_pitch = posterior_pitch
        self.prior_yaw = posterior_yaw

        self.prior_pose.position.x = posterior_x
        self.prior_pose.position.y = posterior_y
        self.prior_pose.position.z = posterior_z   

        self.odometry.pose.pose.position.x = posterior_x.mean()
        self.odometry.pose.pose.position.y = posterior_y.mean()
        self.odometry.pose.pose.position.z = posterior_z.mean()
        kf_euler = posterior_yaw.mean()
        qu = tf.transformations.quaternion_from_euler(0, 0, kf_euler+self.imu_offset)
        pose = Pose()
        pose.orientation.x = qu[0]
        pose.orientation.y = qu[1]
        pose.orientation.z = qu[2]
        pose.orientation.w = qu[3]
        self.odometry.pose.pose.orientation = pose.orientation

        #utm to gps
        # east = posterior_x.mean()+self.utm_orig.easting
        # north = posterior_y.mean()+self.utm_orig.northing
        # filter_latitude, filter_longitude = self.utmToLatLng(51, east, north)
        # self.pub_lat.publish(filter_latitude)
        # self.pub_lon.publish(filter_longitude)
        # x = (posterior_x.mean()-10)*1.2
        # y = ((posterior_y.mean()-15))*1.2
        # self.pub_x.publish(x)
        # self.pub_y.publish(y)

        #speed
        # if(self.prior_location_x == 0):
        #     self.prior_location_x = x
        #     self.prior_location_y = y
        #     self.prior_time = rospy.get_rostime()
        

        # self.time = rospy.get_rostime()
        # self.location_x = x
        # self.location_y = y



        # distance = sqrt(pow(self.location_x-self.prior_location_x,2)+pow(self.location_y-self.prior_location_y,2))
        # speed = distance/(self.time.to_nsec()-self.prior_time.to_nsec()) *1000000000
        # self.pub_speed.publish(speed)
        # heading = atan2((self.location_y-self.prior_location_y),(self.location_x-self.prior_location_x))
        # heading = heading*180/3.14
        # if(heading<0):
        #     heading = 360+heading
        
        # self.pub_heading.publish(heading)

        

        # self.prior_location_x = x
        # self.prior_location_y = y
        # self.prior_time = rospy.get_rostime()
        
        self.slam_pose.pose.position.x = self.odometry.pose.pose.position.x
        self.slam_pose.pose.position.y = self.odometry.pose.pose.position.y
        self.slam_pose.pose.position.z = self.odometry.pose.pose.position.z
        self.slam_pose.pose.orientation  = self.odometry.pose.pose.orientation 
        # publish
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = "map"
        self.slam_pose.header = self.odometry.header
        self.pub_odm.publish(self.odometry)
        self.pub_pose.publish(self.slam_pose)

        # tf transform
        self.br.sendTransform((self.odometry.pose.pose.position.x, \
                         self.odometry.pose.pose.position.y, self.odometry.pose.pose.position.z), \
                        (self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y, \
                        self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w), \
                        rospy.Time.now(),"/base_link","/map")

        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.br.sendTransform((self.utm_orig.easting, self.utm_orig .northing, 0), \
            (q[0], q[1], q[2], q[3]), \
            rospy.Time.now(),"/map","/utm")

        rad_2_deg = 180/math.pi
        # print("X = ", self.pose.position.x, ", Y = ", self.pose.position.y)
        # print(", RPY = ", posterior_roll.mean()*rad_2_deg, posterior_pitch.mean()*rad_2_deg, posterior_yaw.mean()*rad_2_deg+self.imu_offset*rad_2_deg)
        # print("========================================================")

    def cb_srv_imu_offest(self, request): 
        self.imu_offset = request.data
        print ("Set imu offset = " + str(self.imu_offset))
        return SetFloatResponse()

    def measurement(self, measurementx, variance):
        likelihood = norm(loc = measurementx, scale = np.sqrt(variance))
        return likelihood

    def gaussian_multiply(self, g1, g2):
        g1_mean, g1_var = g1.stats(moments='mv')
        g2_mean, g2_var = g2.stats(moments='mv')
        mean = (g1_var * g2_mean + g2_var * g1_mean) / (g1_var + g2_var)
        variance = (g1_var * g2_var) / (g1_var + g2_var)
        #print mean, variance
        return norm(loc = mean, scale = np.sqrt(variance))

    def update_con(self, prior, measurementz, covariance):
        likelihood = self.measurement(measurementz, covariance)
        posterior = self.gaussian_multiply(likelihood, prior)
        return posterior

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))


    def utmToLatLng(self, zone, easting, northing, northernHemisphere=True):
        if not northernHemisphere:
            northing = 10000000 - northing

        a = 6378137
        e = 0.081819191
        e1sq = 0.006739497
        k0 = 0.9996

        arc = northing / k0
        mu = arc / (a * (1 - math.pow(e, 2) / 4.0 - 3 * math.pow(e, 4) / 64.0 - 5 * math.pow(e, 6) / 256.0))

        ei = (1 - math.pow((1 - e * e), (1 / 2.0))) / (1 + math.pow((1 - e * e), (1 / 2.0)))

        ca = 3 * ei / 2 - 27 * math.pow(ei, 3) / 32.0

        cb = 21 * math.pow(ei, 2) / 16 - 55 * math.pow(ei, 4) / 32
        cc = 151 * math.pow(ei, 3) / 96
        cd = 1097 * math.pow(ei, 4) / 512
        phi1 = mu + ca * math.sin(2 * mu) + cb * math.sin(4 * mu) + cc * math.sin(6 * mu) + cd * math.sin(8 * mu)

        n0 = a / math.pow((1 - math.pow((e * math.sin(phi1)), 2)), (1 / 2.0))

        r0 = a * (1 - e * e) / math.pow((1 - math.pow((e * math.sin(phi1)), 2)), (3 / 2.0))
        fact1 = n0 * math.tan(phi1) / r0

        _a1 = 500000 - easting
        dd0 = _a1 / (n0 * k0)
        fact2 = dd0 * dd0 / 2

        t0 = math.pow(math.tan(phi1), 2)
        Q0 = e1sq * math.pow(math.cos(phi1), 2)
        fact3 = (5 + 3 * t0 + 10 * Q0 - 4 * Q0 * Q0 - 9 * e1sq) * math.pow(dd0, 4) / 24

        fact4 = (61 + 90 * t0 + 298 * Q0 + 45 * t0 * t0 - 252 * e1sq - 3 * Q0 * Q0) * math.pow(dd0, 6) / 720

        lof1 = _a1 / (n0 * k0)
        lof2 = (1 + 2 * t0 + Q0) * math.pow(dd0, 3) / 6.0
        lof3 = (5 - 2 * Q0 + 28 * t0 - 3 * math.pow(Q0, 2) + 8 * e1sq + 24 * math.pow(t0, 2)) * math.pow(dd0, 5) / 120
        _a2 = (lof1 - lof2 + lof3) / math.cos(phi1)
        _a3 = _a2 * 180 / math.pi

        latitude = 180 * (phi1 - fact1 * (fact2 + fact3 + fact4)) / math.pi

        if not northernHemisphere:
            latitude = -latitude

        longitude = ((zone > 0) and (6 * zone - 183.0) or 3.0) - _a3

        return (latitude, longitude)


if __name__ == '__main__':
    rospy.init_node('localization_gps_imu_node',anonymous=False)
    localization_gps_imu_node = LocailizationGPSImu()
    rospy.on_shutdown(localization_gps_imu_node.on_shutdown)
    rospy.spin()

