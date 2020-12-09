#!/usr/bin/env python3
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
import math
from std_msgs.msg import Int8MultiArray, Header, String
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import atexit
import numpy as np
import matplotlib.pyplot as plt
import time

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from helper_functions import TFHelper

def exit_handler(node):
    # np.savetxt("map.csv",
    #        node.map,
    #        delimiter =", ",
    #        fmt ='% s')
    # plt.close('all')
    fig = plt.figure()
    plt.scatter(node.map_x, node.map_y)
    plt.scatter(node.map_neatox, node.map_neatoy, color='r')
    plt.scatter(node.map_neatox[0], node.map_neatoy[0], color='y')
    plt.show()
    time.sleep(5)
    plt.close()

class NeatoController():
    "This class encompasses multiple behaviors for the simulated neato"
    def __init__(self):
        rospy.init_node('finite_state')
        self.distance_threshold = 0.8
        self.state = "teleop"
        self.x = 0
        self.y = 0
        self.rotation = 0
        self.linear_error = 0
        self.angular_error = 0
        self.linear_k = 0.1
        self.angular_k = .005
        self.vel_msg = Twist()
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('stable_scan', LaserScan, self.process_scan)
        rospy.Subscriber('projected_stable_scan', PointCloud2, self.projected_scan_received)
        rospy.Subscriber('odom', Odometry, self.process_odom)
        # Initializing user input
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.map_y = []
        self.map_x = []
        self.map_neatox = []
        self.map_neatoy = []
        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from
        self.transform_helper = TFHelper()
        self.initialized = True


    def run(self):
        """ The run loop repeatedly executes the current state function.  Each state function will return a function
        corresponding to the next state to run. """
        # this sleep is to allow any subscribers to cmd_vel to establish a connection to our publisher.  This is only
        # needed in the case where you send the velocity commands once (in some ways sending the commands repeatedly is
        # more robust.
        rospy.sleep(1)
        while not rospy.is_shutdown():
            if self.state == "teleop":
                self.teleop()
            if self.state == "square":
                self.square()
            if self.state == "origin":
                self.origin()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def teleop(self):
        # Checking user input
        if self.key != '\x03':
            self.key = self.getKey()
            if self.key == 'w':
            	self.vel_msg.angular.z = 0
            	self.vel_msg.linear.x = 1
            elif self.key == 'a':
            	self.vel_msg.linear.x = 0
            	self.vel_msg.angular.z = 1
            elif self.key == 's':
            	self.vel_msg.angular.z = 0
            	self.vel_msg.linear.x = -1
            elif self.key == 'd':
            	self.vel_msg.linear.x = 0
            	self.vel_msg.angular.z = -1
            elif self.key == ' ':
            	self.vel_msg.angular.z = 0
            	self.vel_msg.linear.x = 0
            elif self.key == '1':
                self.vel_msg.angular.z = 0
                self.vel_msg.linear.x = 0
                self.state = "square"
            elif self.key == '2':
                self.vel_msg.angular.z = 0
                self.vel_msg.linear.x = 0
                self.state = "origin"
            self.vel_pub.publish(self.vel_msg) # send instructions to the robot
            rospy.Rate(10).sleep

    def square(self):
        print("X position: ", self.x, "\n Y position: ", self.y)
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(linear=Vector3(x=1)))
        rospy.sleep(2)
        self.vel_pub.publish(Twist(angular=Vector3(z=-math.pi/4)))
        rospy.sleep(2)
        #make sure it's not moving when it goes back to teleop
        self.vel_msg.angular.z = 0
        self.vel_msg.linear.x = 0
        self.vel_pub.publish(self.vel_msg)
        print("X position: ", self.x, "\n Y position: ", self.y)
        self.state = "teleop"

    def process_scan(self, msg):
        pass
        # wait a little while to see if the transform becomes available.  This fixes a race
        # condition where the scan would arrive a little bit before the odom to base_link transform
        # was updated.
        # self.tf_listener.waitForTransform(self.base_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
        # # calculate pose of laser relative to the robot base
        # p = PoseStamped(header=Header(stamp=rospy.Time(0),
        #                               frame_id=msg.header.frame_id))
        # laser_pose = self.tf_listener.transformPose(self.base_frame, p)
        # print(laser_pose)
        # #print(self.laser_pose.pose.position)
        # #print(self.laser_pose)
        # print(msg.header.stamp)
        # angles = np.linspace(0, 2*math.pi, num=361)
        # for i,point in enumerate(msg.ranges):
        #     if point != 0:
        #         x = point * np.cos(angles[i])
        #         y = point * np.sin(angles[i])
        #         self.map_x.append(x - self.x)
        #         self.map_y.append(y - self.y)

    def projected_scan_received(self, msg):
        for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
            print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
            self.map_x.append(p[0])
            self.map_y.append(p[1])
        # angles = np.linspace(0, 2*math.pi, num=361)
        # for i, point in enumerate(msg.data):
        #     if point != 0:
        #         x = point * np.cos(angles[i])
        #         y = point * np.sin(angles[i])
        #         self.map_x.append(x - self.x)
        #         self.map_y.append(y - self.y)


    def process_odom(self,msg):
        #get our x and y position relative to the world origin"
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Orientation of the neato according to global reference frame (odom)
        self.rotation = 180 - math.degrees(euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0])
        # print("x: ", self.x, ", y: ", self.y, ", angle: ", self.rotation)
        self.map_neatox.append(self.x)
        self.map_neatoy.append(self.y)

    def origin(self):
        if self.linear_error < 1:
            self.state = "teleop"
        else:
            self.angular_vel = self.angular_k * self.angular_error
            self.linear_vel = self.linear_k * self.linear_error
            # send instructions to the robot
            self.vel_pub.publish(Twist(linear=Vector3(x=self.linear_vel), angular=Vector3(z=self.angular_vel)))
            #print("Angular velocity: ", self.angular_vel, ", Linear velocity: ", self.linear_vel)
            rospy.Rate(10).sleep

if __name__ == '__main__':
    node = NeatoController()
    node.run()
    atexit.register(exit_handler,node)
