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

# Should give us a rotation and translation. Delta x, delta y, delta theta
from icp.icp import icp

from graphslam.edge.edge_odometry import EdgeOdometry
from graphslam.graph import Graph
from graphslam.pose.r2 import PoseR2
from graphslam.pose.r3 import PoseR3
from graphslam.pose.se2 import PoseSE2
from graphslam.pose.se3 import PoseSE3
from graphslam.vertex import Vertex


def save_data(node):
    """
    This function saves data from projected stable laser scans and neato positional data from odom to file.
    """
    np.savetxt("map.csv",
           ((node.map_x, node.map_y),(node.map_neatox, node.map_neatoy)),
           delimiter =", ",
           fmt ='% s')

def exit_handler(node):
    """
    This function is called after the
    """
    save_data(node)
    plot_transform(node)
    # fig = plt.figure()
    # for i in range(len(node.map_x)):
    #     plt.scatter(node.map_x[i], node.map_y[i], color='b', alpha=0.3)
    # plt.scatter(node.map_neatox, node.map_neatoy, color='r')
    # plt.scatter(node.map_neatox[0], node.map_neatoy[0], color='y')

def plot_transform(node):
    # Plot the original scan at beginning and end of loop closure
    fig, ax = plt.subplots(nrows=1, ncols=2, squeeze=False)
    ax[0][0].scatter(node.map_x[0], node.map_y[0])
    ax[0][0].scatter(node.map_x[node.index_saved], node.map_y[node.index_saved])

    # Calculate the transform for the old scan
    res_old = apply_transform(node.old_scan, node)

    # Plot the transformed start scan over the original loop closure scan
    # TODO: Flip so it transforms the new scan instead
    ax[0][1].scatter(res_old[0], res_old[1])
    ax[0][1].scatter(node.map_x[node.index_saved], node.map_y[node.index_saved])

    # Add titles to the subplots
    ax[0][0].title.set_text('Original Loop Closure Scans')
    ax[0][1].title.set_text('After ICP Transform')
    plt.show()

    time.sleep(5)
    plt.close()

def apply_transform(scan, node):
    # Calculate the transformed scan
    # Make C a homogeneous representation of B (later scan)
    C = np.ones((len(scan), 3))
    C[:,0:2] = scan
    print("Before transform", C.shape)

    # Transform C
    C = np.dot(node.transform, C.T).T
    print("After transform", C.shape)
    res = np.rot90(C, 3)
    print(res.shape)
    print(res[0])
    print(res[1])
    print(res[2])
    return res

class NeatoController():
    """
    This class encompasses multiple behaviors for the simulated neato
    """
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
        # rospy.Subscriber('stable_scan', LaserScan, self.process_scan)
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
        self.moved_flag = False
        self.init_x = None
        self.init_y = None
        self.init_z = None
        self.starting_threshold = 0.1
        self.transform = None
        self.index_saved = 0
        self.old_scan = []
        self.new_scan = []


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
            	self.vel_msg.linear.x = 0.5
            elif self.key == 'a':
            	self.vel_msg.linear.x = 0
            	self.vel_msg.angular.z = 1
            elif self.key == 's':
            	self.vel_msg.angular.z = 0
            	self.vel_msg.linear.x = -0.5
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
        #print("X position: ", self.x, "\n Y position: ", self.y)
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
        #print("X position: ", self.x, "\n Y position: ", self.y)
        self.state = "teleop"

    def projected_scan_received(self, msg):
        current_mapx = []
        current_mapy = []
        for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
            #print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
            current_mapx.append(p[0])
            current_mapy.append(p[1])
        self.map_x.append(current_mapx)
        self.map_y.append(current_mapy)


    def process_odom(self,msg):
        #get our x and y position relative to the world origin"
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.init_x == None:
            self.init_x = self.x
        if self.init_y == None:
            self.init_y = self.y

        # Orientation of the neato according to global reference frame (odom)
        self.rotation = 180 - math.degrees(euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0])
        # print("x: ", self.x, ", y: ", self.y, ", angle: ", self.rotation)
        self.map_neatox.append(self.x)
        self.map_neatoy.append(self.y)

        distance = np.sqrt((self.x - self.init_x)**2 + (self.y - self.init_y)**2)

        if distance > self.starting_threshold and not self.moved_flag:
            self.moved_flag = True

        if distance < self.starting_threshold and self.moved_flag:
            print('Oh boy thats a loop closure')
            self.moved_flag = False
            self.index_saved = len(self.map_x)-1
            # Both scans should be in format: (361,2)
            self.old_scan = np.rot90(np.array((self.map_x[0], self.map_y[0])))
            self.new_scan = np.rot90(np.array((self.map_x[-1],self.map_y[-1])))
            # Use ICP to get 3x3 transformation matrix
            t,d,i = icp(self.old_scan,self.new_scan)
            node.transform = t

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
