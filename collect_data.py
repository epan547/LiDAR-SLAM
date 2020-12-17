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
import pickle

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
    print("lidar length: ", len(node.data['scans']))
    print("length of each scan: ", len(node.data['scans'][0]))
    print("odom length: ", len(node.data['odom']))
    print("closures length: ", len(node.data['closures']))
    filename = ('map'+ str(time.localtime())) + '.g2o'
    outfile = open(filename, 'wb')
    pickle.dump(node.data, outfile)
    outfile.close()

def exit_handler(node):
    """
    This function is called when the program is terminated. It saves the LiDAR and odometry data that the robot
    collected between start and the first loop close, then  runs ICP on the first and last scans, showing the
    ouput.
    """
    save_data(node)
    plot_transform(node)
    # simple_graph(node)

def simple_graph(node):
    """
    This uses matplotlib to show all the lidar and odom data collected without any optimization or correction.
    """
    fig = plt.figure()
    # Plot each LiDAR scan
    for i in range(len(node.map_x)):
        plt.scatter(node.map_x[i], node.map_y[i], color='b', alpha=0.3)
    # Plot all odom data
    plt.scatter(node.map_neatox, node.map_neatoy, color='r')
    # Plot the robot's start point in yellow
    plt.scatter(node.map_neatox[0], node.map_neatoy[0], color='y')
    plt.show()
    time.sleep(5) # show the graph for 5 seconds
    plt.close()

def plot_transform(node):
    """
    This function shows the lidar data from the first and last scan (the loop closure)
    and applies the transform from ICP to the last scan. It plots the scans before and after ICP.
    """
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
    """
    This applies the a rigid tranform to a set of LiDAR scans. We use it to adjust the
    loop closure scan after ICP gives us a transform.
    """
    # Calculate the transformed scan
    # Make C a homogeneous representation of B (later scan)
    C = np.ones((len(scan), 3))
    C[:,0:2] = scan

    # Apply transform to C
    C = np.dot(node.transform, C.T).T
    res = np.rot90(C, 3)
    return res

class NeatoController():
    """
    This class encompasses multiple behaviors for the simulated neato. They allow the robot to be teleoped
    or driven in a square, and handle the collecting of data.
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
        self.data = {"odom":[],"scans":[],"closures":[]}


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
        """
        Lets you drive a neato with wasd!
        """
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
        """
        Just drives the neato clockwise in a square.
        """
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
        """
        Runs each time the neato takes a lidar scan.saves the x and y coordinates of each
        laser point in the odom frame to self.map_x and self.map_y respectively.
        """
        current_mapx = []
        current_mapy = []
        pose_array = []
        for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
            #print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
            current_mapx.append(p[0])
            current_mapy.append(p[1])
            pose_array.append([p[0],p[1],p[2]])
        self.data["scans"].append(pose_array)
        self.map_x.append(current_mapx)
        self.map_y.append(current_mapy)
        # map_x and map_y are lists of lists of three values.

    def process_odom(self,msg):
        """
        Runs each time the neato recieves odom data and saves the x and y coordinates
        of the robot's position to self.map_neatox and self.map_neatoy.

        Also, if the odometry indicates that the robot has left its starting position and
        is not back within 10 cm of it, it records the index of that scan as a loop closure and stops
        recording data.

        Once it's recorded a loop closure, it runs ICP on the first and last scans and saves the
        resulting transform to self.tranform
        """
        #get our x and y position relative to the world origin"
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.init_x == None:
            self.init_x = self.x
        if self.init_y == None:
            self.init_y = self.y

        # Orientation of the neato according to global reference frame (odom)
        self.rotation = 180 - math.degrees(euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0])
        self.map_neatox.append(self.x)
        self.map_neatoy.append(self.y)

        self.data["odom"].append([self.x, self.y, 0])

        distance = np.sqrt((self.x - self.init_x)**2 + (self.y - self.init_y)**2)

        if distance > self.starting_threshold and not self.moved_flag: # if we have left the starting area for the first time
            self.moved_flag = True

        if distance < self.starting_threshold and self.moved_flag: # if we are returning to the starting area for the first time
            print('Oh boy thats a loop closure')
            self.moved_flag = False
            self.index_saved = len(self.map_x)-1
            # save the idex of the loop closure to the data
            self.data["closures"] =  [0,self.index_saved]
            # Both scans should be in format: (361,2)
            self.old_scan = np.rot90(np.array((self.map_x[0], self.map_y[0])))
            self.new_scan = np.rot90(np.array((self.map_x[self.index_saved],self.map_y[self.index_saved])))
            # Use ICP to get 3x3 transformation matrix
            t,d,i = icp(self.old_scan,self.new_scan)

            node.transform = t


if __name__ == '__main__':
    node = NeatoController() # Initialize the robot
    node.run()
    atexit.register(exit_handler,node) # when the program ends, save the data we collected
