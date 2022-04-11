#!/usr/bin/env python
print "importing libraries"

print "usage: python2 euroc2rosbag.py --folder MH_01 --output-bag MH_01.bag"

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv

print "import libraries success!"

#structure
# dataset/mav0/cam0/data/TIMESTAMP.png
# dataset/mav0/imu0/data.csv
# dataset/mav0/state_groundtruth_estimate0/data.csv

#setup the argument list
parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data.')
parser.add_argument('--folder',  metavar='folder', nargs='?', help='Data folder')
parser.add_argument('--output-bag', metavar='output_bag',  default="output.bag", help='ROS bag file %(default)s')

#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    image_files.append( os.path.join( path, f ) )
                    timestamps.append(os.path.splitext(f)[0]) 
    #sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [file[1] for file in sort_list]
    return image_files

def loadImageToRosMsg(filename):
    image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    
    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tostring()
    
    return rosimage, timestamp

def createImuMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    
    return rosimu, timestamp

def createOdometryMsg(timestamp_int, position, orientation, linear, angular):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )

    rosodom = Odometry()
    rosodom.header.stamp = timestamp
    rosodom.header.frame_id = "drone"
    # set the position
    rosodom.pose.pose.position.x = float(position[0])
    rosodom.pose.pose.position.y = float(position[1])
    rosodom.pose.pose.position.z = float(position[2])

    rosodom.pose.pose.orientation.w = float(orientation[0])
    rosodom.pose.pose.orientation.x = float(orientation[1])
    rosodom.pose.pose.orientation.y = float(orientation[2])
    rosodom.pose.pose.orientation.z = float(orientation[3])

    # set the velocity
    rosodom.child_frame_id = "drone/odom_local_ned"
    rosodom.twist.twist.linear.x = float(linear[0])
    rosodom.twist.twist.linear.y = float(linear[1])
    rosodom.twist.twist.linear.z = float(linear[2])

    rosodom.twist.twist.angular.x = float(angular[0])
    rosodom.twist.twist.angular.y = float(angular[1])
    rosodom.twist.twist.angular.z = float(angular[2])

    return rosodom, timestamp


#create the bag
try:
    bag = rosbag.Bag(parsed.output_bag, 'w')
    
    #write images
    camfolders = "/mav0/cam0/data"
    camdir = parsed.folder + camfolders
    image_files = getImageFilesFromDir(camdir)
    for image_filename in image_files:
        image_msg, timestamp = loadImageToRosMsg(image_filename)
        bag.write("/cam0/image_raw", image_msg, timestamp)

    #write imu data
    imufile = parsed.folder + "/mav0/imu0/data.csv"
    with open(imufile, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        headers = next(reader, None)
        for row in reader:
            imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
            bag.write("/imu0", imumsg, timestamp)

    #write odometry data
    odometryfile = parsed.folder + "/mav0/state_groundtruth_estimate0/data.csv"
    with open(odometryfile, "rb") as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        headers = next(reader, None)
        for row in reader:
            odometrymsg, timestamp = createOdometryMsg(row[0], row[1:4], row[4:8], row[8:11], row[11:14])
            bag.write("/odometry", odometrymsg, timestamp)

finally:
    bag.close()


