#!/usr/bin/env python2
import sensor_msgs.point_cloud2 as pcl2
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import json
import sys
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from std_msgs.msg import Header
import tf
import os
import rosbag
rospy.init_node('revise', anonymous=True)
pub = rospy.Publisher('detection', MarkerArray, queue_size=100)
lidar_pub = rospy.Publisher('lidar', PointCloud2, queue_size=100)
msgs = []
bag = rosbag.Bag('/home/biomotion/data/datasets/nctu/Second.bag')#read bag
for topic, msg, t in bag.read_messages('/points_raw'):
	msgs.append(msg)
folder_path = '/home/biomotion/bnn-tracking/nctu_bonus/result2/'# The data folder

ids = []
seq = 0
for filename in sorted(os.listdir(folder_path)):          
    resultsf = open(folder_path + filename, 'r')
    results = json.load(resultsf)
    file_list = []
    markerArray = MarkerArray()
    count =0
    for result in results:
        marker = Marker()
        marker.header.frame_id='map'
        marker.header.stamp = rospy.Time.from_sec(float(result['timestamp'])/1000000000.0)
        marker.ns = 'object'
        marker.id = count
        marker.lifetime = rospy.Duration(0.23)#The lifetime of the bounding-box, you can modify it according to the power of your machine.
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = float(result['length'])
        marker.scale.y = float(result['width'])
        marker.scale.z = float(result['height'])
        marker.color.b = 1.0
        marker.color.a = 0.5#The alpha of the bounding-box
        marker.pose.position.x = float(result['center']['x'])
        marker.pose.position.y = float(result['center']['y'])
        marker.pose.position.z = float(result['center']['z'])
        marker.pose.orientation.w = float(result['rotation']['w'])
        marker.pose.orientation.x = float(result['rotation']['x'])
        marker.pose.orientation.y = float(result['rotation']['y'])
        marker.pose.orientation.z = float(result['rotation']['z'])
        marker1 = Marker()
        marker1.header.frame_id='map'
        marker1.header.stamp = rospy.Time.from_sec(float(result['timestamp'])/1000000000.0)
        marker1.ns = 'text'
        marker1.id = count
        marker1.scale.z = 0.8#The size of the text
        marker1.color.b = 1.0
        marker1.color.g = 1.0
        marker1.color.r = 1.0
        marker1.color.a = 1.0
        marker1.pose.position.x = float(result['center']['x'])
        marker1.pose.position.y = float(result['center']['y'])
        marker1.pose.position.z = float(result['center']['z'])
        marker1.lifetime = rospy.Duration(0.23)
        marker1.type = Marker.TEXT_VIEW_FACING
        marker1.action = Marker.ADD
        #marker1.text = result['label_class'] #for visualize detection
        if result['track_label_uuid'] not in ids: #for visualize tracking---
            ids.append(result['track_label_uuid'])
            marker1.text = str(ids.index(result['track_label_uuid']))
        else:
            marker1.text = str(ids.index(result['track_label_uuid']))#----
        markerArray.markers.append(marker)
        markerArray.markers.append(marker1)
        count=count+1   
        ttime = marker1.header.stamp
    if markerArray:
        #print( filename)
        pub.publish(markerArray)
        lidar_pub.publish(msgs[seq])
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, -1.8), (0, 0, 0, 1), ttime, 'map', "velodyne")
        seq += 1
        rospy.sleep(0.2)
