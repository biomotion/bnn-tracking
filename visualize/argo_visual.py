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
from plyfile import PlyData, PlyElement
import pandas as pd

pub = rospy.Publisher('detection', MarkerArray, queue_size=100)
lidar_pub = rospy.Publisher('lidar', PointCloud2, queue_size=100)

rospy.init_node('visualization_argo', anonymous=True)
#detection result path (The directory until you see lots of log folders)
#folder_path = '/media/ee904-pc4/fab370ee-ed64-44f9-b2fb-6927edf87ad6/desktop/dataset/argo/detections_v1.1b/argoverse_detections_2020/testing/'
#tracking result path (The directory until you see lots of log folders)
folder_path = '/home/ee904-pc4/Downloads/SDC/tracking/argoverse_cbgs_kf_tracker/baseline_result/test-split-track-preds-maxage15-minhits5-conf0.3/'
#lidar_data path (The directory until you see lots of log folders)
lidar_path = '/media/ee904-pc4/fab370ee-ed64-44f9-b2fb-6927edf87ad6/desktop/dataset/argo/argoverse-tracking/test/'
result_dirs = [d for d in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, d))]

while not rospy.is_shutdown():
    for dirs in result_dirs:
        ids = []
        for filename in sorted(os.listdir(folder_path + dirs + '/per_sweep_annotations_amodal/')):          
            resultsf = open(folder_path + dirs + '/per_sweep_annotations_amodal/' + filename, 'r')
            results = json.load(resultsf)
            count = 0
            tstamp = 0
            markerArray = MarkerArray()
            for result in results:
                marker = Marker()
                marker.header.frame_id='map'
                marker.header.stamp = rospy.Time.from_sec(result['timestamp']/100000000.0)
                marker.ns = 'object'
                marker.id = count
                marker.lifetime = rospy.Duration(0.23)#The lifetime of the bounding-box, you can modify it according to the power of your machine.
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.scale.x = result['length']
                marker.scale.y = result['width']
                marker.scale.z = result['height']
                marker.color.b = 1.0
                marker.color.a = 0.5#The alpha of the bounding-box
                marker.pose.position.x = result['center']['x']
                marker.pose.position.y = result['center']['y']
                marker.pose.position.z = result['center']['z']
                marker.pose.orientation.w = result['rotation']['w']
                marker.pose.orientation.x = result['rotation']['x']
                marker.pose.orientation.y = result['rotation']['y']
                marker.pose.orientation.z = result['rotation']['z']
                marker1 = Marker()
                marker1.header.frame_id='map'
                marker1.header.stamp = rospy.Time.from_sec(result['timestamp']/100000000.0)
                marker1.ns = 'text'
                marker1.id = count
                marker1.scale.z = 0.8#The size of the text
                marker1.color.b = 1.0
                marker1.color.g = 1.0
                marker1.color.r = 1.0
                marker1.color.a = 1.0
                marker1.pose.position.x = result['center']['x']
                marker1.pose.position.y = result['center']['y']
                marker1.pose.position.z = result['center']['z']
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
                tstamp = result['timestamp']
            for lidar in sorted(os.listdir(lidar_path + dirs + '/lidar/')):
                if str(tstamp) in lidar:
                    try:
                        plydata = PlyData.read(lidar_path + dirs + '/lidar/' + lidar)
                        data = plydata.elements[0].data  
                        data_pd = pd.DataFrame(data)  
                        p = np.zeros(data_pd.shape, dtype=np.float)  
                        property_names = data[0].dtype.names  
                        for i, name in enumerate(property_names):  
                            p[:, i] = data_pd[name]
                    except:
                        continue
                    header = Header()
                    header.stamp = rospy.Time.from_sec(tstamp/100000000.0)
                    header.frame_id = 'map'
                    fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.UINT32, 1),]
                    lidar_pub.publish(pcl2.create_cloud(header,fields,p[:,:4]))
            if markerArray:
                print(dirs + '/' + filename)
                pub.publish(markerArray)
            rospy.sleep(0.2)
    