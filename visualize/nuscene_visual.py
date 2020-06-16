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

 
data_path='/media/ee904-pc4/fab370ee-ed64-44f9-b2fb-6927edf87ad6/desktop/dataset/te/test/'#nuscene data path

resultsf = open('megvii_test.json', 'r')#put the detection/tracking result in the current directory
samplesf = open(data_path + 'v1.0-test/sample.json', 'r')
datasf = open(data_path + 'v1.0-test/sample_data.json', 'r')
tfsf = open(data_path + 'v1.0-test/ego_pose.json', 'r')
calibsf = open(data_path + 'v1.0-test/calibrated_sensor.json', 'r')
calibs = json.load(calibsf)
results = json.load(resultsf)
samples = json.load(samplesf)
datas = json.load(datasf)
tfs = json.load(tfsf)

pub = rospy.Publisher('detection', MarkerArray, queue_size=100)#publish 2 topics
lidar_pub = rospy.Publisher('lidar', PointCloud2, queue_size=100)

rospy.init_node('visualization', anonymous=True)
path = data_path + 'v1.0-test_blobs/'
while not rospy.is_shutdown():    
    for sample in samples:#Each timestamp
        token = sample['token']
        markerArray = MarkerArray()
        count = 0
        for result in results['results'][token]:#Each object in a timestamp
            if token == result['sample_token']:
                marker = Marker()
                marker.header.frame_id='map'
                marker.header.stamp = rospy.Time.from_sec(sample['timestamp']/1000000.0)
                marker.ns = 'object'
                marker.id = count
                marker.lifetime = rospy.Duration(0.5)#The lifetime of the bounding-box, you can modify it according to the power of your machine.
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.scale.x = result['size'][1]
                marker.scale.y = result['size'][0]
                marker.scale.z = result['size'][2]
                marker.color.b = 1.0
                marker.color.a = 0.5#you can modify the alpha here
                marker.pose.position.x = result['translation'][0]
                marker.pose.position.y = result['translation'][1]
                marker.pose.position.z = result['translation'][2]
                marker.pose.orientation.w = result['rotation'][0]
                marker.pose.orientation.x = result['rotation'][1]
                marker.pose.orientation.y = result['rotation'][2]
                marker.pose.orientation.z = result['rotation'][3]

                marker1 = Marker()
                marker1.header.frame_id='map'
                marker1.header.stamp = rospy.Time.from_sec(sample['timestamp']/1000000.0)
                marker1.ns = 'text'
                marker1.id = count
                marker1.scale.z = 1#The size of the text
                marker1.color.b = 1.0
                marker1.color.g = 1.0
                marker1.color.r = 1.0
                marker1.color.a = 1.0
                marker1.pose.position.x = result['translation'][0]
                marker1.pose.position.y = result['translation'][1]
                marker1.pose.position.z = result['translation'][2]
                marker1.lifetime = rospy.Duration(0.5)
                marker1.type = Marker.TEXT_VIEW_FACING
                marker1.action = Marker.ADD
                #marker1.text = result['tracking_id']#for visualize tracking
                marker1.text = result['detection_name']#for visualize detection
                markerArray.markers.append(marker)
                markerArray.markers.append(marker1)
                count=count+1
        for data in datas:#Lidar data in a timestamp
            if data['sample_token'] == token and ('samples/LIDAR_TOP' in data['filename']):
                try:
                    p = np.fromfile(path+data['filename'], dtype=np.float32).reshape(-1, 5)
                except:
                    continue
                header = Header()
                header.stamp = rospy.Time.from_sec(sample['timestamp']/1000000.0)
                header.frame_id = 'nuscenes_lidar'
                p[:,3] = p[:,3] * 255
                fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.UINT32, 1),]
                lidar_pub.publish(pcl2.create_cloud(header,fields,p[:,:4]))
                calibrated_sensor_token = data['calibrated_sensor_token']
        for calib in calibs:#Calibration transform in a timestamp
            if calib['token'] == calibrated_sensor_token:
                br = tf.TransformBroadcaster()
                br.sendTransform((calib['translation'][0], calib['translation'][1], calib['translation'][2]), (calib['rotation'][1], calib['rotation'][2], calib['rotation'][3], calib['rotation'][0]), rospy.Time.from_sec(sample['timestamp']/1000000.0), "nuscenes_lidar", 'car')
        for tff in tfs:#Ego pose in a timestamp
            if tff['timestamp'] == sample['timestamp']:
                br = tf.TransformBroadcaster()
                br.sendTransform((tff['translation'][0], tff['translation'][1], tff['translation'][2]), (tff['rotation'][1], tff['rotation'][2], tff['rotation'][3], tff['rotation'][0]), rospy.Time.from_sec(sample['timestamp']/1000000.0), 'car', "map")
        if markerArray:#publish the bounding-boxes
            print(sample['timestamp'])
            pub.publish(markerArray)

        rospy.sleep(0.5)


