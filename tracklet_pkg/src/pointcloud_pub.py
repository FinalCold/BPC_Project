#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
import tf
import xml.etree.ElementTree as ET
import math

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from tf import transformations

from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_rviz_plugins.msg import Pictogram
from jsk_rviz_plugins.msg import PictogramArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import std_msgs
import os.path

TRACKLET_PATH = '/home/mds/2011_09_26/2011_09_26_drive_0101_sync/result.xml'
f = open('/home/mds/catkin_ws/src/tracklet_pkg/src/predict_result_AB3DMOT_5_1_0000(3).txt', 'r')
# f = open('/home/mds/catkin_ws/src/tracklet_pkg/src/track_history_0000_10(1).txt', 'r')

bounding_boxes = None
pub_boxes = None

pictogram_texts = None
pub_pictograms = None

marker_arrows = None
pub_arrows = None

marker_paths = None
pub_paths = None

def run():
    global pub_boxes, pub_pictograms, pub_arrows, pub_paths

    rospy.init_node('kitti_box_publisher', anonymous=False)

    rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, callback, queue_size=1)
    pub_boxes = rospy.Publisher("bboxs", BoundingBoxArray, queue_size=1)
    pub_pictograms = rospy.Publisher('labels', PictogramArray, queue_size=1)
    pub_arrows = rospy.Publisher('arrows', MarkerArray, queue_size=1)
    pub_paths = rospy.Publisher('paths', MarkerArray, queue_size=1)
    print("running")

    rospy.spin()

def callback(data):
    header = data.header
    frame = header.seq

    boxes = BoundingBoxArray()
    boxes.header = header

    texts = PictogramArray()
    texts.header = header

    arrows = MarkerArray()

    paths = MarkerArray()

    if bounding_boxes.has_key(frame) == True:
        for bbox in bounding_boxes[frame]:
            bbox.header = header
            boxes.boxes.append(bbox)

        maxbox = len(boxes.boxes)
        # print("maxbox: %s"%maxbox)

    if pictogram_texts.has_key(frame) == True:
        for txt in pictogram_texts[frame]:
            txt.header = header
            texts.pictograms.append(txt)

            # str = txt
            # rospy.loginfo(str)
    
    if marker_arrows.has_key(frame) == True:
        for arrow in marker_arrows[frame]:
            arrow.header = header
            arrows.markers.append(arrow)

        maxarrow = len(arrows.markers)

        # print("maxarrow: %s"%(maxarrow))
    if marker_paths.has_key(frame) == True:
        for predict in marker_paths[frame]:
            predict.header = header
            paths.markers.append(predict)
            # print(predict)

    pub_boxes.publish(boxes)
    pub_pictograms.publish(texts)
    pub_arrows.publish(arrows)
    pub_paths.publish(paths)

def prediction():
    lines_li = []
    lines = f.readlines()

    for i in range(len(lines)):
        lines_li.append(lines[i])

    line = []
    backup = []
    l = []

    for i in range(len(lines_li)):
        l = lines_li[i].split()
        line.append(l)
        backup.append(l)

    row = len(line)
    column = len(line[0])

    # print(backup)
    # print(row)
    # print(column)

    for i in range(row):
        for j in range(column):
            if j < 2:
                backup[i][j] = int(backup[i][j])
            else:
                if backup[i][j] == "nan":
                    backup[i][j] = 999
                else:
                    backup[i][j] = float(backup[i][j])

    backup.sort(key=lambda x: (x[0],x[1]))

    with open("/home/mds/Downloads/sorted.txt", 'w') as w:
        for data in backup:
            w.write("%s\n" % data)

    # print(backup)

    predict = {}

    p = Point()

    last_frame = backup[-1][0]
    frame_cnt = 3
    i = 0
    # print(last_frame)
    cnt = 0
    res = 0
    # 동일한 프레임에 들어있는 track
    while(cnt < 10000):
        cnt += 1
        if backup[cnt][1] != backup[cnt+1][1]:
            cnt += 1
            res = cnt
            break

    # print(res)

    while(i < row):
        mark = Marker()

        frame = current_frame = backup[i][0]

        for a in range(res):
            p = Point()
            p.x, p.y, p.z = backup[a+i][2], backup[a+i][3], -1.5
            mark.points.append(p)
        # print(p)

        mark.type = Marker.LINE_STRIP
        mark.action = Marker.ADD
        mark.lifetime = rospy.Duration(0.1)
        mark.scale.x = 0.2
        mark.color = ColorRGBA(0, 1, 0, 1)
        mark.id = backup[i][1]

        i += res
        frame_cnt += 1

        if predict.has_key(frame) == True:
           predict[frame].append(mark)
        else:
           predict[frame] = [mark]

        # print(i)
        # print(frame_cnt)

    return predict

def readXML(file):
    tree = ET.parse(file)
    root = tree.getroot()

    item = root.findall('./tracklets/item')

    bboxs = {}
    pictograms = {}
    arrows = {}
    paths = {}

    for i, v in enumerate(item):
        h = float(v.find('h').text)
        w = float(v.find('w').text)
        l = float(v.find('l').text)
        frame = int(v.find('first_frame').text)
        size = Vector3(l, w, h)

        label = v.find('objectType').text
        track_id = int(v.find('trackid').text)
        
        pose = v.findall('./poses/item')

        for j, p in enumerate(pose):
            tx = float(p.find('tx').text)
            ty = float(p.find('ty').text)
            tz = float(p.find('tz').text) + 0.85
            rz = float(p.find('rz').text)

            q = tf.transformations.quaternion_from_euler(0.0, 0.0, rz)

            bbox = BoundingBox()
            picto = Pictogram()
            arrow = Marker()

            bbox = viz_bbox(Vector3(tx, ty, tz), q, size, track_id)
            picto = viz_picto(Vector3(tx, ty, -tz/2.0), q, label)
            arrow = viz_arrow(Vector3(tx, ty, tz), q, track_id)


            if bboxs.has_key(frame + j) == True:
                bboxs[frame + j].append(bbox)
                pictograms[frame + j].append(picto)
                arrows[frame + j].append(arrow)

            else:
                bboxs[frame + j] = [bbox]
                pictograms[frame + j] = [picto]
                arrows[frame + j] = [arrow]


    return bboxs, pictograms, arrows

def viz_bbox(position, q, size, i):
    bbox = BoundingBox()
    bbox.pose.position = position
    bbox.pose.orientation = Quaternion(*q)
    bbox.dimensions = size
    bbox.label = i

    return bbox

def viz_picto(position, q, label):
    picto = Pictogram()
    picto.mode = Pictogram.STRING_MODE
    picto.pose.position = position
    picto.pose.orientation = Quaternion(0.0, -0.5, 0.0, 0.5)
    picto.size = 3
    picto.color = ColorRGBA(1, 1, 1, 1)
    picto.character = label

    return picto

def viz_arrow(position, q, i):
    arrow = Marker()
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.lifetime = rospy.Duration(0.1)
    arrow.pose.position = position
    arrow.pose.orientation = Quaternion(*q)
    arrow.scale = Vector3(10, 0.3, 0.3)
    arrow.color = ColorRGBA(1, 0, 0, 1)
    arrow.id = i

    return arrow

if __name__ == '__main__':

    xml_file = TRACKLET_PATH
    bounding_boxes, pictogram_texts, marker_arrows = readXML(xml_file)
    marker_paths = prediction()
    run()
