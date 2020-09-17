#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
import tf
import xml.etree.ElementTree as ET

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from tf import transformations

from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_rviz_plugins.msg import Pictogram
from jsk_rviz_plugins.msg import PictogramArray

import std_msgs
import os.path

TRACKLET_PATH = '/home/mds/2011_09_26/2011_09_26_drive_0002_sync/tracklet_labels.xml'

kitti_data = None
pub_boxes = None

pictogram_texts = None
pub_pictograms = None

def run():
    global pub_boxes, pub_pictograms

    rospy.init_node('kitti_box_publisher', anonymous=True)

    rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, callback, queue_size=1)
    pub_boxes = rospy.Publisher("kitti_box", BoundingBoxArray, queue_size=1)
    pub_pictograms = rospy.Publisher('kitti_3d_labels', PictogramArray, queue_size=1)

    rospy.spin()


def callback(data):
    header = data.header
    frame = header.seq

    boxes = BoundingBoxArray()
    boxes.header = header

    texts = PictogramArray()
    texts.header = header

    if kitti_data.has_key(frame) == True:
        for b in kitti_data[frame]:
            b.header = header
            boxes.boxes.append(b)
                
            str = b
            rospy.loginfo(str)

    if pictogram_texts.has_key(frame) == True:
        for txt in pictogram_texts[frame]:
            txt.header = header
            texts.pictograms.append(txt)

            str = txt
            rospy.loginfo(str)

    pub_boxes.publish(boxes)
    pub_pictograms.publish(texts)

def readXML(file):
    tree = ET.parse(file)
    root = tree.getroot()

    item = root.findall('./tracklets/item')

    d = {}
    pictograms = {}

    for i, v in enumerate(item):
        h = float(v.find('h').text)
        w = float(v.find('w').text)
        l = float(v.find('l').text)
        frame = int(v.find('first_frame').text)
        size = Vector3(l, w, h)

        label = v.find('objectType').text
        
        pose = v.findall('./poses/item')

        for j, p in enumerate(pose):
            tx = float(p.find('tx').text)
            ty = float(p.find('ty').text)
            tz = float(p.find('tz').text)
            rz = float(p.find('rz').text)

            q = tf.transformations.quaternion_from_euler(0.0, 0.0, rz)

            b = BoundingBox()
            b.pose.position = Vector3(tx, ty, tz/2.0)
            b.pose.orientation = Quaternion(*q)
            b.dimensions = size
            b.label = i

            picto_text = Pictogram()
            picto_text.mode = Pictogram.STRING_MODE
            picto_text.pose.position = Vector3(tx, ty, -tz/2.0)
            q = tf.transformations.quaternion_from_euler(0.7, 0.0, -0.7)
            picto_text.pose.orientation = Quaternion(0.0, -0.5, 0.0, 0.5)
            picto_text.size = 1
            picto_text.color = std_msgs.msg.ColorRGBA(1, 1, 1, 1)
            picto_text.character = label

            if d.has_key(frame + j) == True:
                d[frame + j].append(b)
                pictograms[frame + j].append(picto_text)
            else:
                d[frame + j] = [b]
                pictograms[frame + j] = [picto_text]

    return d, pictograms


if __name__ == '__main__':

    xml_file = TRACKLET_PATH
    kitti_data, pictogram_texts = readXML(xml_file)
    run()
